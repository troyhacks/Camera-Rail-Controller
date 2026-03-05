// ── main_s3.cpp — Waveshare ESP32-S3-Touch-LCD-7 ─────────────────────────────
// Camera Rail Controller — S3 main board
//
// Motor:   MKS SERVO42D closed-loop stepper, UART half-duplex
//            H3 connector → GPIO43 (TX) / GPIO44 (RX), SW1 = external
// Camera:  PELCO-D PTZ control over RS485
//            J7 connector → IO15 (TX) / IO16 (RX), auto DE/RE (SP3485EN)
// Display: 7" 800×480 RGB LCD + GT911 capacitive touch (Arduino_GFX_Library)
// Remote:  XIAO ESP32-C3 over ESP-NOW — protocol identical to main.cpp
//
// Serial port assignments (set ARDUINO_USB_CDC_ON_BOOT=1 in build_flags):
//   Serial  → USB CDC via native USB (IO19/IO20)  — debug output
//   Serial1 → SERVO42D UART  (RX=GPIO44, TX=GPIO43), 38400 baud
//   Serial2 → RS485 PELCO-D  (RX=IO16,   TX=IO15),   9600 baud
//
// Homing: uses SERVO42D built-in limit switch inputs (home + end stop wired
//   to the driver PCB).  Phase 1 issues the driver's go-home command; the
//   driver seeks the home switch and sets its own zero.  Phase 2 issues a
//   long-distance move; the driver stops on the end switch and we read back
//   the position to calibrate travelSteps.  No ESP32 GPIO required.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>

// ── Motion parameters ────────────────────────────────────────────────────────
// Match SERVO42D microstep DIP switch setting (default = 16 microsteps)
const int MICROSTEPS    = 16;
const int STEPS_PER_REV = 200 * MICROSTEPS;   // 3200 pulses/rev
const int BUFFER_STEPS  = STEPS_PER_REV / 4;  // 800 pulses — margin inside each end

// SERVO42D speed/accel are 0–127 / 0–255 (arbitrary driver units)
const uint8_t HOME_SPEED = 20;   // Slow enough that stall detection is reliable
const uint8_t RUN_SPEED  = 100;
const uint8_t HOME_ACCEL = 30;
const uint8_t RUN_ACCEL  = 100;

// Motion-complete detection: position must change less than this many pulses
// for STALL_CONFIRM_MS continuously before we consider the driver stopped.
const long          STALL_THRESHOLD_PULSES = 5;
const unsigned long STALL_CONFIRM_MS       = 250;

// ── Pin assignments ───────────────────────────────────────────────────────────
// No external limit switches — homing uses SERVO42D stall detection.
// SERVO42D UART (H3 connector — SW1 must be flipped to external position)
const int SERVO_RX   = 44;
const int SERVO_TX   = 43;
const int SERVO_BAUD = 38400;

// RS485 (J7 connector — auto direction, no DE/RE pin needed)
const int RS485_RX   = 16;
const int RS485_TX   = 15;
const int RS485_BAUD = 9600;

// I2C — shared by GT911 touch and CH422G I/O expander
const int I2C_SDA = 8;
const int I2C_SCL = 9;

// ── State ─────────────────────────────────────────────────────────────────────
long travelSteps = 0;
bool is_homed    = false;
long currentPos  = 0;   // Cached position in pulses (polled from SERVO42D)
long targetPos   = 0;   // Last commanded target

enum ControlMode { MODE_SHUTTLE, MODE_JOYSTICK, MODE_POT, MODE_SERIAL };
ControlMode currentMode = MODE_SERIAL;

const char *MODE_NAMES[] = { "SHUTTLE", "JOG", "DIAL", "SERIAL" };

// ── SERVO42D driver ───────────────────────────────────────────────────────────
// MKS SERVO42D v2.0 UART protocol
// Frame: [addr][cmd][data...][crc]   CRC = (sum of addr+cmd+all data bytes) & 0xFF
// Half-duplex: TX echo appears on RX — we discard it before reading the response.
//
// VERIFY against your physical unit if behaviour is unexpected:
//   Enable:        E0 F3 AB [CRC]
//   Disable:       E0 F3 7B [CRC]
//   Emergency stop:E0 F7 98         (0x98 is a fixed stop code, no CRC byte)
//   Set zero:      E0 0A 6D [CRC]
//   Go home:       E0 91 [CRC]      (driver moves to home switch, sets zero)
//   Read position: E0 36 [CRC]  → E0 36 [s32 MSB..LSB] [CRC]  (signed pulses)
//   Move absolute: E0 FD [speed] [accel] [p3 p2 p1 p0] [mode=1] [CRC]

#define SERVO_ADDR 0xE0

HardwareSerial ServoSerial(1);   // UART1

static uint8_t servo_crc(const uint8_t *buf, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++) s += buf[i];
  return (uint8_t)(s & 0xFF);
}

// Write bytes and discard the half-duplex TX echo from RX
static void servo_send(const uint8_t *buf, int len) {
  ServoSerial.write(buf, len);
  ServoSerial.flush();
  unsigned long t0 = millis();
  int got = 0;
  while (got < len && millis() - t0 < 20) {
    if (ServoSerial.available()) { ServoSerial.read(); got++; }
  }
}

// Read a response with timeout; returns byte count received
static int servo_recv(uint8_t *buf, int maxLen, unsigned long timeoutMs = 30) {
  unsigned long t0 = millis();
  int got = 0;
  while (got < maxLen && millis() - t0 < timeoutMs) {
    if (ServoSerial.available()) buf[got++] = ServoSerial.read();
  }
  return got;
}

void servo_enable() {
  uint8_t cmd[] = { SERVO_ADDR, 0xF3, 0xAB, 0x00 };
  cmd[3] = servo_crc(cmd, 3);
  servo_send(cmd, 4);
  delay(5);
}

void servo_disable() {
  uint8_t cmd[] = { SERVO_ADDR, 0xF3, 0x7B, 0x00 };
  cmd[3] = servo_crc(cmd, 3);
  servo_send(cmd, 4);
  delay(5);
}

void servo_emergency_stop() {
  // 0x98 is the fixed verification byte for the stop command — no CRC follows
  uint8_t cmd[] = { SERVO_ADDR, 0xF7, 0x98 };
  servo_send(cmd, 3);
}

// Set the current encoder position as the zero / home reference
void servo_set_zero() {
  uint8_t cmd[] = { SERVO_ADDR, 0x0A, 0x6D, 0x00 };
  cmd[3] = servo_crc(cmd, 3);
  servo_send(cmd, 4);
  delay(10);
}

// Command the SERVO42D to execute its built-in go-home routine.
// The driver moves toward the home switch at its configured homing speed,
// stops when the switch triggers, and internally sets position to zero.
// Call wait_for_motion_complete() after this to block until it finishes.
void servo_go_home() {
  uint8_t cmd[] = { SERVO_ADDR, 0x91, 0x00 };
  cmd[2] = servo_crc(cmd, 2);
  servo_send(cmd, 3);
}

// Read current absolute position in pulses (signed 32-bit from home).
// Returns LONG_MIN on comm failure.
long servo_read_position() {
  uint8_t cmd[] = { SERVO_ADDR, 0x36, 0x00 };
  cmd[2] = servo_crc(cmd, 2);
  servo_send(cmd, 3);

  uint8_t resp[7];
  int n = servo_recv(resp, 7);
  if (n < 7 || resp[0] != SERVO_ADDR || resp[1] != 0x36) return LONG_MIN;

  int32_t pos = ((int32_t)resp[2] << 24) | ((int32_t)resp[3] << 16)
              | ((int32_t)resp[4] <<  8) |  (int32_t)resp[5];
  return (long)pos;
}

// Command the SERVO42D to move to an absolute pulse position.
// Returns true if the driver acknowledged the command.
bool servo_move_to(long pulses, uint8_t speed, uint8_t accel) {
  uint32_t p = (uint32_t)(pulses < 0 ? 0 : pulses);   // clamp negative to 0
  uint8_t cmd[10];
  cmd[0] = SERVO_ADDR;
  cmd[1] = 0xFD;
  cmd[2] = speed;
  cmd[3] = accel;
  cmd[4] = (uint8_t)(p >> 24);
  cmd[5] = (uint8_t)(p >> 16);
  cmd[6] = (uint8_t)(p >>  8);
  cmd[7] = (uint8_t)(p      );
  cmd[8] = 0x01;   // 1 = absolute position mode
  cmd[9] = servo_crc(cmd, 9);
  servo_send(cmd, 10);
  targetPos = pulses;

  uint8_t resp[3];
  int n = servo_recv(resp, 3);
  return (n >= 2 && resp[0] == SERVO_ADDR);
}

// True when SERVO42D position is within 20 pulses of the last target
bool servo_motion_complete() {
  return (abs(currentPos - targetPos) < 20);
}

// ── PELCO-D camera control ────────────────────────────────────────────────────
// Standard PELCO-D frame: [0xFF][addr][cmd1][cmd2][data1][data2][checksum]
// Checksum = (addr + cmd1 + cmd2 + data1 + data2) & 0xFF

#define PELCO_ADDR 0x01

HardwareSerial CameraSerial(2);  // UART2

void pelco_send(uint8_t cmd1, uint8_t cmd2, uint8_t data1, uint8_t data2) {
  uint8_t chk = (uint8_t)((PELCO_ADDR + cmd1 + cmd2 + data1 + data2) & 0xFF);
  uint8_t frame[7] = { 0xFF, PELCO_ADDR, cmd1, cmd2, data1, data2, chk };
  CameraSerial.write(frame, 7);
}

void pelco_stop()                   { pelco_send(0x00, 0x00, 0x00, 0x00); }
void pelco_pan_right(uint8_t spd)   { pelco_send(0x00, 0x02, spd,  0x00); }
void pelco_pan_left(uint8_t spd)    { pelco_send(0x00, 0x04, spd,  0x00); }
void pelco_tilt_up(uint8_t spd)     { pelco_send(0x00, 0x08, 0x00, spd ); }
void pelco_tilt_down(uint8_t spd)   { pelco_send(0x00, 0x10, 0x00, spd ); }
void pelco_zoom_in()                { pelco_send(0x00, 0x20, 0x00, 0x00); }
void pelco_zoom_out()               { pelco_send(0x00, 0x40, 0x00, 0x00); }
void pelco_preset_call(uint8_t n)   { pelco_send(0x00, 0x07, 0x00, n   ); }

// ── Display ───────────────────────────────────────────────────────────────────
// Waveshare ESP32-S3-Touch-LCD-7: 800×480, 16-bit RGB, GT911 touch (I2C 0x5D)
// Init parameters confirmed from Waveshare schematic and working DMX code.

// LovyanGFX configuration for Waveshare ESP32-S3-Touch-LCD-7
// Bus_RGB pin order: d0=B0..d4=B4, d5=G0..d10=G5, d11=R0..d15=R4 (RGB565 LSB-first)
class LGFX : public lgfx::LGFX_Device {
    lgfx::Bus_RGB    _bus_instance;
    lgfx::Panel_RGB  _panel_instance;
public:
    LGFX(void) {
        {
            auto cfg = _bus_instance.config();
            cfg.panel          = &_panel_instance;
            cfg.pin_d0  = 14; cfg.pin_d1  = 38; cfg.pin_d2  = 18; cfg.pin_d3  = 17; cfg.pin_d4  = 10; // B0-B4
            cfg.pin_d5  = 39; cfg.pin_d6  =  0; cfg.pin_d7  = 45; cfg.pin_d8  = 48; cfg.pin_d9  = 47; cfg.pin_d10 = 21; // G0-G5
            cfg.pin_d11 =  1; cfg.pin_d12 =  2; cfg.pin_d13 = 42; cfg.pin_d14 = 41; cfg.pin_d15 = 40; // R0-R4
            cfg.pin_henable   = 5;          // DE
            cfg.pin_vsync     = 3;          // VSYNC
            cfg.pin_hsync     = 46;         // HSYNC
            cfg.pin_pclk      = 7;          // PCLK
            cfg.freq_write    = 16000000;
            cfg.hsync_polarity    = 0; cfg.hsync_front_porch = 210; cfg.hsync_pulse_width = 30; cfg.hsync_back_porch = 16;
            cfg.vsync_polarity    = 0; cfg.vsync_front_porch =  22; cfg.vsync_pulse_width = 13; cfg.vsync_back_porch = 10;
            cfg.pclk_idle_high    = true;   // pclk_neg=1 in Arduino_GFX
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.memory_width  = cfg.panel_width  = 800;
            cfg.memory_height = cfg.panel_height = 480;
            cfg.offset_x = cfg.offset_y = 0;
            _panel_instance.config(cfg);
        }
        setPanel(&_panel_instance);
    }
};

static LGFX lcd;
LGFX *gfx = &lcd;

// Palette
#define C_BG       (uint16_t)0x0000   // Black
#define C_FG       (uint16_t)0xFFFF   // White
#define C_GREEN    (uint16_t)0x07E0   // Bright green
#define C_RED      (uint16_t)0xF800   // Red
#define C_GREY     (uint16_t)0x4208   // Dark grey (header/footer fill)
#define C_BTN      (uint16_t)0x2945   // Inactive button
#define C_BTN_ACT  (uint16_t)0x051F   // Active/selected button
#define C_BAR_BG   (uint16_t)0x2104   // Position bar background

// Layout constants
#define HDR_H   50                    // Header bar height
#define BTN_H   70                    // Bottom button bar height
#define BTN_Y   (480 - BTN_H)         // Y origin of bottom bar
#define BODY_Y  HDR_H                 // Y origin of body area
#define BODY_H  (480 - HDR_H - BTN_H) // Body height = 360px

// ── GT911 touch (minimal direct I2C, no external library) ────────────────────
#define GT911_ADDR  0x5D
#define GT911_STAT  0x814E
#define GT911_PT1   0x8150

static void gt911_write_reg(uint16_t reg, uint8_t val) {
  Wire.beginTransmission(GT911_ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  Wire.endTransmission();
}

// Returns number of touch points; fills *tx, *ty with first point coordinates.
// Returns 0 when no touch is active.
static int gt911_read(int *tx, int *ty) {
  Wire.beginTransmission(GT911_ADDR);
  Wire.write((uint8_t)(GT911_STAT >> 8));
  Wire.write((uint8_t)(GT911_STAT & 0xFF));
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)GT911_ADDR, (uint8_t)1);
  uint8_t stat = Wire.read();
  int n = stat & 0x0F;
  if (!(stat & 0x80) || n == 0) return 0;

  Wire.beginTransmission(GT911_ADDR);
  Wire.write((uint8_t)(GT911_PT1 >> 8));
  Wire.write((uint8_t)(GT911_PT1 & 0xFF));
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)GT911_ADDR, (uint8_t)6);
  Wire.read();                               // track id
  uint8_t xl = Wire.read(), xh = Wire.read();
  uint8_t yl = Wire.read(), yh = Wire.read();
  Wire.read();                               // size

  *tx = (int)xl | ((int)xh << 8);
  *ty = (int)yl | ((int)yh << 8);
  gt911_write_reg(GT911_STAT, 0x00);        // clear buffer-ready flag
  return n;
}

// ── Draw ──────────────────────────────────────────────────────────────────────

static unsigned long last_draw_ms = 0;
#define DRAW_INTERVAL_MS 100   // ~10 fps

// Draws a filled rounded rectangle with centred label text (size 2 = 12×16 px/char)
static void draw_button(int x, int y, int w, int h, const char *label, uint16_t fill) {
  gfx->fillRoundRect(x, y, w, h, 6, fill);
  gfx->setTextColor(C_FG);
  gfx->setTextSize(2);
  int tx = x + (w - (int)strlen(label) * 12) / 2;
  int ty = y + (h - 16) / 2;
  gfx->setCursor(tx, ty);
  gfx->print(label);
}

void draw_display(bool link_ok, uint16_t pos_pct_tenths) {
  if (millis() - last_draw_ms < DRAW_INTERVAL_MS) return;
  last_draw_ms = millis();

  gfx->fillScreen(C_BG);

  // ── Header bar ──────────────────────────────────────────────────────────────
  gfx->fillRect(0, 0, 800, HDR_H, C_GREY);

  gfx->setTextSize(2);
  gfx->setTextColor(C_FG);
  gfx->setCursor(10, 14);
  gfx->print("Mode: ");
  gfx->setTextColor(C_GREEN);
  gfx->print(MODE_NAMES[currentMode]);

  gfx->setTextColor(link_ok ? C_GREEN : C_RED);
  gfx->setCursor(580, 14);
  gfx->print(link_ok ? "[ REMOTE OK ]" : "[ NO REMOTE ]");

  // ── Position bar (body top) ──────────────────────────────────────────────────
  const int BAR_X = 40, BAR_Y = BODY_Y + 30, BAR_W = 720, BAR_H = 44;
  gfx->fillRect(BAR_X, BAR_Y, BAR_W, BAR_H, C_BAR_BG);
  if (is_homed && pos_pct_tenths > 0) {
    int fill = (int)((long)BAR_W * pos_pct_tenths / 1000);
    gfx->fillRect(BAR_X, BAR_Y, fill, BAR_H, C_GREEN);
  }
  gfx->drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, C_FG);

  gfx->setTextSize(2);
  gfx->setTextColor(C_FG);
  gfx->setCursor(BAR_X, BAR_Y + BAR_H + 8);
  if (is_homed) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Position: %d.%d%%   (%ld / %ld pulses)",
             pos_pct_tenths / 10, pos_pct_tenths % 10, currentPos, travelSteps);
    gfx->print(buf);
  } else {
    gfx->setTextColor(C_RED);
    gfx->print(is_homed ? "" : "Not homed — run HOME");
  }

  // Motor status line
  gfx->setTextSize(2);
  gfx->setTextColor(C_FG);
  gfx->setCursor(40, BODY_Y + 130);
  gfx->print("Motor: ");
  gfx->setTextColor(is_homed ? C_GREEN : C_RED);
  gfx->print(is_homed ? "HOMED" : "NEEDS HOMING");

  // ── PELCO-D camera controls ──────────────────────────────────────────────────
  // Arranged as a D-pad + zoom column.  Touch coordinates match handleTouch().
  //
  //   [TILT UP ]
  //   [PAN LFT] [  STOP  ] [PAN RGT]   [ZOOM IN ]
  //   [TILT DN ]                        [ZOOM OUT]

  const int CAM_TOP  = BODY_Y + 180;
  const int BW = 140, BH = 50, GAP = 8;

  // Row offsets (centre column x = 330)
  const int CX = 330, RY0 = CAM_TOP, RY1 = CAM_TOP + BH + GAP, RY2 = CAM_TOP + 2 * (BH + GAP);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->setCursor(40, CAM_TOP - 18);
  gfx->print("Camera (PELCO-D)");

  draw_button(CX,               RY0, BW, BH, "TILT UP",   C_BTN);
  draw_button(CX - BW - GAP,    RY1, BW, BH, "PAN LEFT",  C_BTN);
  draw_button(CX,               RY1, BW, BH, "STOP",      C_RED);
  draw_button(CX + BW + GAP,    RY1, BW, BH, "PAN RIGHT", C_BTN);
  draw_button(CX,               RY2, BW, BH, "TILT DOWN", C_BTN);
  draw_button(CX + 2*(BW+GAP),  RY1, BW, BH, "ZOOM IN",   C_BTN);
  draw_button(CX + 2*(BW+GAP),  RY2, BW, BH, "ZOOM OUT",  C_BTN);

  // ── Bottom mode buttons ──────────────────────────────────────────────────────
  gfx->fillRect(0, BTN_Y, 800, BTN_H, C_GREY);

  const char *btns[] = { "SHUTTLE", "JOG", "DIAL", "SERIAL", "HOME" };
  const int BW5 = 800 / 5;
  for (int i = 0; i < 5; i++) {
    bool active = (i < 4 && currentMode == (ControlMode)i);
    draw_button(i * BW5 + 4, BTN_Y + 6, BW5 - 8, BTN_H - 12,
                btns[i], active ? C_BTN_ACT : C_BTN);
  }
}

// forward declaration — remote_paired is declared later with ESP-NOW globals
extern bool remote_paired;

// ── PCF8574 I2C GPIO expander ────────────────────────────────────────────────
// Address: 0x20 (all address pins pulled to GND — DEVMO default)
// Detected at startup via I2C probe; all functions are no-ops if not found.
//
// Pin allocation (not yet wired — no actions taken on any input):
//   P0 = E-stop button      (input, active-LOW)
//   P1 = Re-home button     (input, active-LOW)
//   P2 = Pair button        (input, active-LOW)
//   P3 = Camera shutter out (output, active-HIGH) — function exists, not called
//   P4 = LED: homed         (output, active-HIGH)
//   P5 = LED: remote linked (output, active-HIGH)
//   P6 = LED: moving        (output, active-HIGH)
//   P7 = spare

#define PCF8574_ADDR 0x20

// Output shadow register (1 = high / input-mode, 0 = driven low)
// Initialise with all input pins HIGH (quasi-bidirectional) and outputs LOW.
static uint8_t pcf_output = 0b00000111; // P0–P2 high (inputs), P3–P7 low

static bool pcf_present = false;

static void pcf_write(uint8_t val) {
    Wire.beginTransmission(PCF8574_ADDR);
    Wire.write(val);
    Wire.endTransmission();
    pcf_output = val;
}

static uint8_t pcf_read_pins() {
    Wire.requestFrom((uint8_t)PCF8574_ADDR, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0xFF;
}

void setup_pcf() {
    Wire.beginTransmission(PCF8574_ADDR);
    if (Wire.endTransmission() == 0) {
        pcf_present = true;
        pcf_write(pcf_output); // set initial pin directions
        Serial.println("[PCF8574] found at 0x20");
    } else {
        Serial.println("[PCF8574] not detected — running without expander");
    }
}

// Read inputs and log — no motor or pairing actions taken yet.
void handle_pcf_inputs() {
    if (!pcf_present) return;
    static uint8_t prev = 0xFF;
    uint8_t pins = pcf_read_pins();
    if (pins == prev) return;
    prev = pins;
    // P0–P2 are active-LOW inputs
    if (!(pins & (1 << 0))) Serial.println("[PCF8574] E-stop pressed (not wired)");
    if (!(pins & (1 << 1))) Serial.println("[PCF8574] Re-home pressed (not wired)");
    if (!(pins & (1 << 2))) Serial.println("[PCF8574] Pair pressed (not wired)");
}

// Update output LEDs to reflect current system state.
void pcf_update_leds() {
    if (!pcf_present) return;
    uint8_t val = pcf_output;
    // Preserve input bits (P0–P2) high; only touch P4–P6
    if (is_homed)        val |=  (1 << 4); else val &= ~(1 << 4);
    if (remote_paired)   val |=  (1 << 5); else val &= ~(1 << 5);
    // "moving" = we have a live remote and the axis is not at rest
    // Leave moving LED off for now (no settled/moving state tracked yet)
    val &= ~(1 << 6);
    if (val != pcf_output) pcf_write(val);
}

// Trigger camera shutter via P3 (50 ms pulse). Not called from anywhere yet.
void camera_shutter_trigger() {
    if (!pcf_present) return;
    pcf_write(pcf_output | (1 << 3));
    delay(50);
    pcf_write(pcf_output & ~(1 << 3));
}

// ── ESP-NOW protocol ─────────────────────────────────────────────────────────
// Packet structures are byte-for-byte identical to main.cpp and remote.cpp.

#define PKT_REMOTE   0x01
#define PKT_STATUS   0x02
#define PKT_PAIR_REQ 0x03
#define PKT_PAIR_ACK 0x04

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint8_t  button1;       // 1 = pressed (FWD)
  uint8_t  button2;       // 1 = pressed (REV)
  uint8_t  mode_request;  // 0–2 = absolute mode, 0xFF = no change
  uint16_t pot_value;     // 0–4095
} RemotePacket;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint8_t  current_mode;
  uint8_t  is_homed;
  uint16_t position_pct;  // 0–1000 (tenths of %)
} StatusPacket;

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
} PairPacket;

RemotePacket latest_remote = {};
volatile bool remote_data_fresh = false;
volatile unsigned long last_remote_received = 0;
#define REMOTE_TIMEOUT 2000UL

Preferences prefs;
uint8_t remote_mac[6] = {};
bool remote_paired   = false;
bool pairing_enabled = false;
volatile bool pair_ack_pending = false;
uint8_t pair_ack_mac[6] = {};

unsigned long last_status_send = 0;
#define STATUS_INTERVAL 200

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t pkt_type = data[0];
  if (pkt_type == PKT_REMOTE && len >= (int)sizeof(RemotePacket)) {
    memcpy(&latest_remote, data, sizeof(RemotePacket));
    remote_data_fresh = true;
    last_remote_received = millis();
  } else if (pkt_type == PKT_PAIR_REQ) {
    if (pairing_enabled || !remote_paired) {
      memcpy(pair_ack_mac, mac_addr, 6);
      pair_ack_pending = true;
    }
  }
}

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  Serial.printf("S3 MAC: %s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  bool was_paired = prefs.getBool("paired", false);
  if (was_paired) {
    prefs.getBytes("remote_mac", remote_mac, 6);
    bool valid = false;
    for (int i = 0; i < 6; i++) if (remote_mac[i]) { valid = true; break; }
    if (valid) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, remote_mac, 6);
      peer.channel = 1; peer.encrypt = false;
      esp_now_add_peer(&peer);
      remote_paired = true;
      Serial.printf("Loaded remote: %02X:%02X:%02X:%02X:%02X:%02X\n",
        remote_mac[0], remote_mac[1], remote_mac[2],
        remote_mac[3], remote_mac[4], remote_mac[5]);
    }
  }
  if (!remote_paired) Serial.println("No paired remote. Send 'pair' via serial.");
}

void send_status() {
  if (!remote_paired) return;
  if (millis() - last_status_send < STATUS_INTERVAL) return;

  StatusPacket pkt;
  pkt.pkt_type     = PKT_STATUS;
  pkt.current_mode = (uint8_t)currentMode;
  pkt.is_homed     = is_homed ? 1 : 0;

  if (is_homed && travelSteps > BUFFER_STEPS) {
    long range  = travelSteps - BUFFER_STEPS;
    long offset = constrain(currentPos, BUFFER_STEPS, travelSteps) - BUFFER_STEPS;
    pkt.position_pct = (uint16_t)((offset * 1000L) / range);
  } else {
    pkt.position_pct = 0;
  }

  esp_now_send(remote_mac, (uint8_t *)&pkt, sizeof(pkt));
  last_status_send = millis();
}

// ── Motion complete polling ───────────────────────────────────────────────────
// Blocks until the SERVO42D position stops changing, or timeoutMs elapses.
// Used after go-home and end-stop moves where the driver stops itself.
// Returns true when motion is complete, false on timeout.
static bool wait_for_motion_complete(unsigned long timeoutMs) {
  long         lastPos     = LONG_MIN;
  unsigned long still_since = 0;
  unsigned long t0          = millis();

  while (millis() - t0 < timeoutMs) {
    delay(50);
    long pos = servo_read_position();
    if (pos == LONG_MIN) continue;    // comm glitch — keep trying
    currentPos = pos;

    if (lastPos == LONG_MIN) {
      lastPos     = pos;
      still_since = millis();
      continue;
    }

    if (abs(pos - lastPos) > STALL_THRESHOLD_PULSES) {
      lastPos     = pos;              // Still moving
      still_since = millis();
    } else if (millis() - still_since >= STALL_CONFIRM_MS) {
      return true;                    // Position settled — motion complete
    }
  }
  return false;
}

// ── Homing ────────────────────────────────────────────────────────────────────
// Phase 1: SERVO42D go-home command → driver moves to home switch and sets
//          zero internally.  We wait until the driver reports motion complete.
// Phase 2: long move toward far end → driver stops on end switch → we read
//          back the position to calibrate travelSteps.
// All switch handling is done by the SERVO42D PCB — no ESP32 GPIO needed.
void homeAxis() {
  is_homed = false;

  // ── Phase 1 ──────────────────────────────────────────────────────────────────
  Serial.println("Homing: Phase 1 — go-home command (driver seeks home switch)...");
  servo_go_home();

  // 90s is ample for a 2000mm rail at the driver's configured homing speed
  if (!wait_for_motion_complete(90000UL)) {
    Serial.println("FATAL: Phase 1 timed out. Check home switch wiring on SERVO42D PCB.");
    while (true) delay(500);
  }

  // Driver has already set its internal position to zero
  currentPos = 0;
  targetPos  = 0;
  Serial.println("Phase 1 complete. Home = 0.");

  // Back off from the home switch before Phase 2
  servo_move_to(BUFFER_STEPS, HOME_SPEED, HOME_ACCEL);
  if (!wait_for_motion_complete(10000UL)) {
    Serial.println("FATAL: Back-off after Phase 1 timed out.");
    while (true) delay(500);
  }

  // ── Phase 2 ──────────────────────────────────────────────────────────────────
  Serial.println("Homing: Phase 2 — seeking far-end switch...");
  // Command a distance well beyond the 2000mm rail; the end switch stops the driver
  servo_move_to(500000L, HOME_SPEED, HOME_ACCEL);

  // 120s covers the full ~2000mm traverse at homing speed
  if (!wait_for_motion_complete(120000UL)) {
    Serial.println("FATAL: Phase 2 timed out. Check end switch wiring on SERVO42D PCB.");
    while (true) delay(500);
  }

  long rawEnd = servo_read_position();
  if (rawEnd == LONG_MIN) {
    Serial.println("FATAL: Cannot read position after Phase 2.");
    while (true) delay(500);
  }
  currentPos  = rawEnd;
  travelSteps = rawEnd - BUFFER_STEPS;

  if (travelSteps <= BUFFER_STEPS) {
    Serial.println("ERROR: Rail travel too short. Halting.");
    while (true) delay(500);
  }

  // Return to safe home position
  servo_move_to(BUFFER_STEPS, RUN_SPEED, RUN_ACCEL);
  wait_for_motion_complete(30000UL);

  is_homed = true;
  Serial.printf("Homing complete. Range: [%d, %ld] pulses.\n", BUFFER_STEPS, travelSteps);
  Serial.println("Commands: 'mode shuttle|joy|dial|serial', '50%', '1500', 'pair', 'unpair', 'home'");
}

// ── Soft limits ───────────────────────────────────────────────────────────────
// After homing, keep the carriage within [0, travelSteps] using the cached
// position.  No external switches — the calibrated range is our boundary.
void checkLimits() {
  if (!is_homed) return;

  if (currentPos < 0) {
    servo_emergency_stop();
    Serial.println("LIMIT: below home. Backing off.");
    servo_move_to(BUFFER_STEPS, HOME_SPEED, HOME_ACCEL);
    targetPos = BUFFER_STEPS;
  } else if (currentPos > travelSteps + BUFFER_STEPS) {
    servo_emergency_stop();
    Serial.println("LIMIT: beyond far end. Backing off.");
    servo_move_to(travelSteps, HOME_SPEED, HOME_ACCEL);
    targetPos = travelSteps;
  }
}

// ── Mode handlers ─────────────────────────────────────────────────────────────
void handleShuttleMode() {
  bool remote_active = remote_paired && last_remote_received > 0;
  int rawVal = remote_active ? (int)latest_remote.pot_value : 0;

  long speed = map(rawVal, 0, 4095, 0, RUN_SPEED);
  if (speed < 2) { servo_emergency_stop(); return; }

  if (servo_motion_complete()) {
    long mid  = (BUFFER_STEPS + travelSteps) / 2;
    long dest = (currentPos < mid) ? travelSteps : BUFFER_STEPS;
    servo_move_to(dest, (uint8_t)speed, RUN_ACCEL);
  }
}

void handleJoystickMode() {
  bool remote_connected = remote_paired && (millis() - last_remote_received < REMOTE_TIMEOUT);
  bool fwd = remote_connected && (latest_remote.button1 == 1);
  bool rev = remote_connected && (latest_remote.button2 == 1);

  static bool jogging = false;
  if (fwd && !rev) {
    if (!jogging) { servo_move_to(travelSteps, RUN_SPEED, RUN_ACCEL); jogging = true; }
  } else if (rev && !fwd) {
    if (!jogging) { servo_move_to(BUFFER_STEPS, RUN_SPEED, RUN_ACCEL); jogging = true; }
  } else if (jogging) {
    servo_emergency_stop();
    jogging = false;
  }
}

void handleDialMode() {
  bool remote_active = remote_paired && last_remote_received > 0;
  int rawVal = remote_active ? (int)latest_remote.pot_value : 0;

  static int lastRaw = -1;
  if (lastRaw >= 0 && abs(rawVal - lastRaw) < 15) return;
  lastRaw = rawVal;

  long dest = map(rawVal, 0, 4095, BUFFER_STEPS, travelSteps);
  servo_move_to(dest, RUN_SPEED, RUN_ACCEL);
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (!input.length()) return;

  if (input.equalsIgnoreCase("mode shuttle")) { currentMode = MODE_SHUTTLE;  Serial.println("Mode: Shuttle");  return; }
  if (input.equalsIgnoreCase("mode joy"))     { currentMode = MODE_JOYSTICK; Serial.println("Mode: Joystick"); return; }
  if (input.equalsIgnoreCase("mode dial"))    { currentMode = MODE_POT;      Serial.println("Mode: Dial");     return; }
  if (input.equalsIgnoreCase("mode serial"))  { currentMode = MODE_SERIAL;   servo_emergency_stop(); Serial.println("Mode: Serial"); return; }
  if (input.equalsIgnoreCase("home"))         { homeAxis(); return; }
  if (input.equalsIgnoreCase("pair"))         { pairing_enabled = true; Serial.println("Pairing active."); return; }
  if (input.equalsIgnoreCase("unpair")) {
    remote_paired = false; pairing_enabled = false; memset(remote_mac, 0, 6);
    prefs.remove("paired"); prefs.remove("remote_mac");
    Serial.println("Unpaired."); return;
  }

  if (currentMode == MODE_SERIAL) {
    long dest;
    if (input.endsWith("%")) {
      float pct = constrain(input.substring(0, input.length() - 1).toFloat(), 0.f, 100.f);
      dest = BUFFER_STEPS + (long)((travelSteps - BUFFER_STEPS) * (pct / 100.f));
      Serial.printf("Moving to %.1f%% (%ld pulses)\n", pct, dest);
    } else {
      dest = constrain(input.toInt(), (long)BUFFER_STEPS, travelSteps);
      Serial.printf("Moving to %ld pulses\n", dest);
    }
    servo_move_to(dest, RUN_SPEED, RUN_ACCEL);
  }
}

// ── Touch handler ─────────────────────────────────────────────────────────────
// Touch regions must match the geometry in draw_display().
void handleTouch() {
  int tx, ty;
  if (gt911_read(&tx, &ty) == 0) return;

  // Bottom bar: 5 equally-wide mode/action buttons
  if (ty >= BTN_Y) {
    int idx = tx / (800 / 5);
    if (idx == 4) {
      homeAxis();
    } else if (idx >= 0 && idx <= 3) {
      currentMode = (ControlMode)idx;
      if (currentMode == MODE_SERIAL) servo_emergency_stop();
      Serial.printf("Touch: mode %d\n", idx);
    }
    return;
  }

  // PELCO-D D-pad (coordinates match the layout in draw_display)
  // Centre column x=330, button size 140×50, gap 8
  const int CX = 330, BW = 140, BH = 50, GAP = 8;
  const int RY0 = BODY_Y + 180, RY1 = RY0 + BH + GAP, RY2 = RY0 + 2 * (BH + GAP);

  // TILT UP
  if (tx >= CX && tx < CX+BW && ty >= RY0 && ty < RY0+BH)          { pelco_tilt_up(0x20);   return; }
  // PAN LEFT
  if (tx >= CX-BW-GAP && tx < CX-GAP && ty >= RY1 && ty < RY1+BH)  { pelco_pan_left(0x20);  return; }
  // STOP
  if (tx >= CX && tx < CX+BW && ty >= RY1 && ty < RY1+BH)           { pelco_stop();           return; }
  // PAN RIGHT
  if (tx >= CX+BW+GAP && tx < CX+2*BW+GAP && ty >= RY1 && ty < RY1+BH) { pelco_pan_right(0x20); return; }
  // TILT DOWN
  if (tx >= CX && tx < CX+BW && ty >= RY2 && ty < RY2+BH)           { pelco_tilt_down(0x20); return; }
  // ZOOM IN
  if (tx >= CX+2*(BW+GAP) && tx < CX+3*BW+2*GAP && ty >= RY1 && ty < RY1+BH) { pelco_zoom_in();  return; }
  // ZOOM OUT
  if (tx >= CX+2*(BW+GAP) && tx < CX+3*BW+2*GAP && ty >= RY2 && ty < RY2+BH) { pelco_zoom_out(); return; }
}

// ── Setup & Loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- S3 Camera Rail Controller ---");

  prefs.begin("main", false);

  // I2C (touch controller + CH422G expander + optional PCF8574)
  Wire.begin(I2C_SDA, I2C_SCL);
  setup_pcf();

  // Display
  gfx->init();
  gfx->fillScreen(C_BG);
  gfx->setTextColor(C_FG);
  gfx->setTextSize(3);
  gfx->setCursor(20, 20);
  gfx->print("Initializing...");

  // SERVO42D
  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  delay(100);
  servo_enable();
  Serial.println("SERVO42D: enabled");

  // RS485 / PELCO-D
  CameraSerial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.println("PELCO-D: RS485 ready");

  // ESP-NOW
  setup_espnow();

  delay(500);
  homeAxis();
}

void loop() {
  // ── Complete deferred pairing (never call esp_now or Preferences from callback) ──
  if (pair_ack_pending) {
    pair_ack_pending = false;
    memcpy(remote_mac, pair_ack_mac, 6);
    prefs.putBytes("remote_mac", remote_mac, 6);
    prefs.putBool("paired", true);
    if (!esp_now_is_peer_exist(remote_mac)) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, remote_mac, 6);
      peer.channel = 1; peer.encrypt = false;
      esp_now_add_peer(&peer);
    }
    PairPacket ack = { PKT_PAIR_ACK };
    esp_now_send(remote_mac, (uint8_t *)&ack, sizeof(ack));
    remote_paired = true; pairing_enabled = false;
    Serial.printf("Paired: %02X:%02X:%02X:%02X:%02X:%02X\n",
      remote_mac[0], remote_mac[1], remote_mac[2],
      remote_mac[3], remote_mac[4], remote_mac[5]);
  }

  // ── Remote mode updates ──────────────────────────────────────────────────────
  if (remote_data_fresh) {
    remote_data_fresh = false;
    if (latest_remote.mode_request <= 2)
      currentMode = (ControlMode)latest_remote.mode_request;
  }

  // ── Poll SERVO42D position every 50 ms ──────────────────────────────────────
  static unsigned long last_pos_poll = 0;
  if (millis() - last_pos_poll > 50) {
    long p = servo_read_position();
    if (p != LONG_MIN) currentPos = p;
    last_pos_poll = millis();
  }

  handle_pcf_inputs();
  pcf_update_leds();
  send_status();
  handleSerialCommands();
  checkLimits();
  handleTouch();

  switch (currentMode) {
    case MODE_SHUTTLE:  handleShuttleMode();  break;
    case MODE_JOYSTICK: handleJoystickMode(); break;
    case MODE_POT:      handleDialMode();     break;
    case MODE_SERIAL:                         break;
  }

  // ── Refresh display ──────────────────────────────────────────────────────────
  bool link_ok = remote_paired && (millis() - last_remote_received < REMOTE_TIMEOUT);
  uint16_t pos_pct = 0;
  if (is_homed && travelSteps > BUFFER_STEPS) {
    long range  = travelSteps - BUFFER_STEPS;
    long offset = constrain(currentPos, BUFFER_STEPS, travelSteps) - BUFFER_STEPS;
    pos_pct = (uint16_t)((offset * 1000L) / range);
  }
  draw_display(link_ok, pos_pct);

  delay(10);
}
