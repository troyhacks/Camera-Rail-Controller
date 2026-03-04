#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_sleep.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "esp_wifi.h"

// ── Screen ────────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define I2C_SDA 6   // XIAO Pin D4
#define I2C_SCL 7   // XIAO Pin D5
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Pins ──────────────────────────────────────────────────────────────────────
#define BUTTON_1 8   // XIAO Pin D1 — FWD
#define BUTTON_2 9   // XIAO Pin D2 — REV
#define BUTTON_3 10  // XIAO Pin D3 — Mode / Pair
#define POT_PIN  2   // XIAO Pin D0 (ADC)

// ── ESP-NOW Protocol ──────────────────────────────────────────────────────────
#define PKT_REMOTE   0x01
#define PKT_STATUS   0x02
#define PKT_PAIR_REQ 0x03
#define PKT_PAIR_ACK 0x04

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint8_t  button1;       // 1=pressed (FWD)
  uint8_t  button2;       // 1=pressed (REV)
  uint8_t  mode_request;  // 0xFE=advance mode, 0xFF=no change
  uint16_t pot_value;     // 0-4095
} RemotePacket;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint8_t  current_mode;
  uint8_t  is_homed;
  uint16_t position_pct;  // 0-1000 (tenths of %)
} StatusPacket;

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
} PairPacket;

// Mode names — must match main.cpp ControlMode enum order
const char* MODE_NAMES[] = { "SHUTTLE", "JOYSTICK", "DIAL", "SERIAL" };

// ── State ─────────────────────────────────────────────────────────────────────
Preferences prefs;

uint8_t main_mac[6] = {};
bool paired = false;

bool pairing_mode = false;
unsigned long pairing_start = 0;
unsigned long last_pair_broadcast = 0;
#define PAIRING_TIMEOUT     30000UL
#define PAIR_BROADCAST_MS   500UL

StatusPacket last_status = {};
unsigned long last_status_received = 0;
uint8_t confirmed_mode = 0;
#define STATUS_TIMEOUT 2000UL

RemotePacket tx_packet = {};
RemotePacket last_tx_packet = { PKT_REMOTE, 0xFF, 0xFF, 0xFF, 0xFFFF };
unsigned long last_send = 0;
#define SEND_INTERVAL    100
#define KEEPALIVE_MS    1000

bool mode_advance_pending = false;
uint8_t pending_mode = 0xFF; // Absolute mode to request (0-2), set on B3 short press
// Deferred pairing — set in callback, acted on in loop()
volatile bool pair_complete_pending = false;
uint8_t pair_complete_mac[6] = {};

// Button 3 state machine
bool b3_last = false;
unsigned long b3_press_time = 0;
bool b3_long_triggered = false;
#define LONG_PRESS_MS 3000UL

// Sleep / inactivity
unsigned long last_activity = 0;
int last_pot_raw = -1;
bool display_on = true;
bool entering_sleep = false;
unsigned long wake_time = 0;
#define INACTIVITY_TIMEOUT 60000UL
#define WAKEUP_GRACE_MS    500UL

// ── ESP-NOW Callbacks ─────────────────────────────────────────────────────────

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // silent
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t pkt_type = data[0];

  if (pkt_type == PKT_STATUS && len >= (int)sizeof(StatusPacket)) {
    memcpy(&last_status, data, sizeof(StatusPacket));
    last_status_received = millis();
    confirmed_mode = last_status.current_mode;
  }
  else if (pkt_type == PKT_PAIR_ACK) {
    // Only set flag — peer add and NVS write happen safely in loop()
    if (pairing_mode) {
      memcpy(pair_complete_mac, mac_addr, 6);
      pair_complete_pending = true;
    }
  }
}

// ── Pairing ───────────────────────────────────────────────────────────────────

void enter_pairing_mode() {
  pairing_mode = true;
  pairing_start = millis();
  last_pair_broadcast = 0;
  Serial.println("Entering pairing mode — hold GPIO13 on main or send 'pair' via serial.");
}

void handle_pairing() {
  if (!pairing_mode) return;

  if (millis() - pairing_start > PAIRING_TIMEOUT) {
    pairing_mode = false;
    Serial.println("Pairing timed out.");
    return;
  }

  if (millis() - last_pair_broadcast > PAIR_BROADCAST_MS) {
    uint8_t broadcast[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    PairPacket pkt = { PKT_PAIR_REQ };
    esp_now_send(broadcast, (uint8_t*)&pkt, sizeof(pkt));
    last_pair_broadcast = millis();
  }
}

// ── ESP-NOW Setup ─────────────────────────────────────────────────────────────

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // Fix channel so ESP32 and ESP32-C3 agree

  Serial.printf("Remote MAC: %s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Always add broadcast peer (needed to send pairing requests)
  esp_now_peer_info_t broadcast_peer = {};
  uint8_t broadcast[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  memcpy(broadcast_peer.peer_addr, broadcast, 6);
  broadcast_peer.channel = 1;
  broadcast_peer.encrypt = false;
  if (!esp_now_is_peer_exist(broadcast)) {
    esp_now_add_peer(&broadcast_peer);
  }

  // Load saved main MAC
  bool was_paired = prefs.getBool("paired", false);
  if (was_paired) {
    prefs.getBytes("main_mac", main_mac, 6);
    bool is_valid = false;
    for (int i = 0; i < 6; i++) if (main_mac[i] != 0) { is_valid = true; break; }
    if (is_valid) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, main_mac, 6);
      peer.channel = 1;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
      paired = true;
      Serial.printf("Loaded main MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        main_mac[0], main_mac[1], main_mac[2],
        main_mac[3], main_mac[4], main_mac[5]);
    }
  }

  if (!paired) {
    Serial.println("No paired device. Hold Button 3 for 3s to pair.");
  }
}

// ── Send Data ─────────────────────────────────────────────────────────────────

void send_remote_data() {
  if (!paired) return;

  bool changed = (
    tx_packet.button1    != last_tx_packet.button1   ||
    tx_packet.button2    != last_tx_packet.button2   ||
    abs((int)tx_packet.pot_value - (int)last_tx_packet.pot_value) > 10
  );

  if (changed && millis() - last_send > SEND_INTERVAL) {
    tx_packet.mode_request = 0xFF;
    esp_now_send(main_mac, (uint8_t*)&tx_packet, sizeof(tx_packet));
    memcpy(&last_tx_packet, &tx_packet, sizeof(tx_packet));
    last_send = millis();
  }
}

void send_mode_advance() {
  if (!paired || pending_mode > 3) return;
  tx_packet.mode_request = pending_mode; // Absolute mode — idempotent if received twice
  esp_now_send(main_mac, (uint8_t*)&tx_packet, sizeof(tx_packet));
  memcpy(&last_tx_packet, &tx_packet, sizeof(tx_packet));
  last_send = millis();
  tx_packet.mode_request = 0xFF;
  pending_mode = 0xFF;
}

// ── Display ───────────────────────────────────────────────────────────────────

void update_display(bool b1, bool b2, int pot_value) {
  display.clearDisplay();
  display.setTextSize(1);

  if (pairing_mode) {
    display.setCursor(0, 0);
    display.print("** PAIRING **");
    display.setCursor(0, 16);
    unsigned long remaining = (PAIRING_TIMEOUT - (millis() - pairing_start)) / 1000;
    display.printf("Timeout: %lus", remaining);
    display.setCursor(0, 32);
    display.print("Waiting for main...");
    display.display();
    return;
  }

  bool link_ok = paired && (millis() - last_status_received < STATUS_TIMEOUT);

  // Line 0: Mode + link status
  display.setCursor(0, 0);
  display.print("Mode: ");
  display.print(last_status_received > 0 ? MODE_NAMES[confirmed_mode % 4] : "---");
  display.setCursor(104, 0);
  display.print(link_ok ? "[OK]" : "[--]");

  float pos_pct = last_status.position_pct / 10.0f;
  int   spd_pct = pot_value * 100 / 4095;

  if (confirmed_mode == 2) {
    // DIAL mode: Line 1 = dial setpoint, Line 2 = actual position
    display.setCursor(0, 16);
    display.printf("Dial: %3d%%", spd_pct);
    display.setCursor(0, 32);
    if (link_ok && last_status.is_homed) {
      display.printf("Pos:  %5.1f%%", pos_pct);
    } else if (link_ok && !last_status.is_homed) {
      display.print("Pos:  HOMING");
    } else {
      display.print("Pos:  ---");
    }
  } else {
    // Line 1: Position
    display.setCursor(0, 16);
    if (link_ok && last_status.is_homed) {
      display.printf("Pos: %5.1f%%", pos_pct);
    } else if (link_ok && !last_status.is_homed) {
      display.print("Pos: HOMING");
    } else {
      display.print("Pos: ---");
    }

    // Line 2: speed% in SHUTTLE mode, button states otherwise
    display.setCursor(0, 32);
    if (confirmed_mode == 0) {
      display.printf("Speed: %3d%%", spd_pct);
    } else {
      display.printf("FWD:[%s] REV:[%s]", b1 ? ">" : "-", b2 ? "<" : "-");
    }
  }

  // Line 3: Hint
  display.setCursor(0, 48);
  if (!paired) {
    display.print("Hold B3 3s: Pair");
  } else {
    display.print("B3:Mode  Hold:Pair");
  }

  display.display();
}

// ── Setup & Loop ──────────────────────────────────────────────────────────────

void setup() {
  last_activity = millis();
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nESP-NOW Remote Starting...");

  prefs.begin("remote", false);

  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed. Check wiring!"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();

  setup_espnow();

  last_pot_raw = analogRead(POT_PIN);
  tx_packet.pkt_type = PKT_REMOTE;
  tx_packet.mode_request = 0xFF;
}

void loop() {
  // Complete pending pairing (deferred from callback to avoid WiFi task issues)
  if (pair_complete_pending) {
    pair_complete_pending = false;
    memcpy(main_mac, pair_complete_mac, 6);
    prefs.putBytes("main_mac", main_mac, 6);
    prefs.putBool("paired", true);
    if (!esp_now_is_peer_exist(main_mac)) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, main_mac, 6);
      peer.channel = 1;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
    }
    paired = true;
    pairing_mode = false;
    Serial.printf("Paired with main: %02X:%02X:%02X:%02X:%02X:%02X\n",
      main_mac[0], main_mac[1], main_mac[2],
      main_mac[3], main_mac[4], main_mac[5]);
  }

  // Read inputs (inverted: 1 = pressed)
  bool b1 = !digitalRead(BUTTON_1);
  bool b2 = !digitalRead(BUTTON_2);
  bool b3_raw = !digitalRead(BUTTON_3);
  bool b3 = b3_raw && (millis() - wake_time > WAKEUP_GRACE_MS);

  // Pot with dead-zone at low end
  int raw_pot = analogRead(POT_PIN);
  const int POT_MIN = 1000, POT_MAX = 4095;
  int pot_value;
  if (raw_pot <= POT_MIN)      pot_value = 0;
  else if (raw_pot >= POT_MAX) pot_value = 4095;
  else                         pot_value = map(raw_pot, POT_MIN, POT_MAX, 0, 4095);

  // ── Button 3 state machine ────────────────────────────────────────────────
  if (b3 && !b3_last) {
    // Rising edge
    b3_press_time = millis();
    b3_long_triggered = false;
  }
  else if (b3 && b3_last && !b3_long_triggered) {
    // Still held — check for long press
    if (millis() - b3_press_time >= LONG_PRESS_MS) {
      b3_long_triggered = true;
      enter_pairing_mode();
    }
  }
  else if (!b3 && b3_last) {
    // Falling edge — compute next mode from current confirmed state
    if (!b3_long_triggered) {
      uint8_t cur = confirmed_mode;
      if (cur == 0)      pending_mode = 1; // SHUTTLE → JOYSTICK
      else if (cur == 1) pending_mode = 2; // JOYSTICK → DIAL
      else               pending_mode = 0; // DIAL (or SERIAL) → SHUTTLE
      mode_advance_pending = true;
    }
  }
  b3_last = b3;

  // ── Activity detection ────────────────────────────────────────────────────
  if (b1 || b2 || b3 || abs(raw_pot - last_pot_raw) > 30) {
    last_activity = millis();
    last_pot_raw = raw_pot;
    display_on = true;
    entering_sleep = false;
  }

  // ── Build TX packet ───────────────────────────────────────────────────────
  tx_packet.button1    = b1 ? 1 : 0;
  tx_packet.button2    = b2 ? 1 : 0;
  tx_packet.pot_value  = pot_value;

  // ── Send mode advance immediately (bypasses rate limit) ───────────────────
  if (mode_advance_pending) {
    mode_advance_pending = false;
    send_mode_advance();
  }

  // ── Regular data send + keepalive + pairing ──────────────────────────────
  handle_pairing();
  send_remote_data();

  // Keepalive: force a send every 1s so main never times out the link
  if (paired && millis() - last_send > KEEPALIVE_MS) {
    tx_packet.mode_request = 0xFF;
    esp_now_send(main_mac, (uint8_t*)&tx_packet, sizeof(tx_packet));
    memcpy(&last_tx_packet, &tx_packet, sizeof(tx_packet));
    last_send = millis();
  }

  // ── Sleep ─────────────────────────────────────────────────────────────────
  if (millis() - last_activity > INACTIVITY_TIMEOUT && display_on && !entering_sleep && !pairing_mode) {
    if (Serial.available() == 0) {
      entering_sleep = true;
      display_on = false;
      display.clearDisplay();
      display.display();
      delay(100);
      Serial.flush();
      delay(50);

      pinMode(BUTTON_3, INPUT_PULLUP);
      esp_sleep_enable_gpio_wakeup();
      gpio_wakeup_enable((gpio_num_t)BUTTON_3, GPIO_INTR_LOW_LEVEL);
      esp_light_sleep_start();

      // Woke up
      gpio_wakeup_disable((gpio_num_t)BUTTON_3);
      delay(10);
      Serial.begin(115200);

      // Re-init ESP-NOW after light sleep
      esp_now_deinit();
      setup_espnow();

      last_activity = millis();
      wake_time = millis();
      display_on = true;
      entering_sleep = false;
    }
  }

  // ── Display ───────────────────────────────────────────────────────────────
  if (display_on) {
    update_display(b1, b2, pot_value);
  } else {
    display.clearDisplay();
    display.display();
  }

  delay(50);
}
