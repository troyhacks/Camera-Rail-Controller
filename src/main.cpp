#include <Arduino.h>
#include <TMC2209.h>
#include <ESP_FlexyStepper.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include "esp_wifi.h"

// ── Hardware Pins & Parameters ────────────────────────────────────────────────
const int MICROSTEPS = 8;
const int STEPS_PER_REV = 200 * MICROSTEPS;     // 1600 steps/rev
const int HOME_SPEED = STEPS_PER_REV;    // 800 steps/s  — slow for homing
const int RUN_SPEED = STEPS_PER_REV * 4;    // 6400 steps/s — shuttle speed
const int BUFFER_STEPS = STEPS_PER_REV / 4;    // 400 steps    — margin off each switch

// TMC2209 UART
const long SERIAL_BAUD_RATE = 115200;
const int  RX_PIN = 26;
const int  TX_PIN = 25;
const int  HW_DISABLE_PIN = 19;
const int  MOTOR_STEP = 22;
const int  MOTOR_DIR = 21;

// Inputs
const int  MOTOR_HOME_PIN = 14;   // begin / home limit switch — INPUT_PULLUP, active LOW
const int  MOTOR_FAR_END_PIN = 33;  // far-end  limit switch     — INPUT_PULLUP, active LOW
const int  POT_PIN = 34;   // Potentiometer analog pin
const int  JOY_FWD_PIN = 32;   // Joystick Forward (or button) — INPUT_PULLUP, active LOW
const int  JOY_REV_PIN = 27;   // Joystick Reverse (or button) — INPUT_PULLUP, active LOW
const int  PAIR_BUTTON_PIN = 13; // Pairing button — INPUT_PULLUP, active LOW

TMC2209 motor;
const uint8_t REPLY_DELAY = 2;
ESP_FlexyStepper stepper;

long travelSteps = 0; // Shuttle range: [BUFFER_STEPS, travelSteps]
bool is_homed = false;

// ── Operating Modes ───────────────────────────────────────────────────────────
enum ControlMode {
  MODE_SHUTTLE,   // Bounces back and forth automatically
  MODE_JOYSTICK,  // Uses buttons to jog, stops smoothly on release
  MODE_POT,       // Maps analog dial to absolute position
  MODE_SERIAL     // Waits for position or percentage commands via serial
};

ControlMode currentMode = MODE_SERIAL; // Default starting mode

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

// Remote link state
RemotePacket latest_remote = {};
volatile bool remote_data_fresh = false;
volatile unsigned long last_remote_received = 0;
#define REMOTE_TIMEOUT 2000UL

// Pairing
Preferences prefs;
uint8_t remote_mac[6] = {};
bool remote_paired = false;
bool pairing_enabled = false;
// Deferred pairing — set in callback, acted on in loop()
volatile bool pair_ack_pending = false;
uint8_t pair_ack_mac[6] = {};

// Status TX
unsigned long last_status_send = 0;
#define STATUS_INTERVAL 200

// ── ESP-NOW Callbacks ─────────────────────────────────────────────────────────

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // silent
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t pkt_type = data[0];

  if (pkt_type == PKT_REMOTE && len >= (int)sizeof(RemotePacket)) {
    memcpy(&latest_remote, data, sizeof(RemotePacket));
    remote_data_fresh = true;
    last_remote_received = millis();
  }
  else if (pkt_type == PKT_PAIR_REQ) {
    // Only set flag — peer add, NVS write, and send happen safely in loop()
    if (pairing_enabled || !remote_paired) {
      memcpy(pair_ack_mac, mac_addr, 6);
      pair_ack_pending = true;
    }
  }
}

// ── ESP-NOW Setup ─────────────────────────────────────────────────────────────

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // Fix channel so ESP32 and ESP32-C3 agree

  Serial.printf("Main MAC: %s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  bool was_paired = prefs.getBool("paired", false);
  if (was_paired) {
    prefs.getBytes("remote_mac", remote_mac, 6);
    bool is_valid = false;
    for (int i = 0; i < 6; i++) if (remote_mac[i] != 0) { is_valid = true; break; }
    if (is_valid) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, remote_mac, 6);
      peer.channel = 1;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
      remote_paired = true;
      Serial.printf("Loaded remote MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        remote_mac[0], remote_mac[1], remote_mac[2],
        remote_mac[3], remote_mac[4], remote_mac[5]);
    }
  }

  if (!remote_paired) {
    Serial.println("No paired remote. Press GPIO13 or send 'pair' to accept a pairing request.");
  }
}

// ── Status Send ───────────────────────────────────────────────────────────────

void send_status() {
  if (!remote_paired) return;
  if (millis() - last_status_send < STATUS_INTERVAL) return;

  StatusPacket pkt;
  pkt.pkt_type = PKT_STATUS;
  pkt.current_mode = (uint8_t)currentMode;
  pkt.is_homed = is_homed ? 1 : 0;

  if (is_homed && travelSteps > BUFFER_STEPS) {
    long pos = stepper.getCurrentPositionInSteps();
    long range = travelSteps - BUFFER_STEPS;
    long offset = constrain(pos, BUFFER_STEPS, travelSteps) - BUFFER_STEPS;
    pkt.position_pct = (uint16_t)((offset * 1000L) / range);
  } else {
    pkt.position_pct = 0;
  }

  esp_now_send(remote_mac, (uint8_t*)&pkt, sizeof(pkt));
  last_status_send = millis();
}

// ── Speed helpers ─────────────────────────────────────────────────────────────
void setHomingSpeed() {
  stepper.setSpeedInStepsPerSecond(HOME_SPEED);
  stepper.setAccelerationInStepsPerSecondPerSecond(STEPS_PER_REV);
  stepper.setDecelerationInStepsPerSecondPerSecond(STEPS_PER_REV);
}

void setRunSpeed() {
  stepper.setSpeedInStepsPerSecond(RUN_SPEED);
  stepper.setAccelerationInStepsPerSecondPerSecond(6 * STEPS_PER_REV);
  stepper.setDecelerationInStepsPerSecondPerSecond(6 * STEPS_PER_REV);
}

// Wait for motionComplete() with a timeout.
static void waitMotionDone(unsigned long timeoutMs = 30000UL) {
  unsigned long t0 = millis();
  while (!stepper.motionComplete()) {
    if (millis() - t0 > timeoutMs) {
      Serial.println("TIMEOUT waiting for motion — Halting.");
      stepper.emergencyStop();
      while (true) delay(500);
    }
    delay(1);
  }
}

// ── Homing sequence ───────────────────────────────────────────────────────────
void homeAxis() {
  setHomingSpeed();

  // Phase 1: find home switch
  if (digitalRead(MOTOR_HOME_PIN) == LOW) {
    stepper.setTargetPositionRelativeInSteps(BUFFER_STEPS * 2);
    waitMotionDone();
  }

  Serial.println("Phase 1: seeking home switch...");
  stepper.setTargetPositionInSteps(-100000L);

  while (digitalRead(MOTOR_HOME_PIN) == HIGH) {
    // CRASH PREVENTION: Did we hit the wrong switch?
    if (digitalRead(MOTOR_FAR_END_PIN) == LOW) {
      stepper.emergencyStop();
      Serial.println("\nFATAL ERROR: Hit FAR END switch while seeking HOME.");
      Serial.println("Cause: Motor direction is physically reversed.");
      Serial.println("Fix: Flip the motor connector 180 degrees, or add motor.setInverseDirection(true);");
      while (true) delay(500); // Halt system
    }
    delay(1);
  }

  stepper.setCurrentPositionAsHomeAndStop(); // Sets internal counter to 0 instantly
  delay(200);

  // Back off
  stepper.setTargetPositionInSteps(BUFFER_STEPS);
  waitMotionDone();

  // Phase 2: find end switch
  if (digitalRead(MOTOR_FAR_END_PIN) == LOW) {
    stepper.setTargetPositionRelativeInSteps(-BUFFER_STEPS * 2);
    waitMotionDone();
  }

  Serial.println("Phase 2: seeking end switch...");
  stepper.setTargetPositionInSteps(100000L);

  while (digitalRead(MOTOR_FAR_END_PIN) == HIGH) {
    // CRASH PREVENTION: Did we bounce back and hit home again?
    if (digitalRead(MOTOR_HOME_PIN) == LOW) {
      stepper.emergencyStop();
      Serial.println("\nFATAL ERROR: Hit HOME switch while seeking FAR END.");
      while (true) delay(500);
    }
    delay(1);
  }

  stepper.emergencyStop();
  delay(100);

  long rawEnd = stepper.getCurrentPositionInSteps();
  travelSteps = rawEnd - BUFFER_STEPS; // Safe upper bound

  if (travelSteps <= BUFFER_STEPS) {
    Serial.println("ERROR: travel distance too short. Halting.");
    while (true) delay(500);
  }

  Serial.printf("Calibrated Range: [%d, %ld] steps.\n", BUFFER_STEPS, travelSteps);

  // Return to safe home
  setRunSpeed();
  stepper.setTargetPositionInSteps(BUFFER_STEPS);
  waitMotionDone();
  Serial.println("Homing complete. Awaiting commands (Default: Serial).");
  Serial.println("Commands: 'mode shuttle', 'mode joy', 'mode dial', 'mode serial', '50%', '1500', 'pair', 'unpair'");

  is_homed = true;
}

// ── Dynamic Limit Correction ──────────────────────────────────────────────────
void checkLimitsAndCorrect() {
  // If we unexpectedly hit a switch, re-calibrate that boundary on the fly
  if (digitalRead(MOTOR_HOME_PIN) == LOW && !stepper.motionComplete()) {
    stepper.emergencyStop();
    stepper.setCurrentPositionInSteps(0); // This is true zero now
    Serial.println("LIMIT HIT: Home boundary updated. Backing off.");
    stepper.setTargetPositionInSteps(BUFFER_STEPS);
  }

  if (digitalRead(MOTOR_FAR_END_PIN) == LOW && !stepper.motionComplete()) {
    stepper.emergencyStop();
    long newEnd = stepper.getCurrentPositionInSteps();
    travelSteps = newEnd - BUFFER_STEPS;
    Serial.printf("LIMIT HIT: Far boundary updated to %ld. Backing off.\n", travelSteps);
    stepper.setTargetPositionInSteps(travelSteps);
  }
}

// ── Control Routines ──────────────────────────────────────────────────────────
void handleSerialCommands() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // Handle Mode Switches
  if (input.equalsIgnoreCase("mode shuttle")) { currentMode = MODE_SHUTTLE; Serial.println("Mode: Shuttle"); return; }
  if (input.equalsIgnoreCase("mode joy")) { currentMode = MODE_JOYSTICK; Serial.println("Mode: Joystick"); return; }
  if (input.equalsIgnoreCase("mode dial")) { currentMode = MODE_POT; Serial.println("Mode: Dial"); return; }
  if (input.equalsIgnoreCase("mode serial")) { currentMode = MODE_SERIAL; Serial.println("Mode: Serial"); stepper.setTargetPositionToStop(); return; }

  // Pairing commands
  if (input.equalsIgnoreCase("pair")) {
    pairing_enabled = true;
    Serial.println("Pairing mode active — hold remote Button 3 for 3s to complete pairing.");
    return;
  }
  if (input.equalsIgnoreCase("unpair")) {
    remote_paired = false;
    pairing_enabled = false;
    memset(remote_mac, 0, 6);
    prefs.remove("paired");
    prefs.remove("remote_mac");
    Serial.println("Unpaired. Remote disconnected.");
    return;
  }

  // Handle Positions (Only in Serial Mode)
  if (currentMode == MODE_SERIAL) {
    long target = 0;
    if (input.endsWith("%")) {
      float pct = input.substring(0, input.length() - 1).toFloat();
      pct = constrain(pct, 0.0, 100.0);
      target = BUFFER_STEPS + (long)((travelSteps - BUFFER_STEPS) * (pct / 100.0));
      Serial.printf("Moving to %.1f%% (%ld steps)\n", pct, target);
    } else {
      target = input.toInt();
      target = constrain(target, BUFFER_STEPS, travelSteps);
      Serial.printf("Moving to step %ld\n", target);
    }
    stepper.setTargetPositionInSteps(target);
  }
}

void handleShuttleMode() {
  int rawVal;
  if (remote_paired && last_remote_received > 0) {
    rawVal = latest_remote.pot_value;
  } else {
    rawVal = analogRead(POT_PIN);
  }

  long shuttle_speed = map(rawVal, 0, 4095, 0, RUN_SPEED);

  if (shuttle_speed < 100) {
    stepper.setTargetPositionToStop();
    return;
  }

  stepper.setSpeedInStepsPerSecond(shuttle_speed);

  if (stepper.motionComplete()) {
    long pos = stepper.getCurrentPositionInSteps();
    long midpoint = (BUFFER_STEPS + travelSteps) / 2;
    if (pos < midpoint) stepper.setTargetPositionInSteps(travelSteps);
    else stepper.setTargetPositionInSteps(BUFFER_STEPS);
  }
}

void handleJoystickMode() {
  bool fwd, rev;
  bool remote_connected = remote_paired && (millis() - last_remote_received < REMOTE_TIMEOUT);

  if (remote_connected) {
    fwd = (latest_remote.button1 == 1);
    rev = (latest_remote.button2 == 1);
  } else {
    fwd = (digitalRead(JOY_FWD_PIN) == LOW);
    rev = (digitalRead(JOY_REV_PIN) == LOW);
  }

  static bool isJogging = false;

  if (fwd && !rev) {
    stepper.setTargetPositionInSteps(travelSteps);
    isJogging = true;
  }
  else if (rev && !fwd) {
    stepper.setTargetPositionInSteps(BUFFER_STEPS);
    isJogging = true;
  }
  else if (isJogging) {
    stepper.setTargetPositionToStop();
    isJogging = false;
  }
}

void handleDialMode() {
  int rawVal;

  if (remote_paired && last_remote_received > 0) {
    // Use remote dial: hold last known value when remote is silent (don't fall
    // back to local pot — that would jump to 0 if the local pot is floating)
    rawVal = latest_remote.pot_value;
  } else {
    rawVal = analogRead(POT_PIN);
  }

  // Deadband: ignore tiny ADC fluctuations that cause the motor to wander
  static int lastRawVal = -1;
  if (lastRawVal >= 0 && abs(rawVal - lastRawVal) < 15) return;
  lastRawVal = rawVal;

  long target = map(rawVal, 0, 4095, BUFFER_STEPS, travelSteps);
  stepper.setTargetPositionInSteps(target);
}

// ── Main Setup & Loop ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000); // Give the serial monitor a moment to connect

  Serial.println("\n--- Stepper Controller Initializing ---");

  prefs.begin("main", false);
  setup_espnow();

  // Attempt to connect to TMC2209
  motor.setup(Serial2, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, RX_PIN, TX_PIN);
  pinMode(HW_DISABLE_PIN, OUTPUT);
  digitalWrite(HW_DISABLE_PIN, LOW); // LOW enables the driver hardware

  // Set up limit switch pins safely AFTER serial is configured
  pinMode(MOTOR_HOME_PIN, INPUT_PULLUP);
  pinMode(MOTOR_FAR_END_PIN, INPUT_PULLUP);
  pinMode(JOY_FWD_PIN, INPUT_PULLUP);
  pinMode(JOY_REV_PIN, INPUT_PULLUP);
  pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);

  // Initialize TMC2209 parameters

  motor.setReplyDelay(REPLY_DELAY);
  motor.setHardwareEnablePin(HW_DISABLE_PIN);
  motor.setRunCurrent(80); // percent %
  motor.setHoldCurrent(5); // percent %
  motor.disableCoolStep();
  motor.disableStealthChop();
  motor.disableAutomaticGradientAdaptation();
  motor.disableAutomaticCurrentScaling();
  motor.setMicrostepsPerStep(MICROSTEPS);
  motor.enable();

  delay(100);

  // --- DIAGNOSTIC CHECK ---
  if (!motor.isSetupAndCommunicating()) {
    Serial.println("CRITICAL WARNING: TMC2209 UART communication failed!");
    Serial.println("Motor is relying entirely on physical VREF dial and default microsteps.");
  } else {
    Serial.println("SUCCESS: TMC2209 UART is communicating properly.");
  }

  stepper.connectToPins(MOTOR_STEP, MOTOR_DIR);
  stepper.setStepsPerRevolution(STEPS_PER_REV);
  stepper.setAccelerationInStepsPerSecondPerSecond(6 * STEPS_PER_REV);
  stepper.setDecelerationInStepsPerSecondPerSecond(6 * STEPS_PER_REV);
  stepper.startAsService(0);

  delay(1500); // in case the board resets during programming.

  // Begin homing sequence to find limits
  homeAxis();
}

void loop() {
  // Complete pending pairing (deferred from callback to avoid WiFi task issues)
  if (pair_ack_pending) {
    pair_ack_pending = false;
    memcpy(remote_mac, pair_ack_mac, 6);
    prefs.putBytes("remote_mac", remote_mac, 6);
    prefs.putBool("paired", true);
    if (!esp_now_is_peer_exist(remote_mac)) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, remote_mac, 6);
      peer.channel = 1;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
    }
    PairPacket ack = { PKT_PAIR_ACK };
    esp_now_send(remote_mac, (uint8_t*)&ack, sizeof(ack));
    remote_paired = true;
    pairing_enabled = false;
    Serial.printf("Paired with remote: %02X:%02X:%02X:%02X:%02X:%02X\n",
      remote_mac[0], remote_mac[1], remote_mac[2],
      remote_mac[3], remote_mac[4], remote_mac[5]);
  }

  // Handle incoming remote data
  if (remote_data_fresh) {
    remote_data_fresh = false;
    if (latest_remote.mode_request <= 2) {
      // Absolute mode 0=SHUTTLE, 1=JOYSTICK, 2=DIAL — idempotent if packet arrives twice
      currentMode = (ControlMode)latest_remote.mode_request;
      Serial.printf("Remote mode set: %d\n", (int)currentMode);
    }
  }

  // Pairing button (GPIO 13) — hold to enter pairing mode
  if (digitalRead(PAIR_BUTTON_PIN) == LOW) {
    pairing_enabled = true;
    Serial.println("Pairing mode active — waiting for remote...");
    delay(500); // debounce / rate limit
  }

  // Send status to remote
  send_status();

  // Always listen for commands and safety limits
  handleSerialCommands();
  checkLimitsAndCorrect();

  // Execute the logic for whatever mode is currently active
  switch (currentMode) {
  case MODE_SHUTTLE:
    handleShuttleMode();
    break;
  case MODE_JOYSTICK:
    handleJoystickMode();
    break;
  case MODE_POT:
    handleDialMode();
    break;
  case MODE_SERIAL:
    // Movement is entirely handled by handleSerialCommands() parsing
    break;
  }

  // Small delay to yield to the FreeRTOS watchdog and prevent crashes
  delay(10);
}
