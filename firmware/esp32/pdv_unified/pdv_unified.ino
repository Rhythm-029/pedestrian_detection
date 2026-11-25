/*
 * PDV Unified Firmware v1.0  — Pedestrian Detection Vehicle
 * ESP32 DevKit V1  |  Arduino Core  |  115200 baud
 *
 * WIRING (authoritative):
 *   PWMA → GPIO25   AIN1 → GPIO26   AIN2 → GPIO27
 *   PWMB → GPIO33   BIN1 → GPIO32   BIN2 → GPIO19
 *   STBY → 3V3 rail (hardware tie — NOT a GPIO)
 *   LED  → GPIO23   Buzzer IO → GPIO22
 *   HC-SR04 TRIG → GPIO5   ECHO → GPIO18 (needs 5V→3.3V divider)
 *   UART RX0 (GPIO3) ← Pi TX   |   TX0 (GPIO1) → Pi RX
 *
 * Serial Protocol (115200 baud, line-delimited):
 *   Web UI → ESP32:  GET_STATUS | GET_CAPABILITIES | LED_ON | LED_OFF |
 *                    LED_BLINK | BUZZER_ON | BUZZER_OFF | BUZZER_BEEP |
 *                    MOTOR_A_TEST:PWM | MOTOR_B_TEST:PWM | MOTOR_BOTH_TEST:PWM |
 *                    MOTOR_STOP | EMERGENCY_STOP | RESET_LATCH |
 *                    SET_MODE:DIAG | SET_MODE:AUTO
 *   Pi    → ESP32:   {"v":1,"seq":N,"zone":"FAR|CAUTION|DANGER",...}
 *   ESP32 → both:    ACK:CMD | NACK:CMD:reason | STATUS:... | CAPS:...
 *                    [US] | [RX] | [INFO] | [ERR]
 *
 * Safety rules:
 *   - Boot → BOOT_STOPPED (motors off)
 *   - AUTO mode: 300 ms watchdog; missing Pi heartbeat → COMM_FAULT + stop
 *   - DIAG mode: no Pi heartbeat required; direct USB commanding
 *   - E-STOP latches until path clear 2 s + RESET_LATCH command
 *   - Motor commands rejected in AUTO mode
 *   - Motor commands rejected if MOTOR_OUTPUTS_ENABLED = false
 *   - Motor tests auto-stop after 5 s (firmware-enforced)
 */

#include <Arduino.h>
#include <ArduinoJson.h>

// ============================================================================
// HARDWARE PINS
// ============================================================================
static constexpr int PIN_AIN1    = 26;
static constexpr int PIN_AIN2    = 27;
static constexpr int PIN_PWMA    = 25;   // LEDC ch 0
static constexpr int PIN_BIN1    = 32;
static constexpr int PIN_BIN2    = 19;
static constexpr int PIN_PWMB    = 33;   // LEDC ch 1
static constexpr int PIN_LED     = 23;   // Status LED (confirmed GPIO23)
static constexpr int PIN_BUZZER  = 22;   // Active-HIGH 3-pin buzzer
static constexpr int PIN_TRIG    = 5;
static constexpr int PIN_ECHO    = 18;   // ⚠️ ECHO via voltage divider

static constexpr bool BUZZER_ACTIVE_HIGH = true;

// LEDC
static constexpr int LEDC_CH_A     = 0;
static constexpr int LEDC_CH_B     = 1;
static constexpr int LEDC_FREQ_HZ  = 5000;
static constexpr int LEDC_RES_BITS = 8;

// ============================================================================
// SAFETY / TIMING CONSTANTS
// ============================================================================
static constexpr uint32_t BAUD                    = 115200;
static constexpr uint32_t HEARTBEAT_TIMEOUT_MS    = 300;
static constexpr uint32_t RECOVERY_CLEAR_MS       = 2000;
static constexpr uint32_t ULTRASONIC_INTERVAL_MS  = 50;
static constexpr float    US_DANGER_THRESHOLD_CM  = 50.0f;
static constexpr uint32_t STATUS_INTERVAL_MS      = 500;
static constexpr uint32_t MOTOR_TEST_TIMEOUT_MS   = 5000;  // firmware-enforced

// Set true only after hardware validation with wheels raised.
// Change to true in firmware AND confirm on web UI before any motor test.
static constexpr bool MOTOR_OUTPUTS_ENABLED = false;

// PWM speed levels
static constexpr int PWM_CRUISE  = 180;
static constexpr int PWM_CAUTION = (PWM_CRUISE * 40) / 100;

// ============================================================================
// PIECEWISE ULTRASONIC CALIBRATION (8 bench points, 10–75 cm)
// ============================================================================
struct CalPoint { float raw_cm; float actual_cm; };
static const CalPoint CAL_TABLE[] = {
  {  8.08f, 10.0f },
  { 18.76f, 20.0f },
  { 27.68f, 30.0f },
  { 37.47f, 40.0f },
  { 47.20f, 50.0f },
  { 57.56f, 60.0f },
  { 68.74f, 70.0f },
  { 73.45f, 75.0f },
};
static constexpr int   CAL_POINTS       = 8;
static constexpr float US_OUT_OF_RANGE  = 999.0f;

enum class USStatus { OK, BELOW_RANGE, ABOVE_RANGE, TIMEOUT };
struct USResult { float raw_cm; float cal_cm; USStatus status; };

float piecewiseCalibrate(float raw) {
  if (raw <= CAL_TABLE[0].raw_cm) {
    float s = (CAL_TABLE[1].actual_cm - CAL_TABLE[0].actual_cm) /
              (CAL_TABLE[1].raw_cm    - CAL_TABLE[0].raw_cm);
    return CAL_TABLE[0].actual_cm + s * (raw - CAL_TABLE[0].raw_cm);
  }
  int n = CAL_POINTS - 1;
  if (raw >= CAL_TABLE[n].raw_cm) {
    float s = (CAL_TABLE[n].actual_cm - CAL_TABLE[n-1].actual_cm) /
              (CAL_TABLE[n].raw_cm    - CAL_TABLE[n-1].raw_cm);
    return CAL_TABLE[n].actual_cm + s * (raw - CAL_TABLE[n].raw_cm);
  }
  for (int i = 0; i < CAL_POINTS - 1; i++) {
    if (raw >= CAL_TABLE[i].raw_cm && raw <= CAL_TABLE[i+1].raw_cm) {
      float t = (raw - CAL_TABLE[i].raw_cm) /
                (CAL_TABLE[i+1].raw_cm - CAL_TABLE[i].raw_cm);
      return CAL_TABLE[i].actual_cm + t * (CAL_TABLE[i+1].actual_cm - CAL_TABLE[i].actual_cm);
    }
  }
  return raw;
}

// ============================================================================
// STATE MACHINE
// ============================================================================
enum SafetyState {
  ST_BOOT_STOPPED,
  ST_FAR,
  ST_CAUTION,
  ST_DANGER_LATCHED,
  ST_COMM_FAULT,
  ST_DIAG         // Diagnostic / hardware-test mode
};

enum Mode { MODE_AUTO, MODE_DIAG };

SafetyState g_state        = ST_BOOT_STOPPED;
Mode        g_mode         = MODE_AUTO;
bool        g_estop        = false;         // E-STOP latched
bool        g_haveMsg      = false;
uint32_t    g_lastMsgMs    = 0;
uint32_t    g_clearSinceMs = 0;
String      g_lineBuf;

USResult    g_us         = { US_OUT_OF_RANGE, US_OUT_OF_RANGE, USStatus::TIMEOUT };
uint32_t    g_lastUSMs   = 0;
uint32_t    g_lastBlinkMs  = 0;
bool        g_blinkState   = false;
uint32_t    g_lastStatusMs = 0;

// LED / Buzzer state (for STATUS reporting)
bool g_ledOn    = false;
bool g_buzzerOn = false;
int  g_motorPWM = 0;

// Motor test watchdog
bool     g_motorTestActive = false;
uint32_t g_motorTestStartMs = 0;

// ============================================================================
// LOW-LEVEL MOTOR / INDICATOR HELPERS
// ============================================================================
inline void buzzerWrite(bool on) {
  bool level = BUZZER_ACTIVE_HIGH ? on : !on;
  digitalWrite(PIN_BUZZER, level ? HIGH : LOW);
  g_buzzerOn = on;
}

inline void ledWrite(bool on) {
  digitalWrite(PIN_LED, on ? HIGH : LOW);
  g_ledOn = on;
}

void allMotorsStop() {
  ledcWrite(LEDC_CH_A, 0); ledcWrite(LEDC_CH_B, 0);
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
  g_motorPWM = 0;
  g_motorTestActive = false;
}

void motorPwmSetup() {
  ledcSetup(LEDC_CH_A, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcSetup(LEDC_CH_B, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(PIN_PWMA, LEDC_CH_A);
  ledcAttachPin(PIN_PWMB, LEDC_CH_B);
}

void setChannelA(int pwm) {
  if (!MOTOR_OUTPUTS_ENABLED) { allMotorsStop(); return; }
  pwm = constrain(pwm, -255, 255);
  if      (pwm > 0) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  ledcWrite(LEDC_CH_A, pwm); }
  else if (pwm < 0) { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); ledcWrite(LEDC_CH_A, -pwm); }
  else              { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, LOW);  ledcWrite(LEDC_CH_A, 0); }
  g_motorPWM = abs(pwm);
}

void setChannelB(int pwm) {
  if (!MOTOR_OUTPUTS_ENABLED) { allMotorsStop(); return; }
  pwm = constrain(pwm, -255, 255);
  if      (pwm > 0) { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);  ledcWrite(LEDC_CH_B, pwm); }
  else if (pwm < 0) { digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); ledcWrite(LEDC_CH_B, -pwm); }
  else              { digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, LOW);  ledcWrite(LEDC_CH_B, 0); }
  g_motorPWM = abs(pwm);
}

void applyVehicleActuation() {
  if (!MOTOR_OUTPUTS_ENABLED) return;
  switch (g_state) {
    case ST_FAR:     setChannelA(PWM_CRUISE);  setChannelB(PWM_CRUISE);  break;
    case ST_CAUTION: setChannelA(PWM_CAUTION); setChannelB(PWM_CAUTION); break;
    default:         allMotorsStop(); break;
  }
}

void enterStop(SafetyState reason) {
  allMotorsStop();
  g_state = reason;
}

// ============================================================================
// INDICATOR STATE MACHINE
// ============================================================================
void updateIndicators() {
  if (g_mode == MODE_DIAG) return;  // Diag mode: indicators controlled by commands
  uint32_t now = millis();
  switch (g_state) {
    case ST_FAR:
      ledWrite(false); buzzerWrite(false);
      break;
    case ST_CAUTION:
      if (now - g_lastBlinkMs >= 125) {
        g_lastBlinkMs = now; g_blinkState = !g_blinkState;
        ledWrite(g_blinkState); buzzerWrite(g_blinkState);
      }
      break;
    case ST_DANGER_LATCHED:
      ledWrite(true); buzzerWrite(true);
      break;
    default: // BOOT, COMM_FAULT
      if (now - g_lastBlinkMs >= 100) {
        g_lastBlinkMs = now; g_blinkState = !g_blinkState;
        ledWrite(g_blinkState); buzzerWrite(false);
      }
      break;
  }
}

// ============================================================================
// STATUS / CAPABILITIES REPORTING
// ============================================================================
const char* stateName(SafetyState s) {
  switch (s) {
    case ST_BOOT_STOPPED:    return "BOOT_STOPPED";
    case ST_FAR:             return "FAR";
    case ST_CAUTION:         return "CAUTION";
    case ST_DANGER_LATCHED:  return "DANGER_LATCHED";
    case ST_COMM_FAULT:      return "COMM_FAULT";
    case ST_DIAG:            return "DIAG";
    default:                 return "UNKNOWN";
  }
}

void sendStatus() {
  Serial.print("STATUS:v=1");
  Serial.print(",mode=");   Serial.print(g_mode == MODE_DIAG ? "DIAG" : "AUTO");
  Serial.print(",state=");  Serial.print(stateName(g_state));
  Serial.print(",estop=");  Serial.print(g_estop ? 1 : 0);
  Serial.print(",led=");    Serial.print(g_ledOn ? 1 : 0);
  Serial.print(",buz=");    Serial.print(g_buzzerOn ? 1 : 0);
  Serial.print(",us_cal="); Serial.print(g_us.cal_cm, 1);
  Serial.print(",pwm=");    Serial.print(g_motorPWM);
  Serial.print(",moe=");    Serial.print(MOTOR_OUTPUTS_ENABLED ? 1 : 0);
  Serial.println();
}

void sendCapabilities() {
  Serial.print("CAPS:v=1");
  Serial.print(",motor=");  Serial.print(MOTOR_OUTPUTS_ENABLED ? 1 : 0);
  Serial.print(",led=1");
  Serial.print(",buzzer=1");
  Serial.print(",us=1");
  Serial.print(",diag=1");
  Serial.print(",estop=1");
  Serial.println();
}

// ============================================================================
// ULTRASONIC
// ============================================================================
void updateUltrasonic() {
  uint32_t now = millis();
  if (now - g_lastUSMs < ULTRASONIC_INTERVAL_MS) return;
  g_lastUSMs = now;

  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (dur == 0) {
    g_us = { US_OUT_OF_RANGE, US_OUT_OF_RANGE, USStatus::TIMEOUT };
    return;
  }
  float raw = (float)dur / 58.2f;
  float cal = piecewiseCalibrate(raw);
  USStatus st;
  if      (raw < CAL_TABLE[0].raw_cm)         st = USStatus::BELOW_RANGE;
  else if (raw > CAL_TABLE[CAL_POINTS-1].raw_cm) st = USStatus::ABOVE_RANGE;
  else                                          st = USStatus::OK;
  g_us = { raw, cal, st };

  Serial.print("[US] raw="); Serial.print(raw, 1);
  Serial.print("cm cal=");   Serial.print(cal, 1);
  Serial.print("cm ");
  switch (st) {
    case USStatus::OK:          Serial.println("OK"); break;
    case USStatus::BELOW_RANGE: Serial.println("BELOW_CAL_RANGE"); break;
    case USStatus::ABOVE_RANGE: Serial.println("ABOVE_CAL_RANGE"); break;
    default: Serial.println("TIMEOUT"); break;
  }

  // Independent near-field safety (not suppressed by DIAG mode — safety first)
  bool validForSafety = (st == USStatus::OK || st == USStatus::BELOW_RANGE);
  if (validForSafety && cal < US_DANGER_THRESHOLD_CM && g_state != ST_DANGER_LATCHED) {
    g_estop = true;
    enterStop(ST_DANGER_LATCHED);
    Serial.print("[INFO] DANGER: US trigger! cal="); Serial.print(cal, 1);
    Serial.print("cm < "); Serial.print(US_DANGER_THRESHOLD_CM, 0); Serial.println("cm");
  }
}

bool ultrasonicClear() {
  if (g_us.status == USStatus::ABOVE_RANGE || g_us.status == USStatus::TIMEOUT) return true;
  return g_us.cal_cm >= US_DANGER_THRESHOLD_CM;
}

// ============================================================================
// COMMAND PARSER  (web UI / Serial Monitor → ESP32)
// ============================================================================
void handleCommand(const String& cmd) {
  // ---- INFO & CAPS ----
  if (cmd == "GET_STATUS") {
    sendStatus();
    return;
  }
  if (cmd == "GET_CAPABILITIES") {
    sendCapabilities();
    return;
  }

  // ---- MODE ----
  if (cmd == "SET_MODE:DIAG") {
    if (g_estop) {
      Serial.println("NACK:SET_MODE:DIAG:ESTOP_ACTIVE_RESET_FIRST");
      return;
    }
    g_mode = MODE_DIAG;
    g_state = ST_DIAG;
    allMotorsStop();
    Serial.println("ACK:SET_MODE:DIAG");
    sendStatus();
    return;
  }
  if (cmd == "SET_MODE:AUTO") {
    g_mode = MODE_AUTO;
    g_state = ST_BOOT_STOPPED;
    g_haveMsg = false;
    allMotorsStop();
    Serial.println("ACK:SET_MODE:AUTO");
    sendStatus();
    return;
  }

  // ---- EMERGENCY STOP ----
  if (cmd == "EMERGENCY_STOP") {
    g_estop = true;
    enterStop(ST_DANGER_LATCHED);
    ledWrite(true); buzzerWrite(true);
    Serial.println("ACK:EMERGENCY_STOP");
    sendStatus();
    return;
  }

  // ---- RESET LATCH ----
  if (cmd == "RESET_LATCH") {
    if (!ultrasonicClear()) {
      Serial.println("NACK:RESET_LATCH:OBSTACLE_PRESENT");
      return;
    }
    g_estop = false;
    g_state = (g_mode == MODE_DIAG) ? ST_DIAG : ST_BOOT_STOPPED;
    g_clearSinceMs = 0;
    allMotorsStop();
    ledWrite(false); buzzerWrite(false);
    Serial.println("ACK:RESET_LATCH");
    sendStatus();
    return;
  }

  // ---- LED ----
  if (cmd == "LED_ON") {
    ledWrite(true);
    Serial.println("ACK:LED_ON");
    return;
  }
  if (cmd == "LED_OFF") {
    ledWrite(false);
    Serial.println("ACK:LED_OFF");
    return;
  }
  if (cmd == "LED_BLINK") {
    // 3 quick blinks as acknowledgment
    for (int i = 0; i < 3; i++) {
      ledWrite(true);  delay(100);
      ledWrite(false); delay(100);
    }
    Serial.println("ACK:LED_BLINK");
    return;
  }

  // ---- BUZZER ----
  if (cmd == "BUZZER_ON") {
    buzzerWrite(true);
    Serial.println("ACK:BUZZER_ON");
    return;
  }
  if (cmd == "BUZZER_OFF") {
    buzzerWrite(false);
    Serial.println("ACK:BUZZER_OFF");
    return;
  }
  if (cmd == "BUZZER_BEEP") {
    buzzerWrite(true);  delay(100);
    buzzerWrite(false);
    Serial.println("ACK:BUZZER_BEEP");
    return;
  }

  // ---- MOTOR TESTS ----
  // Motor commands require DIAG mode + MOTOR_OUTPUTS_ENABLED
  if (cmd.startsWith("MOTOR_A_TEST:") || cmd.startsWith("MOTOR_B_TEST:") ||
      cmd.startsWith("MOTOR_BOTH_TEST:")) {
    if (g_estop) {
      Serial.print("NACK:"); Serial.print(cmd); Serial.println(":ESTOP_ACTIVE");
      return;
    }
    if (g_mode != MODE_DIAG) {
      Serial.print("NACK:"); Serial.print(cmd); Serial.println(":REQUIRES_DIAG_MODE");
      return;
    }
    if (!MOTOR_OUTPUTS_ENABLED) {
      Serial.print("NACK:"); Serial.print(cmd); Serial.println(":MOTOR_OUTPUTS_DISABLED");
      return;
    }
    int colonPos = cmd.lastIndexOf(':');
    int pwm = constrain(cmd.substring(colonPos + 1).toInt(), 0, 255);
    if (cmd.startsWith("MOTOR_A_TEST:")) {
      setChannelA(pwm); setChannelB(0);
      g_motorTestActive = true; g_motorTestStartMs = millis();
      Serial.print("ACK:MOTOR_A_TEST:"); Serial.println(pwm);
    } else if (cmd.startsWith("MOTOR_B_TEST:")) {
      setChannelA(0); setChannelB(pwm);
      g_motorTestActive = true; g_motorTestStartMs = millis();
      Serial.print("ACK:MOTOR_B_TEST:"); Serial.println(pwm);
    } else {
      setChannelA(pwm); setChannelB(pwm);
      g_motorTestActive = true; g_motorTestStartMs = millis();
      Serial.print("ACK:MOTOR_BOTH_TEST:"); Serial.println(pwm);
    }
    return;
  }

  if (cmd == "MOTOR_STOP") {
    allMotorsStop();
    Serial.println("ACK:MOTOR_STOP");
    return;
  }

  // ---- UNKNOWN ----
  Serial.print("NACK:UNKNOWN:"); Serial.println(cmd);
}

// ============================================================================
// PI MESSAGE PARSER  (JSON zone messages from Raspberry Pi)
// ============================================================================
bool parsePiMessage(const String& line) {
  JsonDocument doc;
  if (deserializeJson(doc, line) != DeserializationError::Ok) return false;
  if (doc["v"] != 1 || !doc["zone"].is<const char*>()) return false;

  g_lastMsgMs = millis();
  g_haveMsg   = true;

  const char* zone = doc["zone"];
  Serial.print("[RX] seq="); Serial.print((unsigned long)doc["seq"]);
  Serial.print(" zone=");    Serial.print(zone);
  Serial.print(" ped=");     Serial.print((bool)doc["pedestrian"]);
  Serial.print(" conf=");    Serial.println((float)doc["confidence"], 2);

  if (g_mode == MODE_DIAG) return true;  // In DIAG mode, ignore zone routing
  if (g_estop)              return true;

  if (strcmp(zone, "DANGER") == 0) {
    g_clearSinceMs = 0;
    enterStop(ST_DANGER_LATCHED);
    Serial.println("[INFO] DANGER: camera zone → latch");
    return true;
  }
  if (!ultrasonicClear()) {
    enterStop(ST_DANGER_LATCHED);
    Serial.println("[INFO] DANGER: US still active → maintain latch");
    return true;
  }

  // Recovery from latched states
  if (g_state == ST_DANGER_LATCHED || g_state == ST_COMM_FAULT || g_state == ST_BOOT_STOPPED) {
    bool camClear = (strcmp(zone, "FAR") == 0);
    if (camClear && ultrasonicClear()) {
      if (g_clearSinceMs == 0) { g_clearSinceMs = millis(); Serial.println("[INFO] RECOVERY: 2s timer started"); }
      uint32_t elapsed = millis() - g_clearSinceMs;
      if (elapsed >= RECOVERY_CLEAR_MS) {
        Serial.println("[INFO] RECOVERY: 2s satisfied → resuming FAR");
        g_state = ST_FAR;
        g_clearSinceMs = 0;
        applyVehicleActuation();
      } else {
        Serial.print("[INFO] RECOVERY: waiting "); Serial.print(elapsed); Serial.println("ms");
        allMotorsStop();
      }
    } else {
      g_clearSinceMs = 0;
      allMotorsStop();
    }
    return true;
  }

  g_state = (strcmp(zone, "CAUTION") == 0) ? ST_CAUTION : ST_FAR;
  applyVehicleActuation();
  return true;
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(BAUD);

  pinMode(PIN_LED,    OUTPUT); ledWrite(false);
  pinMode(PIN_BUZZER, OUTPUT); buzzerWrite(false);
  pinMode(PIN_TRIG,   OUTPUT); digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,   INPUT);

  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);

  motorPwmSetup();
  allMotorsStop();
  enterStop(ST_BOOT_STOPPED);

  Serial.println("=========================================");
  Serial.println("BOOT: PDV Unified Firmware v1.0");
  Serial.println("GPIO: LED=23 BUZ=22 TRIG=5 ECHO=18");
  Serial.print  ("MOTOR_OUTPUTS_ENABLED: ");
  Serial.println(MOTOR_OUTPUTS_ENABLED ? "YES" : "NO (safe default)");
  Serial.print  ("US danger threshold: ");
  Serial.print  (US_DANGER_THRESHOLD_CM, 0); Serial.println(" cm");
  Serial.print  ("Heartbeat timeout: ");
  Serial.print  (HEARTBEAT_TIMEOUT_MS);     Serial.println(" ms");
  Serial.println("Type GET_CAPABILITIES to verify firmware");
  Serial.println("=========================================");

  // Send capabilities immediately at boot so web UI can configure itself
  sendCapabilities();
  sendStatus();
}

void loop() {
  // ---- Serial input ----
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      g_lineBuf.trim();
      if (g_lineBuf.length() > 0) {
        // Decide: JSON (from Pi) or plain command (from web UI / Serial Monitor)
        if (g_lineBuf.startsWith("{")) {
          if (!parsePiMessage(g_lineBuf) && g_mode == MODE_AUTO) {
            Serial.print("[ERR] PARSE_ERR: "); Serial.println(g_lineBuf.substring(0, 60));
            if (!g_estop) enterStop(ST_COMM_FAULT);
          }
        } else {
          handleCommand(g_lineBuf);
        }
      }
      g_lineBuf = "";
    } else if (g_lineBuf.length() < 512) {
      g_lineBuf += c;
    } else {
      g_lineBuf = "";
      Serial.println("[ERR] PARSE_ERR: buffer overflow");
    }
  }

  // ---- Motor test watchdog ----
  if (g_motorTestActive && (millis() - g_motorTestStartMs >= MOTOR_TEST_TIMEOUT_MS)) {
    allMotorsStop();
    Serial.println("ACK:MOTOR_STOP (test timeout 5s)");
  }

  // ---- Ultrasonic ----
  updateUltrasonic();

  // ---- Pi heartbeat watchdog (AUTO mode only) ----
  if (g_mode == MODE_AUTO && g_haveMsg &&
      (millis() - g_lastMsgMs > HEARTBEAT_TIMEOUT_MS)) {
    if (g_state != ST_COMM_FAULT && g_state != ST_DANGER_LATCHED) {
      Serial.print("[ERR] WATCHDOG: timeout ");
      Serial.print(millis() - g_lastMsgMs);
      Serial.println("ms → COMM_FAULT");
      enterStop(ST_COMM_FAULT);
    }
  }

  // ---- Indicators (AUTO mode managed here; DIAG mode via commands) ----
  if (g_mode == MODE_AUTO) updateIndicators();

  // ---- Periodic STATUS broadcast ----
  if (millis() - g_lastStatusMs >= STATUS_INTERVAL_MS) {
    g_lastStatusMs = millis();
    sendStatus();
  }

  delay(2);
}
