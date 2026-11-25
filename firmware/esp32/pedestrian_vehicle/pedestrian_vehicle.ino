/*
 * Pedestrian Safety Vehicle — ESP32 Production Firmware v2.0
 * Hardware: ESP32 DevKit V1 + TB6612FNG + HC-SR04 + 3-pin Buzzer + LED
 *
 * WIRING (authoritative — see docs/GPIO_MAPPING.md):
 *   PWMA  → GPIO25   AIN1 → GPIO26   AIN2 → GPIO27
 *   PWMB  → GPIO33   BIN1 → GPIO32   BIN2 → GPIO19
 *   STBY  → 3V3 rail (hardware tie — NOT software-controlled)
 *   Buzzer IO → GPIO22
 *   LED       → GPIO2  (⚠️ CONFIRM before use)
 *   HC-SR04 TRIG → GPIO5   ECHO → GPIO18 (⚠️ CONFIRM; ECHO needs voltage divider)
 *   UART RX0 → GPIO3   TX0 → GPIO1  (Pi UART — shared with USB Serial Monitor)
 *
 * Safety:
 *   - Fail-stopped at boot — motors stay off until valid Pi messages arrive
 *   - 300 ms heartbeat watchdog → COMM_FAULT + motor stop
 *   - Piecewise-calibrated HC-SR04 near-field safety (10–75 cm range)
 *   - Ultrasonic danger threshold: 50 cm (conservative bench value)
 *   - DANGER_LATCHED: 2 s continuous clear path required to recover
 *   - MOTOR_OUTPUTS_ENABLED = false by default
 */

#include <Arduino.h>
#include <ArduinoJson.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

static constexpr uint32_t SERIAL_BAUD = 115200;

// Motor Driver (TB6612FNG) — STBY is 3V3 hardware tie, NOT a GPIO
static constexpr int PIN_PWMA = 25;   // Channel A PWM  (LEDC ch 0)
static constexpr int PIN_AIN1 = 26;   // Channel A dir1
static constexpr int PIN_AIN2 = 27;   // Channel A dir2
static constexpr int PIN_PWMB = 33;   // Channel B PWM  (LEDC ch 1)
static constexpr int PIN_BIN1 = 32;   // Channel B dir1
static constexpr int PIN_BIN2 = 19;   // Channel B dir2

static constexpr int LEDC_CH_A     = 0;
static constexpr int LEDC_CH_B     = 1;
static constexpr int LEDC_FREQ_HZ  = 5000;
static constexpr int LEDC_RES_BITS = 8;  // 0–255

// Indicators
static constexpr int  PIN_LED    = 23;   // Status LED — CONFIRMED GPIO23
static constexpr int  PIN_BUZZER = 22;   // Buzzer IO (3-pin module)

// true  = buzzer on when IO HIGH (active-HIGH module)
// false = buzzer on when IO LOW  (active-LOW  module)
// Confirm polarity from module marking before power-on.
static constexpr bool BUZZER_ACTIVE_HIGH = true;

// HC-SR04 (⚠️ ECHO must have 5V→3.3V voltage divider before GPIO18)
static constexpr int PIN_TRIG = 5;
static constexpr int PIN_ECHO = 18;

// ============================================================================
// SAFETY PARAMETERS
// ============================================================================

static constexpr uint32_t HEARTBEAT_TIMEOUT_MS         = 300;   // ms — spec requirement
static constexpr uint32_t RECOVERY_CLEAR_MS            = 2000;  // ms — 2 s clear-path recovery
static constexpr uint32_t ULTRASONIC_INTERVAL_MS       = 50;    // ms — 20 Hz sampling
static constexpr float    ULTRASONIC_DANGER_THRESHOLD_CM = 50.0f; // cm — conservative bench value

// Set to true only after hardware validation with wheels raised.
static constexpr bool MOTOR_OUTPUTS_ENABLED = false;

// PWM speed levels (8-bit, 0–255)
static constexpr int PWM_CRUISE_SPEED  = 180;
static constexpr int PWM_CAUTION_SPEED = (PWM_CRUISE_SPEED * 40) / 100; // 40% of cruise

// ============================================================================
// PIECEWISE LINEAR ULTRASONIC CALIBRATION
// 8 bench points; maps raw sensor reading (cm) → actual distance (cm)
// Calibrated range: raw 8.08–73.45 cm → actual 10–75 cm
// Readings outside this range are flagged, not silently extrapolated.
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
static constexpr int CAL_POINTS = 8;
static constexpr float ULTRASONIC_OUT_OF_RANGE_CM = 999.0f;

enum class UltrasonicStatus { OK, BELOW_RANGE, ABOVE_RANGE, TIMEOUT };

struct UltrasonicResult {
  float raw_cm;
  float calibrated_cm;
  UltrasonicStatus status;
};

float piecewiseCalibrate(float raw_cm) {
  if (raw_cm <= CAL_TABLE[0].raw_cm) {
    float slope = (CAL_TABLE[1].actual_cm - CAL_TABLE[0].actual_cm) /
                  (CAL_TABLE[1].raw_cm    - CAL_TABLE[0].raw_cm);
    return CAL_TABLE[0].actual_cm + slope * (raw_cm - CAL_TABLE[0].raw_cm);
  }
  if (raw_cm >= CAL_TABLE[CAL_POINTS-1].raw_cm) {
    int n = CAL_POINTS - 1;
    float slope = (CAL_TABLE[n].actual_cm - CAL_TABLE[n-1].actual_cm) /
                  (CAL_TABLE[n].raw_cm    - CAL_TABLE[n-1].raw_cm);
    return CAL_TABLE[n].actual_cm + slope * (raw_cm - CAL_TABLE[n].raw_cm);
  }
  for (int i = 0; i < CAL_POINTS - 1; i++) {
    if (raw_cm >= CAL_TABLE[i].raw_cm && raw_cm <= CAL_TABLE[i+1].raw_cm) {
      float t = (raw_cm - CAL_TABLE[i].raw_cm) /
                (CAL_TABLE[i+1].raw_cm - CAL_TABLE[i].raw_cm);
      return CAL_TABLE[i].actual_cm + t * (CAL_TABLE[i+1].actual_cm - CAL_TABLE[i].actual_cm);
    }
  }
  return raw_cm;
}

// ============================================================================
// STATE MACHINE
// ============================================================================
enum SafetyState {
  STATE_BOOT_STOPPED,
  STATE_FAR,
  STATE_CAUTION,
  STATE_DANGER_LATCHED,
  STATE_COMM_FAULT
};

SafetyState g_state          = STATE_BOOT_STOPPED;
uint32_t    g_lastValidMsgMs = 0;
uint32_t    g_clearSinceMs   = 0;
bool        g_haveValidMsg   = false;
String      g_lineBuffer;

UltrasonicResult g_latestUS = {ULTRASONIC_OUT_OF_RANGE_CM,
                                ULTRASONIC_OUT_OF_RANGE_CM,
                                UltrasonicStatus::TIMEOUT};
uint32_t g_lastUltrasonicMs = 0;
uint32_t g_lastBlinkMs      = 0;
bool     g_blinkState       = false;

// ============================================================================
// LEDC PWM
// ============================================================================
void motorPwmSetup() {
  ledcSetup(LEDC_CH_A, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcSetup(LEDC_CH_B, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(PIN_PWMA, LEDC_CH_A);
  ledcAttachPin(PIN_PWMB, LEDC_CH_B);
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================
void allMotorsStop() {
  ledcWrite(LEDC_CH_A, 0);
  ledcWrite(LEDC_CH_B, 0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
}

void setChannelA(int pwm) {
  if (!MOTOR_OUTPUTS_ENABLED) { allMotorsStop(); return; }
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
    ledcWrite(LEDC_CH_A, pwm);
  } else if (pwm < 0) {
    digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, HIGH);
    ledcWrite(LEDC_CH_A, -pwm);
  } else {
    digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
    ledcWrite(LEDC_CH_A, 0);
  }
}

void setChannelB(int pwm) {
  if (!MOTOR_OUTPUTS_ENABLED) { allMotorsStop(); return; }
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
    ledcWrite(LEDC_CH_B, pwm);
  } else if (pwm < 0) {
    digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(LEDC_CH_B, -pwm);
  } else {
    digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
    ledcWrite(LEDC_CH_B, 0);
  }
}

void setMotorSpeeds(int leftPwm, int rightPwm) {
  setChannelA(leftPwm);
  setChannelB(rightPwm);
}

// ============================================================================
// INDICATORS
// ============================================================================
inline void buzzerWrite(bool on) {
  bool level = BUZZER_ACTIVE_HIGH ? on : !on;
  digitalWrite(PIN_BUZZER, level ? HIGH : LOW);
}

void updateIndicators() {
  uint32_t now = millis();
  switch (g_state) {
    case STATE_FAR:
      if (PIN_LED >= 0) digitalWrite(PIN_LED, LOW);
      buzzerWrite(false);
      break;
    case STATE_CAUTION:
      if (now - g_lastBlinkMs >= 125) {
        g_lastBlinkMs = now;
        g_blinkState  = !g_blinkState;
        if (PIN_LED >= 0) digitalWrite(PIN_LED, g_blinkState);
        buzzerWrite(g_blinkState);
      }
      break;
    case STATE_DANGER_LATCHED:
      if (PIN_LED >= 0) digitalWrite(PIN_LED, HIGH);
      buzzerWrite(true);
      break;
    case STATE_COMM_FAULT:
    case STATE_BOOT_STOPPED:
    default:
      if (now - g_lastBlinkMs >= 100) {
        g_lastBlinkMs = now;
        g_blinkState  = !g_blinkState;
        if (PIN_LED >= 0) digitalWrite(PIN_LED, g_blinkState);
        buzzerWrite(false); // Silent during fault blink
      }
      break;
  }
}

// ============================================================================
// ULTRASONIC UPDATE
// ============================================================================
void updateUltrasonic() {
  uint32_t now = millis();
  if (now - g_lastUltrasonicMs < ULTRASONIC_INTERVAL_MS) return;
  g_lastUltrasonicMs = now;

  UltrasonicResult r;
  if (PIN_TRIG < 0 || PIN_ECHO < 0) {
    r = {ULTRASONIC_OUT_OF_RANGE_CM, ULTRASONIC_OUT_OF_RANGE_CM, UltrasonicStatus::TIMEOUT};
    g_latestUS = r;
    return;
  }

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (dur == 0) {
    r = {ULTRASONIC_OUT_OF_RANGE_CM, ULTRASONIC_OUT_OF_RANGE_CM, UltrasonicStatus::TIMEOUT};
    g_latestUS = r;
    return;
  }

  float raw = (float)dur / 58.2f;
  r.raw_cm       = raw;
  r.calibrated_cm = piecewiseCalibrate(raw);

  if (raw < CAL_TABLE[0].raw_cm)
    r.status = UltrasonicStatus::BELOW_RANGE;
  else if (raw > CAL_TABLE[CAL_POINTS-1].raw_cm)
    r.status = UltrasonicStatus::ABOVE_RANGE;
  else
    r.status = UltrasonicStatus::OK;

  g_latestUS = r;

  // Serial Monitor output
  Serial.print("[US] raw="); Serial.print(raw, 1);
  Serial.print("cm cal="); Serial.print(r.calibrated_cm, 1);
  Serial.print("cm ");
  switch (r.status) {
    case UltrasonicStatus::OK:          Serial.println("OK"); break;
    case UltrasonicStatus::BELOW_RANGE: Serial.println("BELOW_CAL_RANGE"); break;
    case UltrasonicStatus::ABOVE_RANGE: Serial.println("ABOVE_CAL_RANGE (not safety-critical)"); break;
    default: break;
  }

  // Independent near-field safety trigger (only on OK or BELOW_RANGE readings)
  bool validForSafety = (r.status == UltrasonicStatus::OK ||
                         r.status == UltrasonicStatus::BELOW_RANGE);
  if (validForSafety && r.calibrated_cm < ULTRASONIC_DANGER_THRESHOLD_CM &&
      g_state != STATE_DANGER_LATCHED) {
    g_clearSinceMs = 0;
    enterStop(STATE_DANGER_LATCHED);
    Serial.print("DANGER: Ultrasonic independent trigger! cal=");
    Serial.print(r.calibrated_cm, 1);
    Serial.print("cm < ");
    Serial.print(ULTRASONIC_DANGER_THRESHOLD_CM, 0);
    Serial.println("cm threshold");
  }
}

bool ultrasonicPathClear() {
  if (g_latestUS.status == UltrasonicStatus::ABOVE_RANGE ||
      g_latestUS.status == UltrasonicStatus::TIMEOUT) return true;
  return g_latestUS.calibrated_cm >= ULTRASONIC_DANGER_THRESHOLD_CM;
}

// ============================================================================
// STATE MACHINE HELPERS
// ============================================================================
void enterStop(SafetyState reason) {
  allMotorsStop();
  g_state = reason;
}

void applyVehicleActuation() {
  switch (g_state) {
    case STATE_FAR:     setMotorSpeeds(PWM_CRUISE_SPEED,  PWM_CRUISE_SPEED);  break;
    case STATE_CAUTION: setMotorSpeeds(PWM_CAUTION_SPEED, PWM_CAUTION_SPEED); break;
    default:            allMotorsStop();                                        break;
  }
}

// ============================================================================
// PROTOCOL PARSING
// ============================================================================
bool parseAndHandle(const String &line) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("PARSE_ERR: "); Serial.println(err.c_str());
    enterStop(STATE_COMM_FAULT);
    return false;
  }

  if (doc["v"] != 1 ||
      !doc["seq"].is<unsigned long>() ||
      !doc["zone"].is<const char*>() ||
      !doc["heartbeat"].is<bool>() ||
      !doc["pedestrian"].is<bool>() ||
      !doc["confidence"].is<float>()) {
    Serial.println("PARSE_ERR: missing/invalid fields");
    enterStop(STATE_COMM_FAULT);
    return false;
  }

  const char* zone = doc["zone"];
  if (strcmp(zone,"FAR")!=0 && strcmp(zone,"CAUTION")!=0 && strcmp(zone,"DANGER")!=0) {
    Serial.print("PARSE_ERR: bad zone: "); Serial.println(zone);
    enterStop(STATE_COMM_FAULT);
    return false;
  }

  g_lastValidMsgMs = millis();
  g_haveValidMsg   = true;

  Serial.print("[RX] seq="); Serial.print((unsigned long)doc["seq"]);
  Serial.print(" zone="); Serial.print(zone);
  Serial.print(" ped="); Serial.print((bool)doc["pedestrian"]);
  Serial.print(" conf="); Serial.println((float)doc["confidence"], 2);

  // 1. DANGER from camera → immediate latch
  if (strcmp(zone,"DANGER") == 0) {
    g_clearSinceMs = 0;
    enterStop(STATE_DANGER_LATCHED);
    Serial.println("DANGER: camera zone → latch");
    return true;
  }

  // 2. Ultrasonic still danger → keep latched
  if (!ultrasonicPathClear()) {
    g_clearSinceMs = 0;
    enterStop(STATE_DANGER_LATCHED);
    Serial.println("DANGER: ultrasonic still active → maintain latch");
    return true;
  }

  // 3. Recovery from latched states (requires both camera FAR AND ultrasonic clear)
  if (g_state == STATE_DANGER_LATCHED ||
      g_state == STATE_COMM_FAULT    ||
      g_state == STATE_BOOT_STOPPED) {

    bool camClear = (strcmp(zone,"FAR") == 0);
    if (camClear && ultrasonicPathClear()) {
      if (g_clearSinceMs == 0) {
        g_clearSinceMs = millis();
        Serial.println("RECOVERY: path clear — 2s timer started");
      }
      uint32_t elapsed = millis() - g_clearSinceMs;
      if (elapsed >= RECOVERY_CLEAR_MS) {
        Serial.println("RECOVERY: 2s satisfied → resuming FAR");
        g_state = STATE_FAR;
        g_clearSinceMs = 0;
        applyVehicleActuation();
      } else {
        Serial.print("RECOVERY: waiting "); Serial.print(elapsed); Serial.println("ms");
        allMotorsStop();
      }
    } else {
      g_clearSinceMs = 0;
      allMotorsStop();
    }
    return true;
  }

  // 4. Normal FAR ↔ CAUTION
  g_state = (strcmp(zone,"CAUTION")==0) ? STATE_CAUTION : STATE_FAR;
  applyVehicleActuation();
  return true;
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  if (PIN_LED    >= 0) { pinMode(PIN_LED,    OUTPUT); digitalWrite(PIN_LED,    LOW); }
  if (PIN_BUZZER >= 0) { pinMode(PIN_BUZZER, OUTPUT); buzzerWrite(false); }
  if (PIN_TRIG   >= 0) { pinMode(PIN_TRIG,   OUTPUT); digitalWrite(PIN_TRIG,   LOW); }
  if (PIN_ECHO   >= 0) { pinMode(PIN_ECHO,   INPUT); }

  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);

  motorPwmSetup();
  allMotorsStop();
  enterStop(STATE_BOOT_STOPPED);

  Serial.println("=========================================");
  Serial.println("BOOT: Pedestrian Safety Vehicle v2.0");
  Serial.println("State: BOOT_STOPPED — motors off");
  Serial.print  ("MOTOR_OUTPUTS_ENABLED: ");
  Serial.println(MOTOR_OUTPUTS_ENABLED ? "YES (wheels raised?)" : "NO (safe default)");
  Serial.print  ("Ultrasonic danger threshold: ");
  Serial.print  (ULTRASONIC_DANGER_THRESHOLD_CM, 0); Serial.println(" cm");
  Serial.print  ("Heartbeat timeout: ");
  Serial.print  (HEARTBEAT_TIMEOUT_MS); Serial.println(" ms");
  Serial.println("Ultrasonic: piecewise linear, 8 points, 10-75 cm");
  Serial.println("=========================================");
}

void loop() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      g_lineBuffer.trim();
      if (g_lineBuffer.length() > 0) parseAndHandle(g_lineBuffer);
      g_lineBuffer = "";
    } else if (g_lineBuffer.length() < 512) {
      g_lineBuffer += c;
    } else {
      g_lineBuffer = "";
      Serial.println("PARSE_ERR: buffer overflow");
      enterStop(STATE_COMM_FAULT);
    }
  }

  updateUltrasonic();

  if (g_haveValidMsg && (millis() - g_lastValidMsgMs > HEARTBEAT_TIMEOUT_MS)) {
    if (g_state != STATE_COMM_FAULT) {
      Serial.print("WATCHDOG: timeout after ");
      Serial.print(millis() - g_lastValidMsgMs);
      Serial.println("ms → COMM_FAULT");
      enterStop(STATE_COMM_FAULT);
    }
  }

  updateIndicators();
  delay(2);
}
