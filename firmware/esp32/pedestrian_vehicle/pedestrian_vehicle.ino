/*
 * ESP32 Production Firmware for Pedestrian Safety Vehicle
 * Features:
 *   - UART JSON protocol v1 parser with heartbeat watchdog (500 ms)
 *   - Calibrated HC-SR04 ultrasonic distance verification (actual = 0.9975 * sensor + 2.11 cm)
 *   - Autonomous safety trigger: ultrasonic < 1.0 m forces latched emergency stop
 *   - Dual-zone motor control for TB6612FNG / L298N (Cruise in FAR, 40% speed in CAUTION, 0% in DANGER)
 *   - 2-second continuous clear path required to recover from emergency stop
 *   - Audible and visual alert indicator patterns (Buzzer & LED)
 */

#include <Arduino.h>
#include <ArduinoJson.h>

// ============================================================================
// CONFIGURATION & PIN DEFINITIONS
// Match with config/pins.yaml. Adjust to your physical wiring.
// ============================================================================
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;
static constexpr uint32_t RECOVERY_CLEAR_MS = 2000;
static constexpr uint32_t ULTRASONIC_INTERVAL_MS = 50; // Sample ultrasonic at 20 Hz

// Safety threshold: 1.0 m (100 cm) is the DANGER boundary
static constexpr float ULTRASONIC_DANGER_THRESHOLD_CM = 100.0f;

// Calibration parameters from bench dataset:
// actual_cm = (SLOPE * sensor_reading_cm) + INTERCEPT
static constexpr float ULTRASONIC_SLOPE = 0.9975f;
static constexpr float ULTRASONIC_INTERCEPT = 2.1117f;

// Pin Definitions (Standard ESP32 WROOM default pinout)
// Set to -1 if an individual pin is not connected.
static constexpr int PIN_LED = 2;        // Status LED
static constexpr int PIN_BUZZER = 4;     // Active buzzer
static constexpr int PIN_TRIG = 5;       // HC-SR04 Trig (Output)
static constexpr int PIN_ECHO = 18;      // HC-SR04 Echo (Input via voltage divider <= 3.3V!)

// TB6612FNG Motor Driver Pins
static constexpr int PIN_STBY = 19;      // TB6612FNG Standby (HIGH = active)
static constexpr int PIN_AIN1 = 21;      // Left Motor Dir 1
static constexpr int PIN_AIN2 = 22;      // Left Motor Dir 2
static constexpr int PIN_PWMA = 23;      // Left Motor PWM Speed
static constexpr int PIN_BIN1 = 25;      // Right Motor Dir 1
static constexpr int PIN_BIN2 = 26;      // Right Motor Dir 2
static constexpr int PIN_PWMB = 27;      // Right Motor PWM Speed

// Safety Switch: Set to true once verified on elevated wheels
static constexpr bool MOTOR_OUTPUTS_ENABLED = true;

// PWM Speed Levels (0 - 255)
static constexpr int PWM_CRUISE_SPEED = 180;                     // Normal cruise in FAR zone
static constexpr int PWM_CAUTION_SPEED = (PWM_CRUISE_SPEED * 40) / 100; // 40% speed in CAUTION zone

// ============================================================================
// STATE DEFINITIONS
// ============================================================================
enum SafetyState {
  STOPPED,
  FAR,
  CAUTION,
  DANGER_LATCHED,
  COMMUNICATION_FAULT
};

SafetyState state = STOPPED;
SafetyState previousState = STOPPED;

uint32_t lastValidMessageMs = 0;
uint32_t clearSinceMs = 0;
bool haveValidMessage = false;
String lineBuffer;

// Ultrasonic sensor state
float latestUltrasonicDistanceCm = 999.0f;
uint32_t lastUltrasonicSampleMs = 0;

// Non-blocking indicator blink timer
uint32_t lastBlinkToggleMs = 0;
bool blinkState = false;

// ============================================================================
// MOTOR CONTROL PRIMITIVES
// ============================================================================
void allMotorsStop() {
  if (PIN_STBY >= 0) digitalWrite(PIN_STBY, LOW);
  if (PIN_AIN1 >= 0) digitalWrite(PIN_AIN1, LOW);
  if (PIN_AIN2 >= 0) digitalWrite(PIN_AIN2, LOW);
  if (PIN_BIN1 >= 0) digitalWrite(PIN_BIN1, LOW);
  if (PIN_BIN2 >= 0) digitalWrite(PIN_BIN2, LOW);
  if (PIN_PWMA >= 0) analogWrite(PIN_PWMA, 0);
  if (PIN_PWMB >= 0) analogWrite(PIN_PWMB, 0);
}

void setMotorSpeeds(int leftPwm, int rightPwm) {
  if (!MOTOR_OUTPUTS_ENABLED) {
    allMotorsStop();
    return;
  }

  leftPwm = constrain(leftPwm, -255, 255);
  rightPwm = constrain(rightPwm, -255, 255);

  if (PIN_STBY >= 0) digitalWrite(PIN_STBY, HIGH);

  // Left Motor
  if (leftPwm > 0) {
    if (PIN_AIN1 >= 0) digitalWrite(PIN_AIN1, HIGH);
    if (PIN_AIN2 >= 0) digitalWrite(PIN_AIN2, LOW);
    if (PIN_PWMA >= 0) analogWrite(PIN_PWMA, leftPwm);
  } else if (leftPwm < 0) {
    if (PIN_AIN1 >= 0) digitalWrite(PIN_AIN1, LOW);
    if (PIN_AIN2 >= 0) digitalWrite(PIN_AIN2, HIGH);
    if (PIN_PWMA >= 0) analogWrite(PIN_PWMA, -leftPwm);
  } else {
    if (PIN_AIN1 >= 0) digitalWrite(PIN_AIN1, LOW);
    if (PIN_AIN2 >= 0) digitalWrite(PIN_AIN2, LOW);
    if (PIN_PWMA >= 0) analogWrite(PIN_PWMA, 0);
  }

  // Right Motor
  if (rightPwm > 0) {
    if (PIN_BIN1 >= 0) digitalWrite(PIN_BIN1, HIGH);
    if (PIN_BIN2 >= 0) digitalWrite(PIN_BIN2, LOW);
    if (PIN_PWMB >= 0) analogWrite(PIN_PWMB, rightPwm);
  } else if (rightPwm < 0) {
    if (PIN_BIN1 >= 0) digitalWrite(PIN_BIN1, LOW);
    if (PIN_BIN2 >= 0) digitalWrite(PIN_BIN2, HIGH);
    if (PIN_PWMB >= 0) analogWrite(PIN_PWMB, -rightPwm);
  } else {
    if (PIN_BIN1 >= 0) digitalWrite(PIN_BIN1, LOW);
    if (PIN_BIN2 >= 0) digitalWrite(PIN_BIN2, LOW);
    if (PIN_PWMB >= 0) analogWrite(PIN_PWMB, 0);
  }
}

// ============================================================================
// ULTRASONIC SENSOR DRIVER & CALIBRATION
// ============================================================================
float readUltrasonicRawCm() {
  if (PIN_TRIG < 0 || PIN_ECHO < 0) return 999.0f;

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Timeout of 25000 us (~4.3 meters max range)
  unsigned long durationUs = pulseIn(PIN_ECHO, HIGH, 25000);
  if (durationUs == 0) {
    return 999.0f; // Out of range or no echo received
  }

  // Speed of sound: 343 m/s => 29.1 us/cm round trip => duration / 58.2
  float rawDistanceCm = (float)durationUs / 58.2f;
  return rawDistanceCm;
}

float readUltrasonicCalibratedCm() {
  float rawCm = readUltrasonicRawCm();
  if (rawCm >= 400.0f || rawCm <= 1.0f) {
    return rawCm; // Pass through out-of-range or invalid signals
  }
  // Apply linear regression model derived from bench calibration dataset
  float calibratedCm = (rawCm * ULTRASONIC_SLOPE) + ULTRASONIC_INTERCEPT;
  return calibratedCm;
}

void updateUltrasonic() {
  uint32_t now = millis();
  if (now - lastUltrasonicSampleMs < ULTRASONIC_INTERVAL_MS) {
    return;
  }
  lastUltrasonicSampleMs = now;

  float sample = readUltrasonicCalibratedCm();
  // Simple outlier filter: only update if reasonable
  if (sample > 1.0f) {
    latestUltrasonicDistanceCm = sample;
  }

  // INDEPENDENT SAFETY TRIGGER:
  // If the ultrasonic sensor detects an obstacle closer than 1.0 m (100 cm),
  // immediately trigger DANGER_LATCHED regardless of Pi messages.
  if (latestUltrasonicDistanceCm < ULTRASONIC_DANGER_THRESHOLD_CM && state != DANGER_LATCHED) {
    clearSinceMs = 0;
    enterStop(DANGER_LATCHED);
    Serial.print("WARN: Ultrasonic independent trigger fired! Distance: ");
    Serial.print(latestUltrasonicDistanceCm);
    Serial.println(" cm");
  }
}

// ============================================================================
// INDICATOR CONTROL
// ============================================================================
void updateIndicators() {
  uint32_t now = millis();

  switch (state) {
    case FAR:
      if (PIN_LED >= 0) digitalWrite(PIN_LED, LOW);
      if (PIN_BUZZER >= 0) digitalWrite(PIN_BUZZER, LOW);
      break;

    case CAUTION:
      // Beep and blink at 4 Hz (every 125 ms toggle)
      if (now - lastBlinkToggleMs >= 125) {
        lastBlinkToggleMs = now;
        blinkState = !blinkState;
        if (PIN_LED >= 0) digitalWrite(PIN_LED, blinkState ? HIGH : LOW);
        if (PIN_BUZZER >= 0) digitalWrite(PIN_BUZZER, blinkState ? HIGH : LOW);
      }
      break;

    case DANGER_LATCHED:
      // Solid LED, continuous loud alarm
      if (PIN_LED >= 0) digitalWrite(PIN_LED, HIGH);
      if (PIN_BUZZER >= 0) digitalWrite(PIN_BUZZER, HIGH);
      break;

    case COMMUNICATION_FAULT:
    case STOPPED:
      // Fast alert flash
      if (now - lastBlinkToggleMs >= 200) {
        lastBlinkToggleMs = now;
        blinkState = !blinkState;
        if (PIN_LED >= 0) digitalWrite(PIN_LED, blinkState ? HIGH : LOW);
        if (PIN_BUZZER >= 0) digitalWrite(PIN_BUZZER, LOW);
      }
      break;
  }
}

// ============================================================================
// STATE MACHINE TRANSITIONS
// ============================================================================
void enterStop(SafetyState reason) {
  allMotorsStop();
  state = reason;
}

void applyVehicleActuation() {
  switch (state) {
    case FAR:
      setMotorSpeeds(PWM_CRUISE_SPEED, PWM_CRUISE_SPEED);
      break;
    case CAUTION:
      setMotorSpeeds(PWM_CAUTION_SPEED, PWM_CAUTION_SPEED);
      break;
    case DANGER_LATCHED:
    case COMMUNICATION_FAULT:
    case STOPPED:
    default:
      allMotorsStop();
      break;
  }
}

// ============================================================================
// PROTOCOL PARSING & VALIDATION
// ============================================================================
bool parseAndHandle(const String &line) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    enterStop(COMMUNICATION_FAULT);
    return false;
  }

  // Protocol v1 validation
  if (doc["v"] != 1 ||
      !doc["seq"].is<unsigned long>() ||
      !doc["zone"].is<const char *>() ||
      !doc["heartbeat"].is<bool>() ||
      !doc["pedestrian"].is<bool>() ||
      !doc["confidence"].is<float>()) {
    enterStop(COMMUNICATION_FAULT);
    return false;
  }

  const char *zone = doc["zone"];
  if (strcmp(zone, "FAR") != 0 && strcmp(zone, "CAUTION") != 0 && strcmp(zone, "DANGER") != 0) {
    enterStop(COMMUNICATION_FAULT);
    return false;
  }

  lastValidMessageMs = millis();
  haveValidMessage = true;

  // 1. Camera reports DANGER -> instant latch
  if (strcmp(zone, "DANGER") == 0) {
    clearSinceMs = 0;
    enterStop(DANGER_LATCHED);
    return true;
  }

  // 2. Ultrasonic reports DANGER -> keep latched
  if (latestUltrasonicDistanceCm < ULTRASONIC_DANGER_THRESHOLD_CM) {
    clearSinceMs = 0;
    enterStop(DANGER_LATCHED);
    return true;
  }

  // 3. Recovery check from DANGER_LATCHED, COMMUNICATION_FAULT, or STOPPED
  if (state == DANGER_LATCHED || state == COMMUNICATION_FAULT || state == STOPPED) {
    // Both camera must be FAR and ultrasonic must be clear
    bool pathClear = (strcmp(zone, "FAR") == 0 && latestUltrasonicDistanceCm >= ULTRASONIC_DANGER_THRESHOLD_CM);

    if (pathClear) {
      if (clearSinceMs == 0) {
        clearSinceMs = millis();
      }
      if (millis() - clearSinceMs >= RECOVERY_CLEAR_MS) {
        // Recovery satisfied! Re-arm to FAR or CAUTION
        state = (strcmp(zone, "CAUTION") == 0) ? CAUTION : FAR;
        clearSinceMs = 0;
      } else {
        // Still within 2.0 s recovery latch period
        allMotorsStop();
        return true;
      }
    } else {
      // Path not clear: reset recovery timer
      clearSinceMs = 0;
      allMotorsStop();
      return true;
    }
  } else {
    // Normal operation: transition between FAR and CAUTION
    state = (strcmp(zone, "CAUTION") == 0) ? CAUTION : FAR;
  }

  applyVehicleActuation();
  return true;
}

// ============================================================================
// MAIN SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Indicators
  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);
  if (PIN_BUZZER >= 0) pinMode(PIN_BUZZER, OUTPUT);

  // Ultrasonic
  if (PIN_TRIG >= 0) pinMode(PIN_TRIG, OUTPUT);
  if (PIN_ECHO >= 0) pinMode(PIN_ECHO, INPUT);

  // Motor Driver Pins
  if (PIN_STBY >= 0) pinMode(PIN_STBY, OUTPUT);
  if (PIN_AIN1 >= 0) pinMode(PIN_AIN1, OUTPUT);
  if (PIN_AIN2 >= 0) pinMode(PIN_AIN2, OUTPUT);
  if (PIN_PWMA >= 0) pinMode(PIN_PWMA, OUTPUT);
  if (PIN_BIN1 >= 0) pinMode(PIN_BIN1, OUTPUT);
  if (PIN_BIN2 >= 0) pinMode(PIN_BIN2, OUTPUT);
  if (PIN_PWMB >= 0) pinMode(PIN_PWMB, OUTPUT);

  allMotorsStop();
  enterStop(STOPPED);

  Serial.println("=================================================");
  Serial.println("BOOT: Pedestrian Safety Vehicle ESP32 Initialized");
  Serial.println("Status: FAIL-STOPPED. Waiting for UART & Clear Path");
  Serial.print("Ultrasonic Calibration Model: actual = ");
  Serial.print(ULTRASONIC_SLOPE, 4);
  Serial.print(" * sensor + ");
  Serial.print(ULTRASONIC_INTERCEPT, 4);
  Serial.println(" cm");
  Serial.println("=================================================");
}

void loop() {
  // 1. Process UART Serial Input
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      lineBuffer.trim();
      if (lineBuffer.length() > 0) {
        parseAndHandle(lineBuffer);
      }
      lineBuffer = "";
    } else if (lineBuffer.length() < 512) {
      lineBuffer += c;
    } else {
      lineBuffer = "";
      enterStop(COMMUNICATION_FAULT);
    }
  }

  // 2. Update Ultrasonic distance & safety trigger
  updateUltrasonic();

  // 3. Heartbeat Watchdog: Stop vehicle if no valid message within 500 ms
  if (!haveValidMessage || (millis() - lastValidMessageMs > HEARTBEAT_TIMEOUT_MS)) {
    if (state != COMMUNICATION_FAULT) {
      enterStop(COMMUNICATION_FAULT);
    }
  }

  // 4. Update Indicators (LED & Buzzer alert patterns)
  updateIndicators();

  delay(2);
}
