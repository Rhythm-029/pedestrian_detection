/*
 * ESP32 starter firmware. Install ArduinoJson 7.x from Library Manager.
 * Fail-stopped: this starter deliberately does NOT drive motors.
 * Verify actual board, driver IC, pins and electrical levels before extending.
 */
#include <Arduino.h>
#include <ArduinoJson.h>

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;
static constexpr uint32_t RECOVERY_CLEAR_MS = 2000;

// UNVERIFIED placeholders: keep disabled until matched to config/pins.yaml and hardware.
static constexpr int LED_PIN = -1;
static constexpr int BUZZER_PIN = -1;
static constexpr bool MOTOR_OUTPUTS_ENABLED = false;

enum SafetyState { STOPPED, FAR, CAUTION, DANGER_LATCHED, COMMUNICATION_FAULT };
SafetyState state = STOPPED;
uint32_t lastValidMessageMs = 0;
uint32_t clearSinceMs = 0;
bool haveValidMessage = false;
String lineBuffer;

void allMotorsStop() {
  // Implement only after physical driver IC and pin mapping are confirmed.
  // TB6612FNG and L298N are NOT pin-compatible.
}

void setIndicators(bool ledOn, bool buzzerOn) {
  if (LED_PIN >= 0) digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
}

void enterStop(SafetyState reason) {
  allMotorsStop();
  state = reason;
  setIndicators(true, true);
}

bool parseAndHandle(const String &line) {
  JsonDocument doc;
  if (deserializeJson(doc, line)) {
    enterStop(COMMUNICATION_FAULT);
    return false;
  }
  if (doc["v"] != 1 || !doc["seq"].is<unsigned long>() ||
      !doc["zone"].is<const char *>() || !doc["heartbeat"].is<bool>() ||
      !doc["pedestrian"].is<bool>() || !doc["confidence"].is<float>()) {
    enterStop(COMMUNICATION_FAULT);
    return false;
  }

  const char *zone = doc["zone"];
  if (strcmp(zone, "FAR") && strcmp(zone, "CAUTION") && strcmp(zone, "DANGER")) {
    enterStop(COMMUNICATION_FAULT);
    return false;
  }

  lastValidMessageMs = millis();
  haveValidMessage = true;

  if (!strcmp(zone, "DANGER")) {
    clearSinceMs = 0;
    enterStop(DANGER_LATCHED);
    return true;
  }

  if (state == DANGER_LATCHED || state == COMMUNICATION_FAULT || state == STOPPED) {
    if (clearSinceMs == 0) clearSinceMs = millis();
    if (millis() - clearSinceMs < RECOVERY_CLEAR_MS) {
      allMotorsStop();
      setIndicators(true, false);
      return true;
    }
    state = !strcmp(zone, "CAUTION") ? CAUTION : FAR;
  } else {
    state = !strcmp(zone, "CAUTION") ? CAUTION : FAR;
  }

  allMotorsStop(); // Movement intentionally disabled in starter firmware.
  setIndicators(state != FAR, false);
  return true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  if (BUZZER_PIN >= 0) pinMode(BUZZER_PIN, OUTPUT);
  allMotorsStop();
  enterStop(STOPPED);
  Serial.println("BOOT: fail-stopped; motor output disabled; verify pins and driver.");
}

void loop() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      lineBuffer.trim();
      if (lineBuffer.length()) parseAndHandle(lineBuffer);
      lineBuffer = "";
    } else if (lineBuffer.length() < 512) {
      lineBuffer += c;
    } else {
      lineBuffer = "";
      enterStop(COMMUNICATION_FAULT);
    }
  }
  if (!haveValidMessage || millis() - lastValidMessageMs > HEARTBEAT_TIMEOUT_MS) {
    enterStop(COMMUNICATION_FAULT);
  }
  delay(2);
}
