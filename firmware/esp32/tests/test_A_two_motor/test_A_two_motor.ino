/*
 * TEST A: Two-Motor Test (Channel A and Channel B individually, then together)
 *
 * Tests:
 *   - Motor channel A only (left bank)
 *   - Motor channel B only (right bank)
 *   - Both channels simultaneously
 *   - Forward at 20%, 30%, 50% PWM
 *   - Stop between each step
 *
 * USAGE:
 *   1. DISCONNECT motor battery. Upload this sketch.
 *   2. Open Serial Monitor at 115200 baud.
 *   3. Reconnect motor battery. Raise wheels.
 *   4. Type 'g' + Enter to begin the test sequence.
 *   5. Type 's' at any time to emergency-stop.
 *
 * WIRING (matches GPIO_MAPPING.md):
 *   PWMA → GPIO25 (LEDC ch 0)
 *   AIN1 → GPIO26   AIN2 → GPIO27
 *   PWMB → GPIO33 (LEDC ch 1)
 *   BIN1 → GPIO32   BIN2 → GPIO19
 *   STBY → 3V3 rail (not a GPIO)
 */

#include <Arduino.h>

// Pin definitions — must match production firmware
static constexpr int PIN_PWMA = 25;
static constexpr int PIN_AIN1 = 26;
static constexpr int PIN_AIN2 = 27;
static constexpr int PIN_PWMB = 33;
static constexpr int PIN_BIN1 = 32;
static constexpr int PIN_BIN2 = 19;

static constexpr int LEDC_CH_A    = 0;
static constexpr int LEDC_CH_B    = 1;
static constexpr int LEDC_FREQ    = 5000;
static constexpr int LEDC_RES     = 8;

// Safety: set true only after physical inspection with wheels raised
static constexpr bool MOTOR_ENABLED = false;

// PWM test levels
static constexpr int PWM_20 = 51;
static constexpr int PWM_30 = 77;
static constexpr int PWM_50 = 128;

static constexpr uint32_t STEP_MS  = 2000;  // Run each step for 2 s
static constexpr uint32_t STOP_MS  = 1000;  // Pause between steps

bool g_stopped = true;

void motorSetup() {
  ledcSetup(LEDC_CH_A, LEDC_FREQ, LEDC_RES);
  ledcSetup(LEDC_CH_B, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PIN_PWMA, LEDC_CH_A);
  ledcAttachPin(PIN_PWMB, LEDC_CH_B);
  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);
  ledcWrite(LEDC_CH_A, 0);
  ledcWrite(LEDC_CH_B, 0);
}

void stopAll() {
  ledcWrite(LEDC_CH_A, 0); ledcWrite(LEDC_CH_B, 0);
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
  g_stopped = true;
}

void runA(int pwm) {
  if (!MOTOR_ENABLED) { Serial.println("[SAFETY] MOTOR_ENABLED = false — skipping"); return; }
  g_stopped = false;
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  ledcWrite(LEDC_CH_A, pwm);
}

void runB(int pwm) {
  if (!MOTOR_ENABLED) { Serial.println("[SAFETY] MOTOR_ENABLED = false — skipping"); return; }
  g_stopped = false;
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  ledcWrite(LEDC_CH_B, pwm);
}

bool checkStop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') { stopAll(); Serial.println(">>> EMERGENCY STOP"); return true; }
  }
  return false;
}

void runStep(const char* label, int pwmA, int pwmB, uint32_t ms) {
  Serial.print(">>> "); Serial.print(label);
  Serial.print(" A="); Serial.print(pwmA);
  Serial.print(" B="); Serial.print(pwmB);
  Serial.print(" for "); Serial.print(ms); Serial.println("ms");
  if (pwmA > 0) runA(pwmA);
  if (pwmB > 0) runB(pwmB);
  uint32_t t = millis();
  while (millis() - t < ms) {
    if (checkStop()) return;
    delay(10);
  }
  stopAll();
  Serial.println(">>> Step done — stopped");
  delay(STOP_MS);
}

void setup() {
  Serial.begin(115200);
  motorSetup();
  stopAll();
  Serial.println("===========================================");
  Serial.println("TEST A: Two-Motor Test");
  Serial.println("MOTOR_ENABLED: " + String(MOTOR_ENABLED ? "YES" : "NO (safe default)"));
  Serial.println("Send 'g' to start, 's' to emergency-stop at any time.");
  Serial.println("VERIFY WHEELS ARE RAISED before starting.");
  Serial.println("===========================================");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c != 'g' && c != 'G') return;

  Serial.println("\n--- Test sequence starting ---");

  // Channel A only
  runStep("A only  20%", PWM_20, 0, STEP_MS);
  if (!g_stopped) stopAll(); if (!MOTOR_ENABLED) return;
  runStep("A only  30%", PWM_30, 0, STEP_MS);
  if (!g_stopped) stopAll(); if (!MOTOR_ENABLED) return;
  runStep("A only  50%", PWM_50, 0, STEP_MS);
  if (!g_stopped) stopAll(); if (!MOTOR_ENABLED) return;

  // Channel B only
  runStep("B only  20%", 0, PWM_20, STEP_MS);
  if (!g_stopped) stopAll(); if (!MOTOR_ENABLED) return;
  runStep("B only  30%", 0, PWM_30, STEP_MS);
  if (!g_stopped) stopAll(); if (!MOTOR_ENABLED) return;
  runStep("B only  50%", 0, PWM_50, STEP_MS);
  if (!g_stopped) stopAll(); if (!MOTOR_ENABLED) return;

  // Both together
  runStep("BOTH    20%", PWM_20, PWM_20, STEP_MS);
  if (!g_stopped) stopAll(); if (!MOTOR_ENABLED) return;
  runStep("BOTH    30%", PWM_30, PWM_30, STEP_MS);
  if (!g_stopped) stopAll(); if (!MOTOR_ENABLED) return;
  runStep("BOTH    50%", PWM_50, PWM_50, STEP_MS);
  if (!g_stopped) stopAll();

  Serial.println("\n--- TEST A COMPLETE — all motors stopped ---");
  Serial.println("Send 'g' to repeat, 's' to stop.");
}
