/*
 * TEST D: Gradual Acceleration
 * Ramps PWM from 0 → 50% → 0 smoothly. No sudden starts.
 * Type 's' to stop at any time.
 */

#include <Arduino.h>

static constexpr int PIN_PWMA = 25;
static constexpr int PIN_AIN1 = 26;
static constexpr int PIN_AIN2 = 27;
static constexpr int PIN_PWMB = 33;
static constexpr int PIN_BIN1 = 32;
static constexpr int PIN_BIN2 = 19;
static constexpr int LEDC_CH_A = 0;
static constexpr int LEDC_CH_B = 1;
static constexpr bool MOTOR_ENABLED = false;

static constexpr int   RAMP_MAX_PWM  = 128; // 50% — conservative test limit
static constexpr int   RAMP_STEP     = 3;   // PWM units per step
static constexpr int   RAMP_DELAY_MS = 30;  // ms between steps

void motorSetup() {
  ledcSetup(LEDC_CH_A, 5000, 8); ledcSetup(LEDC_CH_B, 5000, 8);
  ledcAttachPin(PIN_PWMA, LEDC_CH_A); ledcAttachPin(PIN_PWMB, LEDC_CH_B);
  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);
}

void stopAll() {
  ledcWrite(LEDC_CH_A, 0); ledcWrite(LEDC_CH_B, 0);
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
}

bool checkStop() {
  if (Serial.available() && (Serial.read() == 's')) { stopAll(); Serial.println("STOP"); return true; }
  return false;
}

void setup() {
  Serial.begin(115200); motorSetup(); stopAll();
  Serial.println("TEST D: Gradual Acceleration");
  Serial.println("MOTOR_ENABLED: " + String(MOTOR_ENABLED ? "YES" : "NO"));
  Serial.println("Send 'g' to ramp up. 's' to stop.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c != 'g' && c != 'G') return;
  if (!MOTOR_ENABLED) { Serial.println("[SAFETY] Set MOTOR_ENABLED=true first."); return; }

  Serial.println("Ramping UP 0 → " + String(RAMP_MAX_PWM));
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);

  for (int pwm = 0; pwm <= RAMP_MAX_PWM; pwm += RAMP_STEP) {
    if (checkStop()) return;
    ledcWrite(LEDC_CH_A, pwm); ledcWrite(LEDC_CH_B, pwm);
    Serial.print("PWM="); Serial.println(pwm);
    delay(RAMP_DELAY_MS);
  }

  Serial.println("Hold at max for 2s...");
  uint32_t t = millis();
  while (millis() - t < 2000) { if (checkStop()) return; delay(10); }

  Serial.println("Ramping DOWN " + String(RAMP_MAX_PWM) + " → 0");
  for (int pwm = RAMP_MAX_PWM; pwm >= 0; pwm -= RAMP_STEP) {
    if (checkStop()) return;
    ledcWrite(LEDC_CH_A, pwm); ledcWrite(LEDC_CH_B, pwm);
    Serial.print("PWM="); Serial.println(pwm);
    delay(RAMP_DELAY_MS);
  }
  stopAll();
  Serial.println("TEST D COMPLETE — motors stopped.");
}
