/*
 * TEST C: PWM Speed Control Test
 * Sweeps through 20%, 30%, 50%, 70%, 85% PWM on both channels.
 * Prints current PWM to Serial Monitor at each step.
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
static constexpr bool MOTOR_ENABLED = false; // Set true after hardware validation

struct SpeedLevel { const char* name; int pwm; };
static const SpeedLevel LEVELS[] = {
  { "Very slow (20%)", 51  },
  { "Slow      (30%)", 77  },
  { "Medium    (50%)", 128 },
  { "Fast      (70%)", 179 },
  { "Max test  (85%)", 217 },
};

void motorSetup() {
  ledcSetup(LEDC_CH_A, 5000, 8);
  ledcSetup(LEDC_CH_B, 5000, 8);
  ledcAttachPin(PIN_PWMA, LEDC_CH_A);
  ledcAttachPin(PIN_PWMB, LEDC_CH_B);
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
  Serial.begin(115200);
  motorSetup(); stopAll();
  Serial.println("TEST C: PWM Speed Control Test");
  Serial.println("MOTOR_ENABLED: " + String(MOTOR_ENABLED ? "YES" : "NO"));
  Serial.println("Send 'g' to start sweep. 's' to stop. 85% only after lower speeds validated.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c != 'g' && c != 'G') return;
  if (!MOTOR_ENABLED) { Serial.println("[SAFETY] Set MOTOR_ENABLED=true after validation."); return; }

  for (const auto& lvl : LEVELS) {
    Serial.print("PWM level: "); Serial.print(lvl.name); Serial.print(" = "); Serial.println(lvl.pwm);
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
    ledcWrite(LEDC_CH_A, lvl.pwm);
    ledcWrite(LEDC_CH_B, lvl.pwm);

    uint32_t t = millis();
    while (millis() - t < 2500) {
      if (checkStop()) return;
      delay(50);
    }
    stopAll();
    Serial.println("  Stopped. Next in 1s...");
    delay(1000);
  }
  Serial.println("TEST C COMPLETE.");
}
