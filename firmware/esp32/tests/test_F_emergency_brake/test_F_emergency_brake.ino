/*
 * TEST F: Emergency Braking Test
 * Runs at medium speed. Sends emergency stop. Verifies:
 *   - Motors stop immediately.
 *   - Emergency state is latched (no auto-restart).
 *   - Explicit 'r' command required to clear latch.
 * Type 's' at any time to trigger emergency stop.
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

static constexpr int RUN_PWM = 128; // 50%

bool g_emergencyLatched = false;

void motorSetup() {
  ledcSetup(LEDC_CH_A, 5000, 8); ledcSetup(LEDC_CH_B, 5000, 8);
  ledcAttachPin(PIN_PWMA, LEDC_CH_A); ledcAttachPin(PIN_PWMB, LEDC_CH_B);
  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);
}

void emergencyStop(const char* reason) {
  ledcWrite(LEDC_CH_A, 0); ledcWrite(LEDC_CH_B, 0);
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
  g_emergencyLatched = true;
  Serial.print(">>> EMERGENCY STOP LATCHED — reason: "); Serial.println(reason);
  Serial.println(">>> Send 'r' to reset latch. Latch will NOT auto-clear.");
}

void setup() {
  Serial.begin(115200); motorSetup();
  ledcWrite(LEDC_CH_A, 0); ledcWrite(LEDC_CH_B, 0);
  Serial.println("TEST F: Emergency Braking Test");
  Serial.println("MOTOR_ENABLED: " + String(MOTOR_ENABLED ? "YES" : "NO"));
  Serial.println("Send 'g' to run at 50% PWM. 's' to trigger emergency stop. 'r' to reset latch.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  if (c == 'r' || c == 'R') {
    if (g_emergencyLatched) {
      g_emergencyLatched = false;
      Serial.println(">>> Latch cleared. Send 'g' to run again.");
    }
    return;
  }

  if (c == 's' || c == 'S') {
    emergencyStop("manual command");
    return;
  }

  if ((c == 'g' || c == 'G') && !g_emergencyLatched) {
    if (!MOTOR_ENABLED) { Serial.println("[SAFETY] Set MOTOR_ENABLED=true first."); return; }
    Serial.println("Running at 50% PWM... send 's' to trigger emergency stop.");
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
    ledcWrite(LEDC_CH_A, RUN_PWM); ledcWrite(LEDC_CH_B, RUN_PWM);

    uint32_t start = millis();
    while (!g_emergencyLatched) {
      if (millis() - start > 10000) {
        emergencyStop("10s timeout");
        break;
      }
      if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S') { emergencyStop("manual command"); break; }
      }
      delay(10);
    }
  } else if (g_emergencyLatched) {
    Serial.println("Latch is active. Send 'r' to reset first.");
  }
}
