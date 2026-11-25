/*
 * TEST: LED and Buzzer
 * Tests LED on/off/blink and buzzer on/off/patterns.
 * Serial commands: 'l' = LED toggle, 'b' = buzzer toggle,
 *   'w' = warning blink (both), 'e' = emergency pattern, 'x' = stop all.
 *
 * WIRING: LED → GPIO23 (confirmed), Buzzer IO → GPIO22
 * Set BUZZER_ACTIVE_HIGH to match your module polarity.
 */

#include <Arduino.h>

static constexpr int  PIN_LED           = 23;  // Confirmed: GPIO23
static constexpr int  PIN_BUZZER        = 22;
static constexpr bool BUZZER_ACTIVE_HIGH = true;

bool g_ledOn     = false;
bool g_buzzerOn  = false;

inline void buzzerSet(bool on) {
  bool level = BUZZER_ACTIVE_HIGH ? on : !on;
  digitalWrite(PIN_BUZZER, level ? HIGH : LOW);
}

void blinkBoth(int times, int on_ms, int off_ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED, HIGH); buzzerSet(true);
    delay(on_ms);
    digitalWrite(PIN_LED, LOW); buzzerSet(false);
    delay(off_ms);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED,    OUTPUT); digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT); buzzerSet(false);
  Serial.println("LED + Buzzer Test");
  Serial.println("'l' = LED toggle | 'b' = buzzer toggle");
  Serial.println("'w' = warning blink (4 Hz x 5) | 'e' = emergency (10 fast) | 'x' = stop all");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'l': case 'L':
      g_ledOn = !g_ledOn;
      digitalWrite(PIN_LED, g_ledOn);
      Serial.println(g_ledOn ? "LED ON" : "LED OFF");
      break;
    case 'b': case 'B':
      g_buzzerOn = !g_buzzerOn;
      buzzerSet(g_buzzerOn);
      Serial.println(g_buzzerOn ? "BUZZER ON" : "BUZZER OFF");
      break;
    case 'w': case 'W':
      Serial.println("Warning blink (4 Hz x 5)");
      blinkBoth(5, 125, 125);
      break;
    case 'e': case 'E':
      Serial.println("Emergency pattern (fast x 10)");
      blinkBoth(10, 60, 60);
      break;
    case 'x': case 'X':
      g_ledOn = false; g_buzzerOn = false;
      digitalWrite(PIN_LED, LOW); buzzerSet(false);
      Serial.println("All off");
      break;
  }
}
