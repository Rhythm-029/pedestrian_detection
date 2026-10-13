#include <Arduino.h>
// USB serial echo test; no motor control. Serial Monitor: 115200 baud.
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 serial echo ready");
}
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    Serial.print("RX: ");
    Serial.println(line);
  }
}
