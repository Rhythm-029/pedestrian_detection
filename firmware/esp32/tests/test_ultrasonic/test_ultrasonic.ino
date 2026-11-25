/*
 * TEST: Ultrasonic Sensor
 * Displays raw and piecewise-calibrated HC-SR04 readings continuously.
 * Flags below-range, above-range, and timeout conditions.
 * Does NOT control motors.
 *
 * WIRING: TRIG → GPIO5, ECHO → GPIO18 (via voltage divider, 5V→3.3V)
 * Open Serial Monitor at 115200 baud.
 */

#include <Arduino.h>

static constexpr int PIN_TRIG = 5;
static constexpr int PIN_ECHO = 18;
static constexpr float OUT_OF_RANGE_CM = 999.0f;

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

float piecewiseCalibrate(float raw) {
  if (raw <= CAL_TABLE[0].raw_cm) {
    float s = (CAL_TABLE[1].actual_cm - CAL_TABLE[0].actual_cm) /
              (CAL_TABLE[1].raw_cm    - CAL_TABLE[0].raw_cm);
    return CAL_TABLE[0].actual_cm + s * (raw - CAL_TABLE[0].raw_cm);
  }
  if (raw >= CAL_TABLE[CAL_POINTS-1].raw_cm) {
    int n = CAL_POINTS-1;
    float s = (CAL_TABLE[n].actual_cm - CAL_TABLE[n-1].actual_cm) /
              (CAL_TABLE[n].raw_cm    - CAL_TABLE[n-1].raw_cm);
    return CAL_TABLE[n].actual_cm + s * (raw - CAL_TABLE[n].raw_cm);
  }
  for (int i = 0; i < CAL_POINTS-1; i++) {
    if (raw >= CAL_TABLE[i].raw_cm && raw <= CAL_TABLE[i+1].raw_cm) {
      float t = (raw - CAL_TABLE[i].raw_cm) / (CAL_TABLE[i+1].raw_cm - CAL_TABLE[i].raw_cm);
      return CAL_TABLE[i].actual_cm + t * (CAL_TABLE[i+1].actual_cm - CAL_TABLE[i].actual_cm);
    }
  }
  return raw;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT); digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  Serial.println("Ultrasonic Calibration Test (piecewise linear)");
  Serial.println("Calibrated range: 10–75 cm actual");
  Serial.println("Format: raw_cm | cal_cm | status");
  Serial.println("------------------------------------");
}

void loop() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 25000UL);

  if (dur == 0) {
    Serial.println("TIMEOUT | -- | TIMEOUT (no echo)");
  } else {
    float raw = (float)dur / 58.2f;
    float cal = piecewiseCalibrate(raw);
    Serial.print(raw, 1); Serial.print(" | ");
    Serial.print(cal, 1); Serial.print(" | ");
    if (raw < CAL_TABLE[0].raw_cm)
      Serial.println("BELOW_CAL_RANGE (extrapolated)");
    else if (raw > CAL_TABLE[CAL_POINTS-1].raw_cm)
      Serial.println("ABOVE_CAL_RANGE (extrapolated, not safety-critical)");
    else
      Serial.println("OK");
  }

  delay(200);
}
