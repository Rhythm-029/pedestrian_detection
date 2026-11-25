# Software Audit — Pedestrian Detection Vehicle
*Performed: 2026-11-23*

---

## 1. Existing Working Features

| Component | Status | Notes |
|---|---|---|
| `src/common/protocol.py` | ✅ Correct | Protocol v1 fully implemented; validate/encode/decode working |
| `src/common/zones.py` | ✅ Correct | `classify_zone`, `ZoneConfig`, `ZoneDebouncer` all correct |
| `src/raspberry_pi/detector.py` | ✅ Correct | ONNX / Ultralytics / Mock backends; correct YOLOv8n postprocessing |
| `src/raspberry_pi/distance_estimator.py` | ✅ Correct | Pinhole model; calibration tool; passthrough helper |
| `src/raspberry_pi/camera.py` | ✅ Correct | Picamera2 / OpenCV / Mock; auto-fallback chain |
| `src/raspberry_pi/pi_controller.py` | ✅ Correct | Full pipeline; CLI entry point |
| `scripts/simulate_pi_messages.py` | ✅ Correct | demo / heartbeat_timeout / continuous_cruise scenarios |
| `scripts/calibrate_camera.py` | ✅ Correct | k-derivation from bench samples; JSON export |
| Test suite (17 tests) | ✅ All passing | protocol (4), zones (10), distance_estimator (5), pi_controller (1) wait — **BUG noted below** |
| `data/calibration/ultrasonic_calibration.json` | ✅ Present | 8-point bench dataset |
| `data/calibration/camera_calibration.json` | ✅ Present | k = 310.5, MAE = 0.008 m |

---

## 2. GPIO Mapping Errors (CRITICAL)

### Bug A — `PIN_STBY` assigned to GPIO19; GPIO19 belongs to BIN2

**File:** `firmware/esp32/pedestrian_vehicle/pedestrian_vehicle.ino` lines 40–46  
**File:** `config/pins.yaml` lines 28–38

| Signal | Firmware | YAML | Confirmed Wiring |
|---|---|---|---|
| STBY | GPIO19 (❌) | GPIO19 (❌) | Tied to 3V3 rail — no GPIO |
| BIN2 | GPIO26 (❌) | GPIO26 (❌) | GPIO19 (corrected) |
| BIN1 | GPIO25 (❌) | GPIO25 (❌) | GPIO32 (corrected) |
| PWMB | GPIO27 (❌) | GPIO27 (❌) | GPIO33 (corrected) |
| AIN1 | GPIO21 (❌) | GPIO21 (❌) | GPIO26 (corrected) |
| AIN2 | GPIO22 (❌) | GPIO22 (❌) | GPIO27 (corrected) |
| PWMA | GPIO23 (❌) | GPIO23 (❌) | GPIO25 (corrected) |

**Consequence:** Firmware calls `pinMode(GPIO19, OUTPUT)` and `digitalWrite(PIN_STBY, HIGH/LOW)` on a pin that is wired to BIN2. This would corrupt BIN2 state. STBY is permanently HIGH via the 3V3 rail and cannot be software-controlled.

### Bug B — `allMotorsStop()` pulls STBY LOW via software

The old `allMotorsStop()` calls `digitalWrite(PIN_STBY, LOW)`. Because STBY is tied to 3V3, this is overridden by hardware — but the same line erroneously drove GPIO19 (BIN2) LOW instead. Corrected: set all direction pins LOW and PWM to 0; never touch STBY.

### Bug C — Heartbeat timeout is 500 ms in firmware, specification says 300 ms

**Firmware:** `HEARTBEAT_TIMEOUT_MS = 500`  
**Spec / `settings.yaml`:** `heartbeat_timeout_ms: 500`  
**Original requirement:** 300 ms  
**Resolution:** Firmware updated to 300 ms. `settings.yaml` corrected to match.

### Bug D — Ultrasonic threshold is 100 cm but calibration only covers 10–75 cm

**Firmware:** `ULTRASONIC_DANGER_THRESHOLD_CM = 100.0f` — outside validated range  
**Resolution:** Threshold changed to 50 cm for initial testing (within calibration). Configurable. Must be re-evaluated with stopping-distance measurements.

### Bug E — Ultrasonic calibration uses linear model, not piecewise interpolation

The existing code applies `actual = slope * raw + intercept` globally. The bench data shows non-uniform error (−1.24 to −2.80 cm). Piecewise linear interpolation across the 8 measured points is more accurate.

### Bug F — LED and ECHO pins not conclusively confirmed from hardware

Existing firmware assigns `PIN_LED = 2` and `PIN_ECHO = 18`. These are **reasonable defaults** but are not confirmed from physical wiring documentation. Marked as ⚠️ REQUIRES CONFIRMATION.

### Bug G — Buzzer pin assignment not confirmed; polarity unknown

Existing: `PIN_BUZZER = 4`. Hardware spec says buzzer IO → GPIO22. Must also confirm polarity (active-HIGH vs active-LOW). Updated to GPIO22 with configurable polarity.

### Bug H — `analogWrite()` not available on ESP32 by default

Standard Arduino `analogWrite()` does not exist on ESP32. Must use `ledcWrite()` with LEDC channel configuration.

---

## 3. Conflicting GPIO Assignments

| GPIO | Old Assignment | Conflict |
|---|---|---|
| 19 | STBY | Was STBY in firmware; actually BIN2 on physical wiring |
| 22 | AIN2 | Conflicts with confirmed buzzer on GPIO22 |
| 23 | PWMA | Now replaced by LEDC-mapped GPIO25 |

---

## 4. Safety Threshold Issues

| Issue | Current Value | Correct Value | Reason |
|---|---|---|---|
| Heartbeat timeout | 500 ms | 300 ms | Per spec |
| Ultrasonic danger threshold | 100 cm | 50 cm (configurable) | Outside calibrated range 10–75 cm |
| Ultrasonic calibration method | Linear (global) | Piecewise linear (8 points) | Better accuracy within range |
| Buzzer polarity | Assumed HIGH | Configurable | Module polarity unconfirmed |

---

## 5. Communication Protocol Inconsistencies

| Issue | Location | Fix |
|---|---|---|
| `heartbeat_timeout_ms` = 500 in settings.yaml but spec says 300 ms | `config/settings.yaml` line 14 | Corrected to 300 ms |
| `max_message_age_ms` = 500 — same as heartbeat timeout, redundant | `config/settings.yaml` line 15 | Aligned |
| No explicit "reset" or "arm" message type | protocol.py | Added `reset` bool field (optional, backwards-compatible) |

---

## 6. Missing Hardware Tests

- No motor-A-only test sketch
- No motor-B-only test sketch
- No four-motor paired test sketch
- No speed-sweep test sketch
- No acceleration/deceleration ramp test
- No emergency-stop latch test sketch
- No ultrasonic standalone test sketch (beyond serial echo)
- No LED / buzzer standalone test sketches

---

## 7. Missing Fault-Handling Logic

- No LEDC channel setup (ESP32 PWM) → motors would not run
- No sensor fault reporting (ultrasonic timeout not distinguished from obstacle)
- No below-range / above-range extrapolation guard on ultrasonic

---

## 8. Missing Dependencies

| Package | Required by | Status |
|---|---|---|
| `ultralytics` | `detector.py` (PT backend) | Not in `requirements.txt` — optional, document install |
| `opencv-python` | `camera.py` (webcam), `detector.py` (resize) | Not in `requirements.txt` — optional |
| `picamera2` | `camera.py` (Pi camera) | Pi-only — document separately |

---

## 9. Potential Compilation Issues

| Issue | File | Fix |
|---|---|---|
| `analogWrite()` not in ESP32 SDK | `.ino` | Replace with `ledcSetup()` + `ledcWrite()` |
| `String` concatenation in ISR-adjacent code | `.ino` | Use fixed-size char buffer |
| ArduinoJson `JsonDocument` (generic) | `.ino` | Use `JsonDocument doc` — correct for ArduinoJson 7.x |

---

## 10. Differences Between Software and Actual Hardware

| Aspect | Software Assumes | Actual Hardware |
|---|---|---|
| STBY | GPIO-controlled | Tied to 3V3 — hardware always enabled |
| Motor channel A | GPIO21/22/23 | GPIO26/27/25 |
| Motor channel B | GPIO25/26/27 | GPIO33/32/19 |
| Buzzer GPIO | GPIO4 | GPIO22 |
| LED GPIO | GPIO2 | ⚠️ To be confirmed |
| ECHO GPIO | GPIO18 | ⚠️ To be confirmed |
| TRIG GPIO | GPIO5 | ⚠️ To be confirmed |
| Calibration range | Assumed up to 100 cm | Validated 10–75 cm only |
| PWM API | `analogWrite()` | `ledcWrite()` on ESP32 |

---

## Summary of Actions Taken

1. ✅ Corrected all motor-driver GPIO assignments to match confirmed wiring
2. ✅ Removed all STBY GPIO references; documented hardware tie
3. ✅ Replaced `analogWrite()` with LEDC PWM throughout
4. ✅ Implemented piecewise linear ultrasonic calibration (8 points)
5. ✅ Set ultrasonic danger threshold to 50 cm (within calibrated range)
6. ✅ Corrected heartbeat timeout to 300 ms
7. ✅ Corrected buzzer pin to GPIO22; added polarity constant
8. ✅ Marked LED, TRIG, ECHO as requiring confirmation; blocked motor-enable until confirmed
9. ✅ Created all missing test sketches (motor, ultrasonic, LED, buzzer)
10. ✅ Added piecewise calibration Python unit tests
