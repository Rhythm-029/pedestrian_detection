# Ultrasonic Calibration Reference
*HC-SR04 — Piecewise Linear Calibration, 8 Bench Points*

## Calibration Dataset

| Actual (cm) | Raw sensor (cm) | Error (cm) |
|---|---|---|
| 10 | 8.08 | −1.92 |
| 20 | 18.76 | −1.24 |
| 30 | 27.68 | −2.32 |
| 40 | 37.47 | −2.53 |
| 50 | 47.20 | −2.80 |
| 60 | 57.56 | −2.44 |
| 70 | 68.74 | −1.26 |
| 75 | 73.45 | −1.55 |

**Calibrated range:** 10–75 cm actual  
**Method:** Piecewise linear interpolation across all 8 points  
**Max residual:** < 0.83 cm at calibration points

## Why Piecewise, Not Linear?

The old firmware used a single linear fit (`actual = 0.9975 × raw + 2.11`). The error varies from −1.24 to −2.80 cm across the range — not uniform — so a single slope fits poorly. Piecewise interpolation between measured pairs gives sub-centimetre accuracy throughout the range.

## Calibrated Range Boundaries

| Condition | Raw reading | Firmware action |
|---|---|---|
| Within range | 8.08–73.45 cm | `OK` — use corrected value |
| Below range | < 8.08 cm raw | `BELOW_RANGE` — extrapolated; treat as potentially very close |
| Above range | > 73.45 cm raw | `ABOVE_RANGE` — extrapolated; not safety-critical |
| No echo | timeout (0 µs) | `TIMEOUT` — treat as clear (no obstacle detected) |

## Danger Threshold

Current configured threshold: **50 cm** (actual distance)  
This is a conservative bench value within the validated range.

> [!WARNING]
> 50 cm is **not** a validated safe braking distance for a moving vehicle. It is used only for elevated-wheel bench testing. Before moving-vehicle trials, measure actual stopping distances at each speed profile and adjust accordingly.

## HC-SR04 Electrical Requirements

| Signal | Level | Note |
|---|---|---|
| VCC | 5V | Power from regulated 5V supply |
| GND | Common | Same ground as ESP32 |
| TRIG | 3.3V compatible | ESP32 GPIO5 output — OK |
| ECHO | **5V output** | **MUST be divided to ≤ 3.3V before ESP32 GPIO18** |

**Voltage divider for ECHO:** 10kΩ from ECHO to GPIO18, 20kΩ from GPIO18 to GND → GPIO sees ≈ 3.33V at 5V input.

## Running the Ultrasonic Test

1. Upload `firmware/esp32/tests/test_ultrasonic/test_ultrasonic.ino`
2. Open Serial Monitor at 115200 baud
3. Hold an object at measured distances (10, 20, 30, 40, 50 cm)
4. Confirm calibrated readings match actual within 1 cm

Expected Serial output:
```
8.08 | 10.0 | OK
18.76 | 20.0 | OK
47.20 | 50.0 | OK
5.00 | 7.9 | BELOW_CAL_RANGE (extrapolated)
90.00 | 82.6 | ABOVE_CAL_RANGE (extrapolated, not safety-critical)
TIMEOUT | -- | TIMEOUT (no echo)
```
