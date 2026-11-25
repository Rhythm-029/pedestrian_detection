# Motor Testing Guide
*Pedestrian Detection Vehicle — TB6612FNG, 4 DC Gear Motors*

> [!CAUTION]
> Never run motor tests without wheels raised. Keep a physical power cutoff switch within reach at all times.

## Wiring Confirmation

| TB6612FNG | ESP32 GPIO | Status |
|---|---|---|
| PWMA | 25 | ✅ Confirmed |
| AIN1 | 26 | ✅ Confirmed |
| AIN2 | 27 | ✅ Confirmed |
| PWMB | 33 | ✅ Confirmed |
| BIN1 | 32 | ✅ Confirmed |
| BIN2 | 19 | ✅ Confirmed |
| STBY | 3V3 rail | Hardware tie — not a GPIO |

## Motor Topology

One TB6612FNG controls two channels:
- **Channel A** → Left-front + Left-rear motors (wired in parallel)
- **Channel B** → Right-front + Right-rear motors (wired in parallel)

> [!WARNING]
> Verify the per-motor and combined stall current against the TB6612FNG rating (1.2A continuous, 3.2A peak per channel). Two motors in parallel draw double the single-motor stall current.

## Test Sketches

| Sketch | Location | Purpose |
|---|---|---|
| test_A_two_motor | `firmware/esp32/tests/test_A_two_motor/` | Channel A, B, then both |
| test_C_speed_control | `firmware/esp32/tests/test_C_speed_control/` | PWM sweep 20→85% |
| test_D_acceleration | `firmware/esp32/tests/test_D_acceleration/` | Ramp up and down |
| test_F_emergency_brake | `firmware/esp32/tests/test_F_emergency_brake/` | Latch test |

## Test Procedure

1. Upload sketch with motor battery **disconnected**.
2. Open Serial Monitor at 115200 baud.
3. Raise all four wheels clear of the bench.
4. Connect motor battery.
5. Send `g` to start. Send `s` to stop at any time.
6. Set `MOTOR_ENABLED = true` in the sketch before the test run.

## PWM Levels

| Label | PWM | % |
|---|---|---|
| Very slow | 51 | 20% |
| Slow | 77 | 30% |
| Medium | 128 | 50% |
| Fast | 179 | 70% |
| Max test | 217 | 85% — only after lower levels validated |

Do not jump to 85% without first validating 20%, 30%, and 50%.

## Expected Serial Output (Channel A test)

```
>>> A only  20% A=51 B=0 for 2000ms
>>> Step done — stopped
>>> A only  30% A=77 B=0 for 2000ms
...
```
