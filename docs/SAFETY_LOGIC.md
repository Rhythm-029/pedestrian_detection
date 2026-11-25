# Safety Logic Reference
*Pedestrian Detection Vehicle*

## Zone Definitions

| Zone | Trigger | Motor Action | Indicator |
|---|---|---|---|
| FAR | No pedestrian, or distance ≥ 2.0 m | Cruise (PWM 180/255) | LED off, buzzer off |
| CAUTION | Pedestrian detected, 1.0–2.0 m | 40% speed (PWM 72/255) | 4 Hz LED + buzzer blink |
| DANGER | Pedestrian < 1.0 m | Emergency stop — **latched** | LED solid on, buzzer continuous |
| BOOT_STOPPED | On boot, before first valid Pi message | Motors off | 5 Hz LED flash |
| COMM_FAULT | No valid message within 300 ms | Motors off | 5 Hz LED flash |

## Safety Hierarchy

```
1. ESP32 internal emergency stop (highest priority)
   ↓ DANGER_LATCHED overrides all motor commands
2. Ultrasonic independent trigger (< 50 cm calibrated)
   ↓ Fires even if Pi messages say FAR
3. Camera zone from Pi (DANGER / CAUTION / FAR)
4. Heartbeat watchdog (300 ms) → COMM_FAULT
5. Normal FAR ↔ CAUTION operation (lowest priority)
```

## DANGER Latch Recovery

DANGER is a **latching state**. Recovery requires **all** of:

1. Camera must report `FAR` (not CAUTION, not DANGER)
2. Ultrasonic must be clear (≥ 50 cm calibrated, or TIMEOUT/ABOVE_RANGE)
3. Both conditions must be sustained for **2.0 continuous seconds**
4. No automatic restart — recovery timer starts only when both conditions are met

## Heartbeat Watchdog

- Pi must send at least one valid JSON message within **300 ms**
- If the timeout expires: `COMM_FAULT` state, motors stop immediately
- Recovery from COMM_FAULT follows the same 2-second clear-path rule as DANGER_LATCHED

## Ultrasonic Safety (Independent of Pi)

- Operates at **20 Hz** (50 ms interval) — independent of Pi messages
- Uses **piecewise linear calibration** — 8 bench-measured points, 10–75 cm range
- Readings flagged as `BELOW_RANGE` are treated as danger (conservative — close obstacle)
- Readings flagged as `ABOVE_RANGE` or `TIMEOUT` are treated as clear (no obstacle)
- Trigger threshold: **50 cm** (bench-safe initial value — adjust after stopping-distance tests)

## Camera Detection Pipeline

```
Frame capture (10 Hz)
  → YOLOv8n ONNX inference (class 0 = person)
  → Closest pedestrian selected (largest bbox height)
  → Distance: d = k / h_bbox  (k = 310.5, 320×320 resolution)
  → Zone classification: FAR / CAUTION / DANGER
  → 3-frame debounce (escalate on 2/3 agree, de-escalate on 3/3 clear)
  → Protocol v1 JSON → UART → ESP32
```

## Three-Frame Debouncer Logic

| Frames in window | Decision |
|---|---|
| ≥ 2× DANGER | → DANGER |
| ≥ 2× (CAUTION + DANGER combined) | → CAUTION |
| 3× FAR | → FAR |
| All other | Maintain previous zone |

This prevents single-frame detection noise from triggering an emergency stop, while escalating rapidly on sustained threats.

## Physical Stopping Clearance Target

Final project requirement: **≥ 30 cm clearance** in ≥ 18 of 20 controlled trials.

> [!WARNING]
> This requirement has **not** been verified until stopping-distance tests at each speed profile are completed. The 50 cm ultrasonic threshold and camera zone boundaries are starting values, not validated safe braking distances.
