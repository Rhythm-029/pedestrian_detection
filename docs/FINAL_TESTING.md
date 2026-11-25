# Final Testing Plan
*Pedestrian Detection Vehicle — 20-Step Procedure*

> Do not check a step as done until it has been physically executed on hardware.

---

## Pre-Test Checklist

- [ ] Motor battery disconnected
- [ ] Wheels raised and clear
- [ ] Physical power cutoff switch accessible
- [ ] Serial Monitor open at 115200 baud
- [ ] `MOTOR_OUTPUTS_ENABLED = false` in firmware

---

## Step 1 — Verify GPIO Mapping

- [ ] Review `docs/GPIO_MAPPING.md`
- [ ] Check HC-SR04 ECHO voltage at ESP32 GPIO18 with multimeter (must be ≤ 3.3V)
- [ ] Confirm LED pin (GPIO2) does not hold line LOW during boot
- [ ] Confirm buzzer IO is on GPIO22
- [ ] Confirm common GND between all components

## Step 2 — Compile and Upload

- [ ] Upload `pedestrian_vehicle.ino` with motor battery disconnected
- [ ] Zero compiler errors and warnings

## Step 3 — Safe Startup Verification

- [ ] Serial Monitor shows `BOOT_STOPPED — motors off`
- [ ] `MOTOR_OUTPUTS_ENABLED: NO` confirmed in startup message
- [ ] No unexpected motor movement

## Step 4 — LED and Buzzer Test

- [ ] Upload `test_led_buzzer.ino`
- [ ] LED toggles with `l` command
- [ ] Buzzer toggles with `b` command
- [ ] Warning blink (4 Hz) works with `w`
- [ ] Emergency pattern works with `e`
- [ ] Note: confirm buzzer polarity matches `BUZZER_ACTIVE_HIGH` constant

## Step 5 — Ultrasonic Measurements

- [ ] Upload `test_ultrasonic.ino`
- [ ] Hold object at 10, 20, 30, 40, 50 cm
- [ ] Calibrated reading matches actual distance within 1 cm
- [ ] `TIMEOUT` appears when no object present
- [ ] `BELOW_CAL_RANGE` appears for very close objects (< 8 cm raw)

## Step 6 — Motor Channel A Test

- [ ] `MOTOR_OUTPUTS_ENABLED = true`, wheels raised
- [ ] Upload `test_A_two_motor.ino`
- [ ] Send `g` — left bank motors spin at 20%, 30%, 50%
- [ ] Motors stop between steps
- [ ] Send `s` — immediate stop

## Step 7 — Motor Channel B Test

- [ ] Right bank motors spin at 20%, 30%, 50%
- [ ] Direction is consistent with expected forward motion
- [ ] Send `s` — immediate stop

## Step 8 — Four Motors Together

- [ ] Both channels run simultaneously
- [ ] No overheating of TB6612FNG or motors
- [ ] All four wheels spin in correct direction

## Step 9 — Speed Profile Test

- [ ] Upload `test_C_speed_control.ino`
- [ ] Verify 20%, 30%, 50% PWM levels spin at visibly different speeds
- [ ] Advance to 70% only after lower levels confirmed safe
- [ ] 85% only after 70% validated

## Step 10 — Gradual Acceleration

- [ ] Upload `test_D_acceleration.ino`
- [ ] Ramp from 0 to 50% — smooth, no jerking
- [ ] Hold at 50% for 2s
- [ ] Ramp back to 0 smoothly

## Step 11 — Gradual Deceleration

- [ ] Modify test_D to add a deceleration-only run
- [ ] Verify motors coast to stop smoothly

## Step 12 — Emergency Braking

- [ ] Upload `test_F_emergency_brake.ino`
- [ ] Run at 50%, send `s` — motors stop immediately (< 100 ms)
- [ ] State remains latched — no auto-restart
- [ ] Send `r` to reset latch — confirm state clears
- [ ] No unintended restart without explicit `r`

## Step 13 — Simulated Pi Messages

- [ ] Upload production `pedestrian_vehicle.ino`
- [ ] Run `python3 scripts/simulate_pi_messages.py --scenario demo --port <PORT>`
- [ ] Observe `[RX]` messages on Serial Monitor
- [ ] State transitions: BOOT_STOPPED → FAR → CAUTION → DANGER_LATCHED → (recovery)
- [ ] Verify heartbeat_timeout scenario triggers `COMM_FAULT` within 300–500 ms

## Step 14 — UART Heartbeat Timeout

- [ ] With real or simulated Pi sending FAR, kill the sender
- [ ] ESP32 must enter `COMM_FAULT` within 300 ms
- [ ] Motors must stop

## Step 15 — Raspberry Pi Camera Test

- [ ] Boot Pi, activate virtual environment
- [ ] Run `libcamera-hello --timeout 5000` — confirm camera streams
- [ ] Run `python3 src/raspberry_pi/pi_controller.py --camera picamera2 --model data/models/yolov8n.onnx`
- [ ] Confirm detections appear in console output

## Step 16 — YOLO Pedestrian Detection

- [ ] Walk in front of camera at 1–3 m
- [ ] Confirm detection output with confidence ≥ 0.5
- [ ] Confirm zone changes from FAR to CAUTION/DANGER as you approach

## Step 17 — Full UART Integration Test

- [ ] Pi running pi_controller.py → sending to ESP32 via serial
- [ ] Walk toward vehicle
- [ ] ESP32 Serial Monitor shows zone transitions matching Pi output
- [ ] Motors respond to zone changes (wheels raised)

## Step 18 — Full System Integration

- [ ] Camera → YOLO → distance → zone → debounce → UART → ESP32 → motor
- [ ] Ultrasonic running in parallel
- [ ] Walk toward vehicle: CAUTION (slowdown) → DANGER (stop) → recover after 2s clear

## Step 19 — Stopping Distance Measurements

- [ ] Vehicle moving at 20% PWM — measure stopping distance
- [ ] Repeat at 30%, 50%
- [ ] Record results and compare against 30 cm clearance target
- [ ] Adjust ultrasonic danger threshold based on measured stopping distances

## Step 20 — 20-Trial Pedestrian Stopping Trials

- [ ] Vehicle on flat surface, wheels down, low speed
- [ ] Pedestrian figure placed at known distance
- [ ] Run 20 controlled trials
- [ ] Record: stop distance, stopping clearance, false positives, missed detections
- [ ] **Target: ≥ 18/20 stops with ≥ 30 cm clearance**

> [!IMPORTANT]
> Do not claim the 18/20 requirement is met until these trials are physically conducted and documented.
