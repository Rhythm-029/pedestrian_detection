# Project Completion Checklist
*Pedestrian Detection Vehicle*

## Software (✅ Complete)

- [x] Protocol v1 — encode, decode, validate
- [x] Zone classifier — FAR / CAUTION / DANGER with configurable thresholds
- [x] 3-frame ZoneDebouncer — conservative escalation, strict de-escalation
- [x] YOLOv8n detector — ONNX / Ultralytics / Mock backends
- [x] Pinhole distance estimator — d = k / h_bbox, k = 310.5
- [x] Camera abstraction — Picamera2 / OpenCV / Mock
- [x] Pi controller — full pipeline, CLI, 10 Hz loop
- [x] ESP32 firmware — corrected GPIO, piecewise ultrasonic, LEDC PWM, 300 ms watchdog
- [x] Motor test sketches — A, C, D, F
- [x] Ultrasonic test sketch
- [x] LED + buzzer test sketch
- [x] 60/60 automated tests passing
- [x] SOFTWARE_AUDIT.md
- [x] GPIO_MAPPING.md (authoritative)
- [x] config/pins.yaml (corrected)
- [x] config/settings.yaml (corrected)
- [x] ESP32_UPLOAD_GUIDE.md
- [x] MOTOR_TESTING.md
- [x] ULTRASONIC_CALIBRATION.md
- [x] RASPBERRY_PI_SETUP.md
- [x] PI_ESP32_CONNECTION.md
- [x] SAFETY_LOGIC.md
- [x] FINAL_TESTING.md

## Hardware Confirmations Required (⚠️ Not Yet Done)

- [ ] HC-SR04 ECHO voltage measured at GPIO18 (must be ≤ 3.3V)
- [ ] LED pin (GPIO2) confirmed — no boot conflict
- [ ] HC-SR04 TRIG/ECHO physical pin confirmed as GPIO5/GPIO18
- [ ] Buzzer polarity confirmed — set `BUZZER_ACTIVE_HIGH` correctly
- [ ] Common ground confirmed across all subsystems
- [ ] Motor driver current within TB6612FNG limits (two motors per channel)

## Hardware Tests Required (⚠️ Not Yet Run)

- [ ] LED + buzzer test (test_led_buzzer.ino)
- [ ] Ultrasonic measurements at known distances (test_ultrasonic.ino)
- [ ] Motor Channel A test (test_A_two_motor.ino)
- [ ] Motor Channel B test
- [ ] Four motors together
- [ ] Speed sweep 20% → 85%
- [ ] Gradual acceleration ramp
- [ ] Emergency stop latch verification
- [ ] Simulated Pi messages → ESP32 FSM verification
- [ ] Heartbeat timeout verification (< 300 ms)
- [ ] Full UART integration (Pi → ESP32)
- [ ] YOLO pedestrian detection
- [ ] Stopping distance measurements (all speed levels)
- [ ] 20 pedestrian stopping trials

## Final Performance Target

- [ ] ≥ 18/20 trials with ≥ 30 cm stopping clearance

**NOT claimed until physical trials complete.**

## First Test to Perform

**Upload `test_led_buzzer.ino` with motor battery disconnected.**  
Confirm LED and buzzer work on GPIO2 and GPIO22 before any other testing.  
This validates the two safest pins and confirms buzzer polarity.
