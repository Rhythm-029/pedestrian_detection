# Bring-up checklist

## A. Software-only on Mac
- [ ] Create virtual environment; install requirements-dev.txt.
- [ ] Run `python -m pytest`.
- [ ] Inspect protocol and zone tests.

## B. ESP32 only
- [ ] Confirm exact ESP32 board and serial port.
- [ ] Install Espressif board support and ArduinoJson 7.x.
- [ ] Compile/upload with motor battery disconnected.
- [ ] Confirm startup says fail-stopped and motor output disabled.

## C. Power checks
- [ ] Identify battery chemistry, voltage, charger and connector.
- [ ] Confirm driver IC marking on both boards.
- [ ] Check motor stall current against driver ratings.
- [ ] Verify buck output with multimeter and no Pi connected.
- [ ] Confirm fuse and switch ratings.
- [ ] Check continuity and polarity with all power disconnected.

## D. Sensors and motors
- [ ] LED uses a current-limiting resistor.
- [ ] Buzzer voltage/current are suitable.
- [ ] HC-SR04 Echo has a level shifter/divider.
- [ ] Test each motor individually with wheels raised and low PWM.
- [ ] Verify stop disables all motor outputs.

## E. Camera and integration
- [ ] Boot Pi from known-good storage.
- [ ] Verify Camera V2 using camera tools supported by installed Pi OS.
- [ ] Test saved images before live video.
- [ ] Validate detector and distance estimation.
- [ ] Test UART messages and heartbeat timeout.
- [ ] Verify stop latch and 2-second recovery.
- [ ] Low-speed trial in clear area with physical power cutoff.
