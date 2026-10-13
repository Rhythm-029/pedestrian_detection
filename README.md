# Pedestrian Detection Vehicle

Starter repository for a Raspberry Pi 4 + Camera Module V2 (IMX219), ESP32, HC-SR04 ultrasonic sensor, four geared DC motors, and two motor-driver boards.

## Architecture
- Raspberry Pi: camera capture, pedestrian detection, distance estimate, safety-zone decision, UART transport.
- ESP32: message validation, heartbeat timeout, safety state machine, motor stop, indicators.
- Protocol: newline-delimited JSON over 115200-baud UART.
- Zones: FAR, CAUTION below 2.0 m, DANGER below 1.0 m. DANGER latches a stop; recovery requires 2 seconds continuously clear.

This is an educational prototype, not a safety-certified vehicle. Test with wheels raised, low motor power, a clear area, and a physical power cut-off.

## Mac setup
Requires Python 3.10+ and Git:
```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements-dev.txt
python -m pytest
```

## ESP32 setup on your friend's laptop
1. Install Arduino IDE 2.x and Espressif ESP32 board support.
2. Install ArduinoJson 7.x using Library Manager.
3. Open `firmware/esp32/pedestrian_vehicle/pedestrian_vehicle.ino`.
4. Verify `config/pins.yaml` against the physical board. Do not use placeholder pins.
5. Keep motor battery disconnected for first firmware upload.
6. The starter intentionally does not drive motors; motor output stays disabled until pin mapping and driver chip are confirmed.

## Critical hardware notes
- Your invoice description contains both TB6612FNG and L298N wording. Verify the actual chip marking on each board before wiring; pinouts and electrical behavior differ.
- HC-SR04 Echo is typically 5 V: use a divider/level shifter before an ESP32 GPIO.
- Do not power motors from the Pi's 5 V rail. Use a properly rated, fused motor supply and a stable 5 V supply for the Pi.
- A slotted disc without an optical sensor does not provide encoder readings.
- Camera V2 uses the Pi CSI connector, not USB. Confirm cable orientation and latch before powering.

## Git workflow
```bash
git status
git add README.md .gitignore config docs src firmware tests scripts requirements-dev.txt requirements.txt
git commit -m "Add pedestrian vehicle starter architecture"
git push
```
Do not commit credentials, virtual environments, private images, or large model weights.
