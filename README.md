# Pedestrian Detection Vehicle

Autonomous pedestrian detection and emergency braking vehicle using a Raspberry Pi 4 + Camera V2 (IMX219), ESP32 DevKit V1, HC-SR04 ultrasonic sensor, four DC gear motors (TB6612FNG driver), 3-pin buzzer, and LED.

> **Hardware status (2026-11-24):** All hardware physically connected and verified — motors slow down based on detected distance. ✅

---

## 🌐 Web UI (Quick Start)

The Web UI lets you monitor live sensor data, inject safety zones, run tests, and view the serial monitor — all from a browser.

```bash
cd /Users/yashwardhansingh/pedestrian_detection
python3 scripts/start_web_ui.py --port 8080
```
Then open: **http://localhost:8080**

> **Note:** macOS reserves port 5000 for AirPlay Receiver. Use `--port 8080` (or any free port).

### Web UI Features

| Panel | What it does |
|---|---|
| **Navbar** | Select ESP32 serial port + baud, scan ports, Connect/Disconnect |
| **Safety Zone** | Live colour-coded zone (FAR=green, CAUTION=amber, DANGER=red blinking) |
| **Camera Distance** | Pedestrian distance from Pi camera (pinhole model) |
| **Detection Confidence** | YOLO detection confidence + pedestrian present/absent |
| **ESP32 FSM State** | Inferred state machine state (FAR / CAUTION / DANGER_LATCHED / COMM_FAULT) |
| **Ultrasonic Panel** | Raw + calibrated HC-SR04 reading with visual 0–75 cm bar |
| **Zone Controller** | Inject FAR / CAUTION / DANGER messages (single send or 10 Hz heartbeat) |
| **Test Runner** | Run pytest suite + Pi simulation scenarios (demo / heartbeat_timeout / continuous_cruise) |
| **Serial Monitor** | Colour-coded live ESP32 serial output with auto-scroll |

---

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
