# ESP32 Upload Guide
*Pedestrian Detection Vehicle — ESP32 DevKit V1*

## Prerequisites

1. **Arduino IDE 2.x** — download from https://www.arduino.cc/en/software
2. **Espressif ESP32 Board Support:**
   - In Arduino IDE: `File → Preferences → Additional Board Manager URLs`
   - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - `Tools → Board → Board Manager` → search "esp32" → install **esp32 by Espressif Systems**
3. **ArduinoJson 7.x:**
   - `Sketch → Include Library → Manage Libraries` → search "ArduinoJson" → install version **7.x**

## Board Settings

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module |
| Upload Speed | 921600 |
| CPU Frequency | 240 MHz |
| Flash Mode | QIO |
| Flash Size | 4MB |
| Partition Scheme | Default |
| Core Debug Level | None |

## Upload Procedure

> [!CAUTION]
> Disconnect the **motor battery** before uploading. Only USB power should be active.

1. Connect ESP32 to your computer via USB.
2. Select the correct port under `Tools → Port` (usually `/dev/cu.usbserial-*` on Mac or `COM*` on Windows).
3. Open `firmware/esp32/pedestrian_vehicle/pedestrian_vehicle.ino`.
4. Click **Verify** (✓) first — confirm zero errors before uploading.
5. Click **Upload** (→).
6. If upload fails: hold the ESP32 BOOT button while clicking Upload, release when "Connecting..." appears.

## Verify Successful Boot

1. Open `Tools → Serial Monitor`, set baud to `115200`.
2. Press the ESP32 RST button.
3. Confirm you see:
```
=========================================
BOOT: Pedestrian Safety Vehicle v2.0
State: BOOT_STOPPED — motors off
MOTOR_OUTPUTS_ENABLED: NO (safe default)
Ultrasonic danger threshold: 50 cm
Heartbeat timeout: 300 ms
Ultrasonic: piecewise linear, 8 points, 10-75 cm
=========================================
Waiting for Pi UART messages...
```

> [!IMPORTANT]
> `MOTOR_OUTPUTS_ENABLED: NO` is correct and expected. Motors will not move until you change this constant and re-upload **after physical validation**.

## Enabling Motor Outputs

Only after:
- [ ] All GPIO pins confirmed with multimeter
- [ ] HC-SR04 voltage divider verified (ECHO ≤ 3.3V at ESP32)
- [ ] Wheels raised and clear
- [ ] Physical power cutoff switch on motor battery line

Then:
1. Open the `.ino` file.
2. Change `static constexpr bool MOTOR_OUTPUTS_ENABLED = false;` to `true`.
3. Re-upload.
