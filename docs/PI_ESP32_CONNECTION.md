# Pi ↔ ESP32 Connection Guide
*UART JSON Protocol v1 — 115200 baud, 8N1, 3.3V logic*

## Physical Connection

| Raspberry Pi | GPIO header | ESP32 DevKit V1 |
|---|---|---|
| GPIO14 (TXD) | Pin 8 | GPIO3 (RX0) |
| GPIO15 (RXD) | Pin 10 | GPIO1 (TX0) |
| GND | Pin 6 | GND |

> [!IMPORTANT]
> TX→RX and RX←TX lines are **crossed**. Pi TX goes to ESP32 RX and vice versa.
> Both devices operate at **3.3V logic** — no level shifter needed between Pi and ESP32.
> Do NOT connect Pi 5V to any ESP32 pin.

## UART0 Sharing with USB

The ESP32 UART0 (GPIO1/3) is shared with the USB-to-Serial chip used for:
- Firmware uploading via Arduino IDE
- Serial Monitor output

**During normal Pi integration:** UART0 carries Pi messages AND ESP32 debug output simultaneously. The Pi only reads structured JSON lines starting at `{` — ESP32 `Serial.print` debug lines are plain text and are ignored by the Pi parser.

**During firmware upload:** Disconnect Pi UART connections or the data lines will conflict with the upload. Never upload firmware with the Pi UART connected.

## Protocol Summary

**Direction:** Pi → ESP32 only (Pi sends; ESP32 reads)

**Message format:**
```json
{"v":1,"seq":42,"timestamp_ms":123456,"zone":"CAUTION","distance_m":1.6,"confidence":0.91,"pedestrian":true,"heartbeat":true}
```

**Fields:**

| Field | Type | Description |
|---|---|---|
| `v` | int | Protocol version (always 1) |
| `seq` | int | Increasing sequence number |
| `timestamp_ms` | int | Monotonic time (ms) |
| `zone` | string | `FAR` / `CAUTION` / `DANGER` |
| `distance_m` | float or null | Estimated pedestrian distance |
| `confidence` | float [0,1] | Detector confidence |
| `pedestrian` | bool | Pedestrian detected |
| `heartbeat` | bool | Always true |

**Heartbeat rate:** 100 ms (10 Hz)  
**ESP32 timeout:** 300 ms — motors stop if no valid message received

## Testing the Connection

### Step 1 — Software only (no Pi needed)

```bash
python3 scripts/simulate_pi_messages.py --port /dev/cu.usbserial-XXXX --scenario demo
```

Observe ESP32 Serial Monitor for `[RX]` lines showing received messages.

### Step 2 — Pi sending live

```bash
python3 src/raspberry_pi/pi_controller.py --camera mock --port /dev/serial0 --fps 10 --iterations 50
```

### Step 3 — Verify heartbeat timeout

```bash
python3 scripts/simulate_pi_messages.py --scenario heartbeat_timeout --port /dev/cu.usbserial-XXXX
```

ESP32 must report `WATCHDOG: timeout after Xms → COMM_FAULT` within 300–500 ms of Pi going silent.
