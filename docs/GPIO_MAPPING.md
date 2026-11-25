# GPIO Mapping — Pedestrian Detection Vehicle
*Authoritative reference — matches firmware and config/pins.yaml*

> [!IMPORTANT]
> This document is the single source of truth. If the firmware, YAML, and this document disagree, this document takes precedence. Update all three together.

---

## ESP32 DevKit V1 — Confirmed Pin Assignments

### Motor Driver (TB6612FNG)

| TB6612FNG Signal | ESP32 GPIO | Direction | Notes |
|---|---|---|---|
| PWMA | **GPIO25** | OUT | Channel A PWM speed (LEDC ch 0) |
| AIN1 | **GPIO26** | OUT | Channel A direction 1 |
| AIN2 | **GPIO27** | OUT | Channel A direction 2 |
| PWMB | **GPIO33** | OUT | Channel B PWM speed (LEDC ch 1) |
| BIN1 | **GPIO32** | OUT | Channel B direction 1 |
| BIN2 | **GPIO19** | OUT | Channel B direction 2 |
| VCC | 3V3 | PWR | Logic supply |
| GND | GND | PWR | Common ground |
| VM | Motor battery (+) | PWR | Motor power — separate fused supply |
| STBY | 3V3 rail (hardware tie) | — | **NOT connected to any GPIO** |

> [!WARNING]
> **STBY is permanently tied to 3V3.** Software cannot disable the motor driver via STBY.
> Motors are stopped by commanding zero PWM and coasting direction inputs.
> A physical motor-power switch on the VM line is mandatory for safe bench testing.

**Motor topology — one TB6612FNG, 4 motors paired:**

| Channel | Motors | Physical Side |
|---|---|---|
| A (AIN1/AIN2/PWMA) | Left-front + Left-rear (wired in parallel) | Left |
| B (BIN1/BIN2/PWMB) | Right-front + Right-rear (wired in parallel) | Right |

---

### Indicators

| Device | ESP32 GPIO | Direction | Notes |
|---|---|---|---|
| Buzzer (IO pin) | **GPIO22** | OUT | Active 3-pin module; polarity set by `BUZZER_ACTIVE_HIGH` constant |
| Status LED | **GPIO2** | OUT | ⚠️ REQUIRES HARDWARE CONFIRMATION — assumed built-in LED |

> [!NOTE]
> The buzzer module polarity must be confirmed from the module marking.
> The firmware constant `BUZZER_ACTIVE_HIGH` controls polarity.
> Set `true` if the buzzer sounds when the IO pin is HIGH; `false` if it sounds when LOW.

---

### Ultrasonic Sensor (HC-SR04)

| HC-SR04 Signal | ESP32 GPIO | Direction | Notes |
|---|---|---|---|
| TRIG | **GPIO5** | OUT | Trigger pulse — 5V-tolerant output |
| ECHO | **GPIO18** | IN | ⚠️ REQUIRES HARDWARE CONFIRMATION — MUST use voltage divider (5V→3.3V) |

> [!CAUTION]
> HC-SR04 ECHO outputs 5V logic. Connecting directly to an ESP32 GPIO (3.3V max) will permanently damage the ESP32. A voltage divider (10kΩ + 20kΩ) or dedicated level shifter MUST be in place before powering on.

> [!WARNING]
> GPIO5 (TRIG) and GPIO18 (ECHO) match the original firmware and are used here pending physical confirmation. If the actual wiring differs, update `config/pins.yaml` and `hw_config.h` before uploading firmware.

---

### UART Communication (Pi ↔ ESP32)

| Signal | ESP32 GPIO | Raspberry Pi Header |
|---|---|---|
| RX (ESP32 receives) | GPIO3 (RX0) | GPIO14 (TXD, pin 8) |
| TX (ESP32 sends) | GPIO1 (TX0) | GPIO15 (RXD, pin 10) |
| GND | GND | GND (pin 6) |

> [!IMPORTANT]
> Both devices operate at 3.3V logic — no level shifter required between Pi GPIO and ESP32.
> TX/RX lines are **crossed**: Pi TX → ESP32 RX; ESP32 TX → Pi RX.
> Using UART0 (GPIO1/3) means Serial Monitor output during normal operation shares the same lines as Pi communication. Use USB flashing; disconnect Pi UART during firmware upload.

---

## Pin Status Summary

| GPIO | Signal | Status |
|---|---|---|
| 1 | UART TX0 | ✅ Confirmed |
| 2 | LED | ⚠️ Assumed (requires confirmation) |
| 3 | UART RX0 | ✅ Confirmed |
| 5 | HC-SR04 TRIG | ⚠️ Assumed (requires confirmation) |
| 18 | HC-SR04 ECHO | ⚠️ Assumed (requires confirmation) |
| 19 | BIN2 (Motor B dir 2) | ✅ Confirmed |
| 22 | Buzzer IO | ✅ Confirmed |
| 25 | PWMA (Motor A PWM) | ✅ Confirmed |
| 26 | AIN1 (Motor A dir 1) | ✅ Confirmed |
| 27 | AIN2 (Motor A dir 2) | ✅ Confirmed |
| 32 | BIN1 (Motor B dir 1) | ✅ Confirmed |
| 33 | PWMB (Motor B PWM) | ✅ Confirmed |

---

## Boot-Strapping Pin Caution

The following ESP32 pins affect boot mode and must NOT be driven LOW at power-on:

| GPIO | Boot behaviour | Used in this project |
|---|---|---|
| 0 | LOW = download mode | Not used |
| 2 | Must float or HIGH at boot | ⚠️ LED — ensure LED circuit does not force LOW |
| 15 | LOW = silent boot | Not used |

GPIO2 is used for the LED. Verify that the LED circuit (LED + resistor to GND) does not hold GPIO2 LOW during power-on. If it does, the ESP32 will fail to boot normally. Use an external LED on a different pin if boot problems occur.

---

## Pins Explicitly NOT Used

| GPIO | Reason |
|---|---|
| 6–11 | Connected to internal SPI Flash — never use |
| 16, 17 | Not broken out on standard DevKit V1 |
| 34, 35, 36, 39 | Input-only — not suitable for motor/indicator outputs |

---

## Confirmation Checklist

Before enabling `MOTOR_OUTPUTS_ENABLED = true`:

- [ ] Measure voltage on HC-SR04 ECHO line before connecting to GPIO18 (must be ≤ 3.3V at ESP32 pin)
- [ ] Confirm LED pin (GPIO2) with multimeter — ensure no boot conflict
- [ ] Confirm TRIG/ECHO physical wiring matches GPIO5/GPIO18
- [ ] Confirm buzzer polarity with a bench test using `test_buzzer.ino`
- [ ] Verify motor driver channel A and B assignments match left/right orientation
- [ ] Verify common ground between ESP32, motor driver, ultrasonic sensor, and buzzer
- [ ] Physical motor-power cutoff switch installed before first motor test
