# Wiring plan — verify before connecting power

## Power
1. Battery positive -> correctly rated fuse close to battery -> master switch -> motor supply input, sized for motor and driver current.
2. Raspberry Pi 4 requires a stable 5 V supply capable of up to 3 A; verify buck converter current and thermal capacity before using it.
3. During development, power ESP32 from USB. For deployment, use a regulated supply appropriate to the exact board.
4. Logic and driver grounds need a common reference; route motor current away from signal wiring to reduce noise.
5. Never feed motor power to Pi GPIO or 5 V pins. Never apply 5 V logic to Pi/ESP32 GPIO.
6. Set buck output with a multimeter and load disconnected before connecting electronics.

## Driver boards
Only use TB6612FNG wiring if the actual IC is confirmed as TB6612FNG. Typical signals are VM, VCC, GND, STBY, AIN1/AIN2/PWMA, BIN1/BIN2/PWMB and motor outputs. If the chip is L298N, stop and use its own diagram. Check motor stall current against driver ratings.

## HC-SR04
VCC -> 5 V; GND -> common ground; TRIG <- ESP32 GPIO; ECHO -> resistor divider/level shifter -> ESP32 GPIO. Never connect 5 V Echo directly.

## Pi to ESP32 UART
Pi TXD -> ESP32 RX; ESP32 TX -> Pi RX; GND -> GND. Use 3.3 V logic and verify the selected pins and Pi UART configuration. Do not connect Pi 5 V to ESP32 3V3.

## Camera
Camera V2 connects to Pi's CSI connector, not USB. Disconnect power before inserting/removing ribbon. Check connector latch and contact orientation.

## Encoders
The disc alone is not an encoder measurement system. Compatible optical sensors and mounts are needed. Encoder feedback is optional for the first stop demonstration but required for measured closed-loop wheel-speed PID.
