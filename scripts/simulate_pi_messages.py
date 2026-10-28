"""Hardware-in-the-loop and software test simulator for ESP32.

Can transmit simulated Raspberry Pi telemetry over a serial COM port or print to stdout.
Used to test ESP32 FSM states, motor speeds, indicator beeps, heartbeat timeouts,
and 2-second recovery clear logic without needing the actual Raspberry Pi or Camera.
"""

import argparse
import sys
import time
from pathlib import Path

# Add project root to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

try:
    import serial
except ImportError:
    serial = None

from src.common.protocol import encode_message, make_message



def run_scenario(port=None, baud=115200, scenario="demo"):
    ser = None
    if port:
        if serial is None:
            print("Error: pyserial is required for serial communication. Install with: pip install pyserial")
            sys.exit(1)
        ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.0)  # Wait for ESP32 bootloader reset
        print(f"Connected to ESP32 on {port} at {baud} baud.")

    seq = 0

    def send(zone, distance, confidence, pedestrian, label=""):
        nonlocal seq
        seq += 1
        msg = make_message(
            seq=seq,
            timestamp_ms=int(time.monotonic() * 1000),
            zone=zone,
            distance_m=distance,
            confidence=confidence,
            pedestrian=pedestrian,
            heartbeat=True
        )
        data = encode_message(msg)
        if ser:
            ser.write(data)
            ser.flush()
        out_str = data.decode().rstrip()
        prefix = f"[{label}] " if label else ""
        print(f"{prefix}TX -> {out_str}")

    print(f"\n--- Running Scenario: {scenario.upper()} ---")

    if scenario == "demo":
        # 1. Cruise for 3 seconds (FAR)
        print("\n>> 1. Cruising (FAR zone, normal speed)...")
        for _ in range(30):
            send("FAR", 3.2, 0.95, False, "CRUISE")
            time.sleep(0.1)

        # 2. Pedestrian in Caution zone for 2 seconds (CAUTION)
        print("\n>> 2. Approaching Pedestrian (CAUTION zone, 40% speed, blinking alert)...")
        for _ in range(20):
            send("CAUTION", 1.5, 0.91, True, "CAUTION")
            time.sleep(0.1)

        # 3. Sudden Pedestrian in Danger zone for 2 seconds (DANGER)
        print("\n>> 3. Emergency Stop (DANGER zone, latch stop, continuous buzzer)...")
        for _ in range(20):
            send("DANGER", 0.6, 0.98, True, "DANGER")
            time.sleep(0.1)

        # 4. Path clears, observe 2-second recovery requirement
        print("\n>> 4. Path Clear (Testing 2.0s recovery latch requirement)...")
        for i in range(40):
            send("FAR", 3.5, 0.99, False, f"RECOVERY ({i*100}ms)")
            time.sleep(0.1)

        print("\n>> Demo sequence completed.")

    elif scenario == "heartbeat_timeout":
        print("\n>> Sending FAR messages for 1 second...")
        for _ in range(10):
            send("FAR", 3.0, 0.95, False, "CRUISE")
            time.sleep(0.1)

        print("\n>> Simulating sudden Pi freeze/communication loss (silence for 2 seconds)...")
        print(">> ESP32 should trip COMMUNICATION_FAULT within 500 ms and stop all motors!")
        time.sleep(2.0)

    elif scenario == "continuous_cruise":
        print("\n>> Streaming continuous FAR telemetry at 10 Hz (Ctrl+C to stop)...")
        try:
            while True:
                send("FAR", 3.0, 0.95, False, "CRUISE")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopped.")


def main():
    parser = argparse.ArgumentParser(description="Simulate Pi telemetry to ESP32")
    parser.add_argument("--port", "-p", default=None, help="Serial port (e.g., /dev/ttyUSB0 or COM3). Omit for stdout.")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument(
        "--scenario", "-s",
        choices=["demo", "heartbeat_timeout", "continuous_cruise"],
        default="demo",
        help="Test scenario to execute"
    )
    args = parser.parse_args()
    run_scenario(port=args.port, baud=args.baud, scenario=args.scenario)


if __name__ == "__main__":
    main()
