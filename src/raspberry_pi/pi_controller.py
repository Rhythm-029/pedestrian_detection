"""Raspberry Pi Safety Controller & Real-Time Pipeline Orchestrator.

Full Perception-to-Action Loop:
1. Capture frame from camera (Pi Camera V2 / Webcam / Mock)
2. Run pedestrian detection (YOLO / ONNX Runtime)
3. Estimate monocular distance via d = k / h_bbox
4. Apply 3-frame temporal debouncing
5. Encode telemetry into Protocol v1 JSON
6. Transmit to ESP32 via 115200 baud UART at 10-15 Hz
"""

from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path
from typing import Sequence

# Add workspace root to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from src.common.protocol import make_message, encode_message
from src.common.zones import classify_zone, ZoneConfig, ZoneDebouncer
from src.raspberry_pi.camera import open_camera
from src.raspberry_pi.detector import PedestrianDetector
from src.raspberry_pi.distance_estimator import DistanceEstimator

try:
    import serial
except ImportError:
    serial = None


def build_status_message(seq: int, distance_m: float | None, confidence: float, pedestrian: bool, config: ZoneConfig = ZoneConfig()) -> bytes:
    """Helper to build and encode a single status message (backwards-compatible)."""
    zone = classify_zone(distance_m, confidence, pedestrian, config)
    msg = make_message(seq, int(time.monotonic() * 1000), zone, distance_m, confidence, pedestrian, True)
    return encode_message(msg)


class PiSafetyController:
    """Coordinates camera capture, detection, distance estimation, debouncing, and UART output."""

    def __init__(
        self,
        serial_port: str | None = None,
        baud_rate: int = 115200,
        model_path: str | None = None,
        camera_mode: str = "auto",
        k: float = 310.5,
        target_fps: float = 10.0,
        min_confidence: float = 0.50
    ):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.target_fps = target_fps
        self.frame_interval = 1.0 / target_fps

        # Subsystems
        self.camera = open_camera(mode=camera_mode)
        self.detector = PedestrianDetector(model_path=model_path, min_confidence=min_confidence)
        self.distance_estimator = DistanceEstimator(k=k)
        self.zone_config = ZoneConfig(danger_distance_m=1.0, caution_distance_m=2.0, minimum_confidence=min_confidence)
        self.debouncer = ZoneDebouncer(window_size=3)

        # Serial transport
        self.ser = None
        if serial_port:
            if serial is None:
                raise RuntimeError("pyserial is not installed. Install with: pip install pyserial")
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            print(f"[Controller] Opened UART on {serial_port} at {baud_rate} baud.")

        self.seq = 0
        self.running = False

    def step(self) -> dict:
        """Execute one iteration of the perception and transmission pipeline."""
        self.seq += 1
        t_start = time.monotonic()

        # 1. Grab camera frame
        frame = self.camera.capture_array()

        # 2. Run detector
        detection = self.detector.detect(frame)

        # 3. Estimate distance from bounding box height: d = k / h
        distance_m = None
        if detection.pedestrian and detection.bbox_xyxy:
            distance_m = self.distance_estimator.estimate_from_bbox(detection.bbox_xyxy)

        # 4. Instantaneous zone classification
        instant_zone = classify_zone(
            distance_m=distance_m,
            confidence=detection.confidence,
            pedestrian=detection.pedestrian,
            config=self.zone_config
        )

        # 5. Apply 3-frame debounce filter (Slide 7)
        debounced_zone = self.debouncer.update(instant_zone)

        # 6. Build Protocol v1 JSON packet
        msg = make_message(
            seq=self.seq,
            timestamp_ms=int(time.monotonic() * 1000),
            zone=debounced_zone,
            distance_m=distance_m,
            confidence=round(detection.confidence, 2),
            pedestrian=detection.pedestrian,
            heartbeat=True
        )
        packet = encode_message(msg)

        # 7. Transmit to ESP32
        if self.ser and self.ser.is_open:
            self.ser.write(packet)
            self.ser.flush()

        elapsed = time.monotonic() - t_start
        return {
            "seq": self.seq,
            "instant_zone": instant_zone,
            "debounced_zone": debounced_zone,
            "distance_m": distance_m,
            "confidence": detection.confidence,
            "pedestrian": detection.pedestrian,
            "latency_ms": round(elapsed * 1000, 1),
            "packet": packet.decode().rstrip()
        }

    def run(self, max_iterations: int | None = None):
        """Run the continuous control loop."""
        self.running = True
        print(f"[Controller] Starting main safety loop at {self.target_fps} Hz...")

        iteration = 0
        try:
            while self.running:
                loop_start = time.monotonic()
                telemetry = self.step()
                iteration += 1

                # Console log
                zone_str = f"[{telemetry['debounced_zone']}]"
                dist_str = f"{telemetry['distance_m']:.2f}m" if telemetry['distance_m'] is not None else "None"
                print(f"#{telemetry['seq']:04d} | {zone_str:<10} | Dist: {dist_str:<7} | Conf: {telemetry['confidence']:.2f} | Latency: {telemetry['latency_ms']}ms")

                if max_iterations and iteration >= max_iterations:
                    break

                # Sleep to maintain target rate
                elapsed = time.monotonic() - loop_start
                sleep_time = max(0.0, self.frame_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[Controller] Stopping...")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.camera.stop()
        print("[Controller] Shutdown complete.")


def main():
    parser = argparse.ArgumentParser(description="Raspberry Pi Safety Controller")
    parser.add_argument("--port", "-p", default=None, help="Serial UART port to ESP32 (e.g. /dev/serial0). Omit for console.")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--model", "-m", default=None, help="Path to YOLO ONNX/PT model")
    parser.add_argument("--fps", type=float, default=10.0, help="Target loop frequency (Hz)")
    parser.add_argument("--camera", choices=["auto", "picamera2", "webcam", "mock"], default="auto")
    parser.add_argument("--iterations", "-n", type=int, default=None, help="Run N steps then exit")
    args = parser.parse_args()

    controller = PiSafetyController(
        serial_port=args.port,
        baud_rate=args.baud,
        model_path=args.model,
        camera_mode=args.camera,
        target_fps=args.fps
    )
    controller.run(max_iterations=args.iterations)


if __name__ == "__main__":
    main()
