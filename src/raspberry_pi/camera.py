"""Unified Camera Capture Module.

Supports:
- Pi Camera V2 via libcamera / Picamera2 (Raspberry Pi CSI port)
- USB Webcam via OpenCV VideoCapture (for desktop/bench testing)
- Synthetic / Mock Camera (for offline development and unit tests)
"""

from __future__ import annotations
import time
import numpy as np


class CameraNotConfigured(RuntimeError):
    pass


class CameraInterface:
    """Base interface for all camera backends."""

    def capture_array(self) -> np.ndarray:
        raise NotImplementedError

    def stop(self):
        pass


class Picamera2Backend(CameraInterface):
    """Raspberry Pi Camera V2 using the official Picamera2 library."""

    def __init__(self, width: int = 640, height: int = 480):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise CameraNotConfigured("Picamera2 is not installed on this system.") from exc

        self.camera = Picamera2()
        config = self.camera.create_video_configuration(main={"size": (width, height), "format": "RGB888"})
        self.camera.configure(config)
        self.camera.start()
        time.sleep(1.0)  # Camera warmup

    def capture_array(self) -> np.ndarray:
        return self.camera.capture_array()

    def stop(self):
        try:
            self.camera.stop()
        except Exception:
            pass


class OpenCVWebcamBackend(CameraInterface):
    """Standard USB webcam or laptop camera using OpenCV."""

    def __init__(self, device_index: int = 0, width: int = 640, height: int = 480):
        try:
            import cv2
        except ImportError as exc:
            raise CameraNotConfigured("OpenCV (cv2) is required for USB webcam capture.") from exc

        self.cap = cv2.VideoCapture(device_index)
        if not self.cap.isOpened():
            raise CameraNotConfigured(f"Cannot open video capture device {device_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def capture_array(self) -> np.ndarray:
        import cv2
        ret, frame = self.cap.read()
        if not ret or frame is None:
            raise RuntimeError("Failed to capture frame from webcam")
        # Convert BGR to RGB
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def stop(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()


class MockCameraBackend(CameraInterface):
    """Mock camera generating synthetic RGB frames for offline testing."""

    def __init__(self, width: int = 320, height: int = 320):
        self.width = width
        self.height = height

    def capture_array(self) -> np.ndarray:
        # Return a blank RGB frame
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def stop(self):
        pass


def open_camera(mode: str = "auto", width: int = 640, height: int = 480) -> CameraInterface:
    """Open camera with automatic fallback: Picamera2 -> OpenCV -> Mock."""
    if mode == "picamera2":
        return Picamera2Backend(width=width, height=height)
    elif mode == "webcam":
        return OpenCVWebcamBackend(width=width, height=height)
    elif mode == "mock":
        return MockCameraBackend(width=width, height=height)

    # Auto mode: try picamera2 first, then webcam, then mock
    try:
        return Picamera2Backend(width=width, height=height)
    except Exception:
        pass

    try:
        return OpenCVWebcamBackend(width=width, height=height)
    except Exception:
        pass

    return MockCameraBackend(width=width, height=height)
