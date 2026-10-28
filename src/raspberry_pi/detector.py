"""Pedestrian Detection Module for Raspberry Pi.

Supports:
- ONNX Runtime backend (recommended for Pi 4 CPU deployment)
- Ultralytics YOLO backend (PyTorch)
- Mock backend (for testing without GPU/model files)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Sequence
import numpy as np


@dataclass(frozen=True)
class Detection:
    pedestrian: bool
    confidence: float
    bbox_xyxy: tuple[int, int, int, int] | None = None
    all_bboxes: list[tuple[int, int, int, int]] | None = None


class DetectorNotConfigured(RuntimeError):
    pass


class PedestrianDetector:
    """YOLO model adapter supporting ONNX Runtime, Ultralytics, and Mock backends."""

    def __init__(
        self,
        model_path: str | None = None,
        min_confidence: float = 0.50,
        backend: str = "auto",
        input_size: tuple[int, int] = (320, 320)
    ):
        self.model_path = model_path
        self.min_confidence = float(min_confidence)
        self.input_size = input_size
        self.backend = backend
        self._session = None
        self._ultralytics_model = None

        if backend == "auto":
            if model_path is None:
                self.backend = "mock"
            elif model_path.endswith(".onnx"):
                self.backend = "onnx"
            elif model_path.endswith(".pt"):
                self.backend = "ultralytics"
            else:
                self.backend = "mock"

        self._init_backend()

    def _init_backend(self):
        if self.backend == "onnx":
            try:
                import onnxruntime as ort
                # Use CPU provider for Raspberry Pi
                self._session = ort.InferenceSession(
                    self.model_path,
                    providers=["CPUExecutionProvider"]
                )
            except Exception as exc:
                raise DetectorNotConfigured(f"Failed to load ONNX model '{self.model_path}': {exc}") from exc

        elif self.backend == "ultralytics":
            try:
                from ultralytics import YOLO
                self._ultralytics_model = YOLO(self.model_path)
            except Exception as exc:
                raise DetectorNotConfigured(f"Failed to load YOLO model '{self.model_path}': {exc}") from exc

        elif self.backend == "mock":
            # Mock backend for offline tests and validation
            pass

    def detect(self, frame: np.ndarray | None) -> Detection:
        """Run pedestrian detection on an input RGB image frame.

        Returns Detection with the closest pedestrian (largest bounding box height).
        """
        if self.backend == "mock":
            # Return no pedestrian by default in mock unless configured
            return Detection(pedestrian=False, confidence=0.0, bbox_xyxy=None, all_bboxes=[])

        if frame is None:
            return Detection(pedestrian=False, confidence=0.0, bbox_xyxy=None, all_bboxes=[])

        if self.backend == "onnx":
            return self._detect_onnx(frame)
        elif self.backend == "ultralytics":
            return self._detect_ultralytics(frame)

        raise DetectorNotConfigured(f"Unsupported backend: {self.backend}")

    def _detect_ultralytics(self, frame: np.ndarray) -> Detection:
        """Run Ultralytics YOLO inference."""
        results = self._ultralytics_model(
            frame,
            imgsz=self.input_size[0],
            conf=self.min_confidence,
            classes=[0],  # Class 0 is 'person' in COCO
            verbose=False
        )

        boxes = []
        max_height = 0
        best_box = None
        best_conf = 0.0

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf >= self.min_confidence:
                    xyxy = tuple(map(int, box.xyxy[0].tolist()))
                    boxes.append(xyxy)
                    height = abs(xyxy[3] - xyxy[1])
                    # Closest pedestrian has largest pixel height
                    if height > max_height:
                        max_height = height
                        best_box = xyxy
                        best_conf = conf

        has_pedestrian = best_box is not None
        return Detection(
            pedestrian=has_pedestrian,
            confidence=best_conf if has_pedestrian else 0.0,
            bbox_xyxy=best_box,
            all_bboxes=boxes
        )

    def _detect_onnx(self, frame: np.ndarray) -> Detection:
        """Run ONNX Runtime inference for standard YOLOv8n ONNX export."""
        # 1. Preprocess: resize to 320x320, normalize [0, 1], transpose to (1, 3, H, W)
        h, w = frame.shape[:2]
        target_w, target_h = self.input_size

        # Simple aspect resize or crop
        resized = self._resize_image(frame, target_w, target_h)
        input_tensor = resized.astype(np.float32) / 255.0
        input_tensor = np.transpose(input_tensor, (2, 0, 1))
        input_tensor = np.expand_dims(input_tensor, axis=0)

        # 2. Run inference
        input_name = self._session.get_inputs()[0].name
        outputs = self._session.run(None, {input_name: input_tensor})
        raw_output = outputs[0]  # Shape: (1, 84, N) for YOLOv8 (4 box coords + 80 classes)

        # 3. Postprocess YOLOv8 output
        predictions = np.squeeze(raw_output).T  # Shape: (N, 84)
        boxes = []
        confidences = []

        # YOLOv8 format: [cx, cy, w, h, class_0, class_1, ...]
        person_scores = predictions[:, 4]  # Class 0 is person
        mask = person_scores >= self.min_confidence

        filtered_preds = predictions[mask]
        filtered_scores = person_scores[mask]

        best_box = None
        max_height = 0
        best_conf = 0.0

        for pred, conf in zip(filtered_preds, filtered_scores):
            cx, cy, bw, bh = pred[:4]
            # Convert back to original frame scale
            scale_x = w / target_w
            scale_y = h / target_h

            xmin = int((cx - bw / 2) * scale_x)
            ymin = int((cy - bh / 2) * scale_y)
            xmax = int((cx + bw / 2) * scale_x)
            ymax = int((cy + bh / 2) * scale_y)

            box = (xmin, ymin, xmax, ymax)
            boxes.append(box)
            height = abs(ymax - ymin)

            if height > max_height:
                max_height = height
                best_box = box
                best_conf = float(conf)

        has_pedestrian = best_box is not None
        return Detection(
            pedestrian=has_pedestrian,
            confidence=best_conf if has_pedestrian else 0.0,
            bbox_xyxy=best_box,
            all_bboxes=boxes
        )

    @staticmethod
    def _resize_image(img: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
        """Bilinear resize image array without external cv2 dependency if needed."""
        try:
            import cv2
            return cv2.resize(img, (target_w, target_h))
        except ImportError:
            # Fallback pure-numpy nearest neighbor resize
            h, w = img.shape[:2]
            y_indices = (np.linspace(0, h - 1, target_h)).astype(int)
            x_indices = (np.linspace(0, w - 1, target_w)).astype(int)
            return img[y_indices[:, None], x_indices]
