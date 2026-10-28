"""Monocular Distance Estimation Module.

Implements the pinhole camera geometry model:
    d = k / h_bbox

Where:
    - d is the estimated distance in meters.
    - h_bbox is the height of the detected pedestrian bounding box in pixels.
    - k is the optical calibration constant: k = f_pixels * H_actual.
"""

from __future__ import annotations

from typing import Sequence
import math


class DistanceEstimator:
    """Estimates real-world distance from pedestrian bounding-box geometry."""

    def __init__(self, k: float = 320.0, min_height_px: float = 10.0, max_distance_m: float = 10.0):
        """
        Args:
            k: Calibration factor (k = distance_m * height_pixels).
            min_height_px: Minimum height in pixels to avoid division by zero or extreme noise.
            max_distance_m: Maximum clamp limit for distance.
        """
        if k <= 0:
            raise ValueError("Calibration constant k must be positive")
        self.k = float(k)
        self.min_height_px = float(min_height_px)
        self.max_distance_m = float(max_distance_m)

    def estimate_from_height(self, height_px: float) -> float | None:
        """Estimate distance in meters given bounding box pixel height."""
        if height_px is None or not math.isfinite(height_px) or height_px < self.min_height_px:
            return None
        distance = self.k / float(height_px)
        return min(round(distance, 2), self.max_distance_m)

    def estimate_from_bbox(self, bbox_xyxy: Sequence[int | float]) -> float | None:
        """Estimate distance from a bounding box: (xmin, ymin, xmax, ymax)."""
        if bbox_xyxy is None or len(bbox_xyxy) != 4:
            return None
        ymin, ymax = bbox_xyxy[1], bbox_xyxy[3]
        height_px = abs(ymax - ymin)
        return self.estimate_from_height(height_px)

    @staticmethod
    def calculate_k(samples: list[tuple[float, float]]) -> tuple[float, float]:
        """Derive calibration constant k from bench calibration pairs: (actual_distance_m, bbox_height_px).

        Returns:
            (optimal_k, mean_absolute_error_m)
        """
        if not samples:
            raise ValueError("At least one calibration sample is required")

        # k = distance * height
        k_values = [dist * height for dist, height in samples if dist > 0 and height > 0]
        if not k_values:
            raise ValueError("All calibration samples must have positive distance and height")

        optimal_k = sum(k_values) / len(k_values)

        errors = [abs(dist - (optimal_k / height)) for dist, height in samples]
        mae = sum(errors) / len(errors)
        return round(optimal_k, 2), round(mae, 3)


def estimate_distance_m(*, calibrated_distance_m=None):
    """Backwards-compatible passthrough helper for pre-computed distances."""
    if calibrated_distance_m is None:
        return None
    if calibrated_distance_m < 0:
        raise ValueError("distance cannot be negative")
    return float(calibrated_distance_m)
