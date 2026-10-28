from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ZoneConfig:
    danger_distance_m: float = 1.0
    caution_distance_m: float = 2.0
    minimum_confidence: float = 0.5

    def __post_init__(self):
        if not (0 < self.danger_distance_m < self.caution_distance_m):
            raise ValueError("require 0 < danger threshold < caution threshold")
        if not 0 <= self.minimum_confidence <= 1:
            raise ValueError("minimum_confidence must be in [0, 1]")


def classify_zone(distance_m, confidence, pedestrian, config=ZoneConfig()):
    if not pedestrian or confidence < config.minimum_confidence:
        return "FAR"
    if distance_m is None or not math.isfinite(distance_m) or distance_m < 0:
        return "CAUTION"
    if distance_m < config.danger_distance_m:
        return "DANGER"
    if distance_m < config.caution_distance_m:
        return "CAUTION"
    return "FAR"


class ZoneDebouncer:
    """Sliding-window temporal filter (3-frame debounce) as specified in Slide 7.

    Prevents instantaneous detection noise or camera drops from causing erratic
    zone transitions.
    """
    def __init__(self, window_size: int = 3):
        if window_size < 1:
            raise ValueError("window_size must be >= 1")
        self.window_size = window_size
        self._history: deque[str] = deque(maxlen=window_size)
        self._current_zone = "FAR"

    @property
    def current_zone(self) -> str:
        return self._current_zone

    def update(self, instantaneous_zone: str) -> str:
        if instantaneous_zone not in {"FAR", "CAUTION", "DANGER"}:
            raise ValueError(f"Unknown zone: {instantaneous_zone}")

        self._history.append(instantaneous_zone)

        # Count occurrences in window
        danger_count = self._history.count("DANGER")
        caution_count = self._history.count("CAUTION")
        far_count = self._history.count("FAR")

        # Conservative safety escalation:
        # If at least 2 out of 3 frames are DANGER -> DANGER
        # Or if 2 consecutive DANGER -> DANGER
        if danger_count >= 2:
            self._current_zone = "DANGER"
        elif (danger_count + caution_count) >= 2:
            self._current_zone = "CAUTION"
        elif far_count == self.window_size:
            # Strict de-escalation: requires all frames in window to be clear
            self._current_zone = "FAR"

        return self._current_zone

    def reset(self):
        self._history.clear()
        self._current_zone = "FAR"
