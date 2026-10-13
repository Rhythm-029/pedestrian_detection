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
