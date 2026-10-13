from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    pedestrian: bool
    confidence: float
    bbox_xyxy: tuple[int, int, int, int] | None = None


class DetectorNotConfigured(RuntimeError):
    pass


class PedestrianDetector:
    """Model adapter seam. Select and validate an inference backend before use."""
    def __init__(self, model_path=None, min_confidence=0.5):
        self.model_path = model_path
        self.min_confidence = min_confidence

    def detect(self, frame):
        raise DetectorNotConfigured(
            "No inference backend configured. Select a model/runtime and implement it "
            "before enabling vehicle motion."
        )
