from __future__ import annotations
import json
import math
from typing import Any

PROTOCOL_VERSION = 1
VALID_ZONES = {"FAR", "CAUTION", "DANGER"}


class ProtocolError(ValueError):
    """Raised when a protocol message is malformed or unsafe."""


def validate_message(message: dict[str, Any]) -> None:
    required = {"v", "seq", "timestamp_ms", "zone", "distance_m", "confidence", "pedestrian", "heartbeat"}
    if not isinstance(message, dict):
        raise ProtocolError("message must be an object")
    missing = required - message.keys()
    if missing:
        raise ProtocolError(f"missing fields: {sorted(missing)}")
    if message["v"] != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    for field in ("seq", "timestamp_ms"):
        value = message[field]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ProtocolError(f"{field} must be a non-negative integer")
    if message["zone"] not in VALID_ZONES:
        raise ProtocolError("invalid zone")
    distance = message["distance_m"]
    if distance is not None and (isinstance(distance, bool) or not isinstance(distance, (int, float)) or not math.isfinite(float(distance)) or distance < 0):
        raise ProtocolError("distance_m must be null or finite and non-negative")
    confidence = message["confidence"]
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not math.isfinite(float(confidence)) or not 0 <= confidence <= 1:
        raise ProtocolError("confidence must be finite and between 0 and 1")
    if not isinstance(message["pedestrian"], bool) or not isinstance(message["heartbeat"], bool):
        raise ProtocolError("pedestrian and heartbeat must be booleans")


def encode_message(message: dict[str, Any]) -> bytes:
    validate_message(message)
    return (json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")


def decode_message(line: bytes | str) -> dict[str, Any]:
    try:
        text = line.decode("utf-8") if isinstance(line, bytes) else line
        obj = json.loads(text)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("invalid JSON/UTF-8") from exc
    validate_message(obj)
    return obj


def make_message(seq: int, timestamp_ms: int, zone: str, distance_m: float | None,
                 confidence: float, pedestrian: bool, heartbeat: bool = True) -> dict[str, Any]:
    message = {"v": PROTOCOL_VERSION, "seq": seq, "timestamp_ms": timestamp_ms,
               "zone": zone, "distance_m": distance_m, "confidence": confidence,
               "pedestrian": pedestrian, "heartbeat": heartbeat}
    validate_message(message)
    return message
