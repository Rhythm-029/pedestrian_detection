import time
from src.common.protocol import make_message, encode_message
from src.common.zones import classify_zone, ZoneConfig


def build_status_message(seq, distance_m, confidence, pedestrian, config=ZoneConfig()):
    zone = classify_zone(distance_m, confidence, pedestrian, config)
    msg = make_message(seq, int(time.monotonic() * 1000), zone, distance_m, confidence, pedestrian, True)
    return encode_message(msg)
