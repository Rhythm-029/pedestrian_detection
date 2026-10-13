import pytest
from src.common.zones import classify_zone, ZoneConfig


def test_far():
    assert classify_zone(3.0, 0.9, True) == "FAR"


def test_caution():
    assert classify_zone(1.5, 0.9, True) == "CAUTION"


def test_danger():
    assert classify_zone(0.8, 0.9, True) == "DANGER"


def test_no_pedestrian_is_far():
    assert classify_zone(0.5, 0.99, False) == "FAR"


def test_low_confidence_is_far():
    assert classify_zone(0.5, 0.2, True) == "FAR"


def test_unknown_distance_is_caution():
    assert classify_zone(None, 0.9, True) == "CAUTION"


def test_invalid_threshold_order():
    with pytest.raises(ValueError):
        ZoneConfig(danger_distance_m=2.0, caution_distance_m=1.0)
