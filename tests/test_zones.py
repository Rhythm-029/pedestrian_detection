try:
    import pytest
except ImportError:
    pytest = None

from src.common.zones import classify_zone, ZoneConfig, ZoneDebouncer


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
    if pytest is not None:
        with pytest.raises(ValueError):
            ZoneConfig(danger_distance_m=2.0, caution_distance_m=1.0)
    else:
        try:
            ZoneConfig(danger_distance_m=2.0, caution_distance_m=1.0)
            assert False, "Should have raised ValueError"
        except ValueError:
            pass



def test_debouncer_single_glitch_ignored():
    debouncer = ZoneDebouncer(window_size=3)
    # Starts in FAR
    assert debouncer.current_zone == "FAR"
    # Single frame glitch to DANGER does not immediately latch if previous were FAR
    # With history: [DANGER], count=1 < 2 -> remains FAR
    assert debouncer.update("DANGER") == "FAR"
    # Next frame back to FAR
    assert debouncer.update("FAR") == "FAR"


def test_debouncer_escalates_on_two_danger_frames():
    debouncer = ZoneDebouncer(window_size=3)
    debouncer.update("DANGER")
    # Second DANGER frame triggers DANGER
    assert debouncer.update("DANGER") == "DANGER"


def test_debouncer_requires_all_frames_to_recover():
    debouncer = ZoneDebouncer(window_size=3)
    debouncer.update("DANGER")
    debouncer.update("DANGER")
    assert debouncer.current_zone == "DANGER"

    # One FAR frame is not enough to de-escalate
    debouncer.update("FAR")
    assert debouncer.current_zone == "DANGER"

    # Second FAR frame is still not enough
    debouncer.update("FAR")
    assert debouncer.current_zone == "DANGER"

    # Third consecutive FAR frame satisfies 3-frame debounce
    assert debouncer.update("FAR") == "FAR"
