try:
    import pytest
except ImportError:
    pytest = None

from src.raspberry_pi.distance_estimator import DistanceEstimator, estimate_distance_m


def test_estimate_distance_m_passthrough():
    assert estimate_distance_m(calibrated_distance_m=2.5) == 2.5
    assert estimate_distance_m(calibrated_distance_m=None) is None
    if pytest is not None:
        with pytest.raises(ValueError):
            estimate_distance_m(calibrated_distance_m=-1.0)
    else:
        try:
            estimate_distance_m(calibrated_distance_m=-1.0)
            assert False, "Should have raised ValueError"
        except ValueError:
            pass



def test_distance_estimator_formula():
    # d = k / h. With k = 320, h = 160 px -> d = 2.0 m
    estimator = DistanceEstimator(k=320.0)
    assert estimator.estimate_from_height(160.0) == 2.0
    # h = 320 px -> d = 1.0 m
    assert estimator.estimate_from_height(320.0) == 1.0
    # h = 80 px -> d = 4.0 m
    assert estimator.estimate_from_height(80.0) == 4.0


def test_distance_estimator_from_bbox():
    estimator = DistanceEstimator(k=300.0)
    # BBox: (xmin, ymin, xmax, ymax) -> height = 300 - 100 = 200 -> d = 300 / 200 = 1.5 m
    bbox = (50, 100, 150, 300)
    assert estimator.estimate_from_bbox(bbox) == 1.5


def test_distance_estimator_tiny_height_ignored():
    estimator = DistanceEstimator(k=320.0, min_height_px=10.0)
    # Noise below min_height returns None
    assert estimator.estimate_from_height(5.0) is None


def test_distance_estimator_calculate_k():
    samples = [(1.0, 300.0), (2.0, 150.0), (3.0, 100.0)]
    k, mae = DistanceEstimator.calculate_k(samples)
    assert k == 300.0
    assert mae == 0.0
