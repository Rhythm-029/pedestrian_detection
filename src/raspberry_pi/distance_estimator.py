def estimate_distance_m(*, calibrated_distance_m=None):
    """Return a validated calibrated estimate, or None when no estimate is available.

    A monocular camera does not directly measure metric distance. Do not invent
    distance from detector confidence; calibrate and validate the chosen method.
    """
    if calibrated_distance_m is None:
        return None
    if calibrated_distance_m < 0:
        raise ValueError("distance cannot be negative")
    return float(calibrated_distance_m)
