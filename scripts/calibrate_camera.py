"""Camera Distance Calibration Tool.

Derives the optical constant k for the pinhole model:
    d = k / h_bbox

Usage:
    python scripts/calibrate_camera.py
    python scripts/calibrate_camera.py --save
"""

import argparse
import json
from pathlib import Path
import sys

# Add project root to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.raspberry_pi.distance_estimator import DistanceEstimator

# Sample bench calibration dataset (distance in meters vs bounding box height in 320x320 pixels)
# For standing pedestrian (~1.7m) with Pi Camera V2 at 320x320 resolution
BENCH_CALIBRATION_SAMPLES = [
    (1.0, 310.0),  # 1.0 m -> ~310 px
    (1.5, 208.0),  # 1.5 m -> ~208 px
    (2.0, 155.0),  # 2.0 m -> ~155 px
    (2.5, 124.0),  # 2.5 m -> ~124 px
    (3.0, 103.0),  # 3.0 m -> ~103 px
    (4.0,  78.0),  # 4.0 m -> ~78 px
]


def calibrate(samples=None, save=False):
    samples = samples or BENCH_CALIBRATION_SAMPLES
    print("==================================================")
    print("      CAMERA MONOCULAR DISTANCE CALIBRATION       ")
    print("               Formula: d = k / h                 ")
    print("==================================================")
    print(f"{'Actual Dist (m)':<18}{'BBox Height (px)':<18}{'Sample k':<12}")
    print("-" * 48)

    for dist, height in samples:
        sample_k = dist * height
        print(f"{dist:<18.2f}{height:<18.1f}{sample_k:<12.1f}")

    optimal_k, mae = DistanceEstimator.calculate_k(samples)

    print("-" * 48)
    print(f"Optimal constant k : {optimal_k:.2f}")
    print(f"Mean Abs Error (m) : {mae:.3f} m")
    print("==================================================")

    # Verification table
    print("\nModel Verification:")
    print(f"{'Actual (m)':<12}{'Measured (px)':<15}{'Estimated (m)':<15}{'Error (m)':<12}")
    estimator = DistanceEstimator(k=optimal_k)
    for dist, height in samples:
        est = estimator.estimate_from_height(height)
        err = est - dist
        print(f"{dist:<12.2f}{height:<15.1f}{est:<15.2f}{err:<+12.2f}")

    if save:
        calib_file = Path(__file__).resolve().parent.parent / "data" / "calibration" / "camera_calibration.json"
        calib_file.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "model": "pinhole_bbox_height",
            "equation": "d = k / h_bbox",
            "k": optimal_k,
            "resolution": "320x320",
            "mae_m": mae,
            "samples": [{"distance_m": d, "height_px": h} for d, h in samples]
        }
        with open(calib_file, "w") as f:
            json.dump(data, f, indent=2)
        print(f"\n[OK] Calibration saved to: {calib_file}")


def main():
    parser = argparse.ArgumentParser(description="Calibrate camera distance model k")
    parser.add_argument("--save", action="store_true", help="Save calculated k to data/calibration/camera_calibration.json")
    args = parser.parse_args()
    calibrate(save=args.save)


if __name__ == "__main__":
    main()
