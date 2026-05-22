"""
04_polar_to_cartesian.py
========================

Purpose
-------
Take one full revolution from the sensor and convert each (angle,
distance) sample into a cartesian (x, y) point in the sensor's body
frame. Print the first few points so the conversion is concrete and
inspectable, and report some summary stats.

What this teaches
-----------------
- The math: x = d * cos(theta), y = d * sin(theta).
- Unit conventions: distance is in millimetres, angle is in degrees as
  reported by the sensor, theta in the math is radians.
- The coordinate convention used in this repo: sensor at origin, 0 deg
  along +X, angles increase counter-clockwise as seen from above.
- Why "distance == 0" samples must be dropped: they mean "no return",
  not "the world starts at the sensor".

Run
---
    python experiments/04_polar_to_cartesian.py
    python experiments/04_polar_to_cartesian.py --max-points 12

Expected output
---------------
    Captured one revolution: 482 samples (412 valid, 70 dropped).
    First 12 valid points in body frame (x_mm, y_mm):
       0:  angle=  0.84 deg  d= 2410.0 mm  ->  x= +2410.0  y=   +35.2
       ...
"""

from __future__ import annotations

import argparse
import glob
import math
import platform
import sys
from pathlib import Path
from typing import List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lib.rplidar_c1 import RPLidarC1, DEFAULT_MOTOR_PWM  # noqa: E402


DEFAULT_BAUDRATE = 460800


def auto_detect_port() -> Optional[str]:
    system = platform.system()
    if system == "Darwin":
        candidates = sorted(glob.glob("/dev/cu.usbserial-*"))
        candidates += sorted(glob.glob("/dev/cu.SLAB_USBtoUART"))
    elif system == "Linux":
        candidates = sorted(glob.glob("/dev/ttyUSB*"))
    else:
        return None
    return candidates[0] if candidates else None


def polar_to_cartesian(angle_deg: float, distance_mm: float) -> Tuple[float, float]:
    """Convert one polar sample to cartesian (x_mm, y_mm) in the body frame."""
    theta = math.radians(angle_deg)
    return distance_mm * math.cos(theta), distance_mm * math.sin(theta)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert one revolution from polar to cartesian.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--max-points", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    try:
        with RPLidarC1(port=port, baudrate=args.baudrate, timeout=2.0) as lidar:
            lidar.set_motor_pwm(args.motor_pwm)

            # Collect one full revolution: start at the first start_flag
            # and stop just before the next.
            raw: List[Tuple[int, float, float]] = []
            seen_start = False
            for sample in lidar.iter_scans():
                if sample.start_flag:
                    if seen_start:
                        break
                    seen_start = True
                if seen_start:
                    raw.append((sample.quality, sample.angle, sample.distance))

            if not raw:
                print("Captured zero samples in this revolution.", file=sys.stderr)
                return 1

            valid = [(q, a, d) for (q, a, d) in raw if d > 0.0]
            dropped = len(raw) - len(valid)
            print(f"Captured one revolution: {len(raw)} samples "
                  f"({len(valid)} valid, {dropped} dropped).")

            print(f"First {min(args.max_points, len(valid))} valid points in body "
                  f"frame (x_mm, y_mm):")
            for idx, (_, angle, dist) in enumerate(valid[: args.max_points]):
                x, y = polar_to_cartesian(angle, dist)
                print(f"  {idx:3d}:  angle={angle:6.2f} deg  d={dist:7.1f} mm  "
                      f"->  x={x:+9.1f}  y={y:+9.1f}")

            xs = [polar_to_cartesian(a, d)[0] for (_, a, d) in valid]
            ys = [polar_to_cartesian(a, d)[1] for (_, a, d) in valid]
            print("Bounding box of this scan in body frame:")
            print(f"  x_min={min(xs):+9.1f}   x_max={max(xs):+9.1f}")
            print(f"  y_min={min(ys):+9.1f}   y_max={max(ys):+9.1f}")
    except Exception as exc:
        print(f"Scan failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
