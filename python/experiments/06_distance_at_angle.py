"""
06_distance_at_angle.py
=======================

Purpose
-------
Given an angle (in degrees, in the sensor body frame), report the live
distance to whatever obstacle is in that direction. Use an angular
tolerance so we are not at the mercy of which exact sample falls at
exactly that bearing, and smooth across recent revolutions so the
displayed number stops bouncing on noise.

Run
---
    python experiments/06_distance_at_angle.py --angle 0
    python experiments/06_distance_at_angle.py --angle 90 --tolerance 10
"""

from __future__ import annotations

import argparse
import collections
import glob
import platform
import signal
import sys
from pathlib import Path
from typing import List, Optional

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


def angular_distance_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def median(values: List[float]) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 == 1 else (s[mid - 1] + s[mid]) / 2.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live distance at a chosen bearing.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--angle", type=float, default=0.0,
                        help="Target bearing in degrees, body frame.")
    parser.add_argument("--tolerance", type=float, default=5.0,
                        help="+/- degrees around the target angle.")
    parser.add_argument("--smoothing", type=int, default=5,
                        help="Number of revolutions to median-smooth across.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    interrupted = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: interrupted.__setitem__("flag", True))

    try:
        with RPLidarC1(port=port, baudrate=args.baudrate, timeout=2.0) as lidar:
            lidar.set_motor_pwm(args.motor_pwm)

            print(f"Watching angle {args.angle} deg, +/- {args.tolerance} deg. "
                  f"Press Ctrl+C to stop.")

            sector: List[float] = []
            recent_medians: collections.deque[float] = collections.deque(
                maxlen=max(1, args.smoothing))
            revolution_idx = 0

            for sample in lidar.iter_scans():
                if interrupted["flag"]:
                    break

                if sample.start_flag and sector:
                    rev_median = median(sector)
                    recent_medians.append(rev_median)
                    smoothed = median(list(recent_medians))
                    revolution_idx += 1
                    print(f"  {revolution_idx:3d}: {smoothed:6.0f} mm  "
                          f"(samples in sector: {len(sector)})")
                    sector = []

                if sample.distance <= 0.0:
                    continue
                if angular_distance_deg(sample.angle, args.angle) <= args.tolerance:
                    sector.append(sample.distance)
    except Exception as exc:
        print(f"Scan failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
