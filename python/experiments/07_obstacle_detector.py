"""
07_obstacle_detector.py
=======================

Purpose
-------
Watch a configurable angular sector in front of the sensor and print a
warning whenever any sample inside it falls below a configured
threshold distance. This is the simplest useful spatial behavior: the
building block of reactive obstacle avoidance.

Run
---
    python experiments/07_obstacle_detector.py
    python experiments/07_obstacle_detector.py --center 0 --width 60 --threshold-mm 500
"""

from __future__ import annotations

import argparse
import glob
import platform
import signal
import sys
from pathlib import Path
from typing import Optional

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


def in_sector(angle_deg: float, center_deg: float, width_deg: float) -> bool:
    half = width_deg / 2.0
    return abs((angle_deg - center_deg + 180.0) % 360.0 - 180.0) <= half


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reactive obstacle detector.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--center", type=float, default=0.0)
    parser.add_argument("--width", type=float, default=60.0)
    parser.add_argument("--threshold-mm", type=float, default=500.0)
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

            print(f"Watching sector center={args.center} deg "
                  f"width={args.width} deg threshold={args.threshold_mm:.0f} mm.")

            closest_dist: float = float("inf")
            closest_angle: float = 0.0
            revolution_idx = 0

            for sample in lidar.iter_scans():
                if interrupted["flag"]:
                    break

                if sample.start_flag and revolution_idx >= 0:
                    if closest_dist == float("inf"):
                        print(f"rev {revolution_idx:3d}: no valid samples in sector.")
                    else:
                        status = "ALERT" if closest_dist < args.threshold_mm else "clear"
                        print(f"rev {revolution_idx:3d}: {status}  "
                              f"(closest in sector: {closest_dist:5.0f} mm "
                              f"at angle {closest_angle:6.1f})")
                    revolution_idx += 1
                    closest_dist = float("inf")
                    closest_angle = 0.0

                if sample.distance <= 0.0:
                    continue
                if in_sector(sample.angle, args.center, args.width):
                    if sample.distance < closest_dist:
                        closest_dist = sample.distance
                        closest_angle = sample.angle
    except Exception as exc:
        print(f"Scan failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
