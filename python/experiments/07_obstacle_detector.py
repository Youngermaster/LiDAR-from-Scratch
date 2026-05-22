"""
07_obstacle_detector.py
=======================

Purpose
-------
Watch a configurable angular sector in front of the sensor and print a
warning whenever any sample inside it falls below a configured threshold
distance. This is the simplest useful spatial behavior: the building
block of reactive obstacle avoidance for a wheeled robot.

What this teaches
-----------------
- That "obstacle detection" with a 2D LiDAR is, at this level, a one-line
  predicate: any (angle, distance) inside the watched sector with
  distance < threshold.
- The importance of dropping zero-distance ("no return") samples before
  applying the predicate. Otherwise every revolution shows a false
  positive at 0 mm.
- Sector wrap-around. A "front" sector that spans -30 deg to +30 deg
  crosses the 360 / 0 seam and must be tested with modular arithmetic.

Run
---
    python experiments/07_obstacle_detector.py
    python experiments/07_obstacle_detector.py --center 0 --width 60 --threshold-mm 500

Expected output
---------------
    Watching sector center=0 deg width=60 deg threshold=500 mm.
    rev   1: clear  (closest in sector: 1842 mm)
    rev   2: clear  (closest in sector: 1820 mm)
    rev   3: ALERT  (closest in sector:  412 mm at angle  -5.3)
    ...

Common failures
---------------
- Permanent ALERT: the chosen threshold is larger than the room. Lower
  --threshold-mm or change the sector.
- Permanent "clear" even with an obstacle nearby: the sector is pointing
  the wrong way. Recall that 0 deg is along +X in the body frame, and
  body +X usually points away from the cable. Walk through experiment 5
  to confirm which way the sensor is facing.
"""

from __future__ import annotations

import argparse
import glob
import platform
import signal
import sys
from typing import Optional

from pyrplidar import PyRPlidar


DEFAULT_BAUDRATE = 460800
DEFAULT_MOTOR_PWM = 500


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
    """True if angle_deg is within +/- width_deg/2 of center_deg, wrapping at 360."""
    half = width_deg / 2.0
    diff = abs((angle_deg - center_deg + 180.0) % 360.0 - 180.0)
    return diff <= half


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reactive obstacle detector.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--center", type=float, default=0.0,
                        help="Sector center, degrees, body frame.")
    parser.add_argument("--width", type=float, default=60.0,
                        help="Sector width, degrees.")
    parser.add_argument("--threshold-mm", type=float, default=500.0,
                        help="Alert when closest point is below this distance.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    interrupted = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: interrupted.__setitem__("flag", True))

    lidar = PyRPlidar()
    try:
        lidar.connect(port=port, baudrate=args.baudrate, timeout=3.0)
        lidar.set_motor_pwm(args.motor_pwm)
        scan_gen = lidar.start_scan()

        print(f"Watching sector center={args.center} deg "
              f"width={args.width} deg threshold={args.threshold_mm:.0f} mm.")

        closest_dist: float = float("inf")
        closest_angle: float = 0.0
        revolution_idx = 0

        for sample in scan_gen():
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
    finally:
        try:
            lidar.stop()
            lidar.set_motor_pwm(0)
        except Exception as exc:
            print(f"Warning: shutdown step failed: {exc}", file=sys.stderr)
        try:
            lidar.disconnect()
        except Exception as exc:
            print(f"Warning: disconnect failed: {exc}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
