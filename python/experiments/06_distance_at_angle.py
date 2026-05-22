"""
06_distance_at_angle.py
=======================

Purpose
-------
Given an angle (in degrees, in the sensor body frame), report the live
distance to whatever obstacle is in that direction. Use an angular
tolerance so we are not at the mercy of which exact sample falls at
exactly that bearing, and smooth across recent revolutions so the
displayed number stops bouncing around on noise.

What this teaches
-----------------
- Filtering scan data by bearing: a single number from a 500-sample scan
  by selecting the slice within +/- tolerance of the target angle.
- Wrap-around at 0 / 360 degrees. Asking for "0 degrees +/- 5 deg" must
  include samples at both 355 and 5.
- Why smoothing matters: even at rest, the per-sample distance jitters
  by a few tens of millimetres. A short moving median is the cheapest
  way to make the displayed number stable enough to act on.

Run
---
    # Forward (0 deg in body frame, the cable-out side of the unit):
    python experiments/06_distance_at_angle.py --angle 0

    # Right side, +/- 10 deg tolerance:
    python experiments/06_distance_at_angle.py --angle 90 --tolerance 10

Expected output
---------------
    Watching angle 0.0 deg, +/- 5.0 deg. Press Ctrl+C to stop.
       1: 2410 mm  (samples in sector: 14)
       2: 2407 mm  (samples in sector: 12)
       3: 2413 mm  (samples in sector: 13)
    ...

Common failures
---------------
- "No samples in sector": the tolerance is too tight, or the chosen
  bearing happens to point at a window or open space (no return).
  Increase --tolerance or change the bearing.
- The number changes wildly between revolutions: lower the motor PWM,
  let the sensor warm up for a few seconds, or increase --smoothing.
"""

from __future__ import annotations

import argparse
import collections
import glob
import platform
import signal
import sys
from typing import List, Optional

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


def angular_distance_deg(a: float, b: float) -> float:
    """Smallest absolute distance between two bearings, wrapped to [0, 180]."""
    diff = abs((a - b + 180.0) % 360.0 - 180.0)
    return diff


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

    lidar = PyRPlidar()
    try:
        lidar.connect(port=port, baudrate=args.baudrate, timeout=3.0)
        lidar.set_motor_pwm(args.motor_pwm)
        scan_gen = lidar.start_scan()

        print(f"Watching angle {args.angle} deg, +/- {args.tolerance} deg. "
              f"Press Ctrl+C to stop.")

        # Per-revolution buffer of distances in the target sector.
        sector: List[float] = []
        # Rolling median across the last N revolutions.
        recent_medians: collections.deque[float] = collections.deque(
            maxlen=max(1, args.smoothing))
        revolution_idx = 0

        for sample in scan_gen():
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
