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
- The unit conventions: distance is in millimetres, angle is in degrees
  as reported by the sensor, theta in the math is radians. Mixing those
  up is the most common source of wrong-looking plots in later
  experiments.
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
       1:  angle=  1.55 deg  d= 2406.5 mm  ->  x= +2405.6  y=   +65.0
       ...
    Bounding box of this scan in body frame:
      x_min= -3219.5   x_max= +3450.0
      y_min= -2880.0   y_max= +2762.5

Common failures
---------------
- "Captured zero samples": the scan generator returned nothing. Most
  often the motor failed to start. Listen for the rotor.
- All x and y are near zero: every sample has distance=0. Either the
  environment is at >12 m (the C1 max range) or the optical head is
  obstructed.
"""

from __future__ import annotations

import argparse
import glob
import math
import platform
import sys
from typing import List, Optional, Tuple

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


def polar_to_cartesian(angle_deg: float, distance_mm: float) -> Tuple[float, float]:
    """Convert a single polar sample to cartesian (x_mm, y_mm).

    The sensor reports angle in degrees, increasing counter-clockwise,
    with zero along the +X axis of the body frame. We convert to radians
    for the trig functions.
    """
    theta = math.radians(angle_deg)
    x = distance_mm * math.cos(theta)
    y = distance_mm * math.sin(theta)
    return x, y


def collect_one_revolution(scan_iter) -> List[Tuple[int, float, float]]:
    """Pull samples until we see a complete revolution.

    A revolution starts on the first sample whose start_flag is True
    and ends just before the next such sample. We discard anything
    before the first start_flag because it may be a partial fragment.
    """
    samples: List[Tuple[int, float, float]] = []
    seen_start = False
    for sample in scan_iter:
        is_start = bool(sample.start_flag)
        if not seen_start:
            if is_start:
                seen_start = True
                samples.append((sample.quality, sample.angle, sample.distance))
            continue
        if is_start:
            return samples
        samples.append((sample.quality, sample.angle, sample.distance))
    return samples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert one revolution from polar to cartesian.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--max-points", type=int, default=10,
                        help="Maximum number of valid points to print.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    lidar = PyRPlidar()
    try:
        lidar.connect(port=port, baudrate=args.baudrate, timeout=3.0)
        lidar.set_motor_pwm(args.motor_pwm)
        scan_gen = lidar.start_scan()
        raw = collect_one_revolution(scan_gen())

        if not raw:
            print("Captured zero samples in this revolution.", file=sys.stderr)
            return 1

        # Drop zero-distance samples: those are non-returns, not obstacles
        # at the origin. See docs/protocol-notes.md.
        valid = [(q, a, d) for (q, a, d) in raw if d > 0.0]
        dropped = len(raw) - len(valid)
        print(f"Captured one revolution: {len(raw)} samples "
              f"({len(valid)} valid, {dropped} dropped).")

        print(f"First {min(args.max_points, len(valid))} valid points in body "
              f"frame (x_mm, y_mm):")
        for idx, (q, angle, dist) in enumerate(valid[: args.max_points]):
            x, y = polar_to_cartesian(angle, dist)
            print(f"  {idx:3d}:  angle={angle:6.2f} deg  d={dist:7.1f} mm  "
                  f"->  x={x:+9.1f}  y={y:+9.1f}")

        xs = [polar_to_cartesian(a, d)[0] for (_, a, d) in valid]
        ys = [polar_to_cartesian(a, d)[1] for (_, a, d) in valid]
        if xs and ys:
            print("Bounding box of this scan in body frame:")
            print(f"  x_min={min(xs):+9.1f}   x_max={max(xs):+9.1f}")
            print(f"  y_min={min(ys):+9.1f}   y_max={max(ys):+9.1f}")

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
