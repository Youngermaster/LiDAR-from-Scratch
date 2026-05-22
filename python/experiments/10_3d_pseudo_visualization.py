"""
10_3d_pseudo_visualization.py
=============================

Purpose
-------
Stack consecutive 2D scans along a Z axis and render them with
matplotlib's 3D scatter. The Z axis here represents *time*, not height.
This is the experiment that makes the limits of a 2D LiDAR concrete:
without a tilt mechanism or motion through space, "3D" from a C1 is
really just "many 2D scans drawn on the same canvas".

What this teaches
-----------------
- The difference between a 2D LiDAR and a 3D LiDAR. A 2D unit measures a
  ring; stacking rings over time does not produce true 3D structure
  unless the sensor itself moves or rotates out of plane.
- The case for a depth camera (Intel RealSense, Stereolabs) or a 3D
  LiDAR (Velodyne, Ouster) later, when 3D structure is actually needed.
- How to render a 3D scatter cleanly: axes-equal in the plane, a small
  marker at the origin for orientation, and a colormap on Z so older
  scans fade visually.

Run
---
    # Live, accumulating N revolutions then redrawing:
    python experiments/10_3d_pseudo_visualization.py --revolutions 30

    # Or replay from a CSV recorded with experiment 08:
    python experiments/10_3d_pseudo_visualization.py --input data/sample_room_01.csv

Expected output
---------------
    A 3D matplotlib window opens. Each revolution becomes a slice in Z.
    Older slices use cooler colours. The scene rotates with the mouse;
    use the toolbar to reset the view.

Common failures
---------------
- "matplotlib 3D toolkit not available": ancient matplotlib. Upgrade.
- Plot is flat-looking: increase --revolutions. A single ring on Z=0
  does not show structure.
- Memory growth over very long runs: rendered points accumulate. Restart
  the experiment with --revolutions and a finite count, or use --input
  to replay a recording.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import platform
import signal
import sys
from pathlib import Path
from typing import Iterator, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

# 3D toolkit is registered as a side effect of importing this submodule.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

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


def iter_revolutions_from_csv(
    csv_path: Path,
) -> Iterator[List[Tuple[float, float]]]:
    """Yield one (angle, distance) list per revolution from a recorded CSV."""
    current_idx: Optional[int] = None
    bucket: List[Tuple[float, float]] = []
    with csv_path.open() as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            scan_index = int(row["scan_index"])
            angle = float(row["angle_deg"])
            distance = float(row["distance_mm"])
            if current_idx is None:
                current_idx = scan_index
            if scan_index != current_idx:
                yield bucket
                bucket = []
                current_idx = scan_index
            bucket.append((angle, distance))
    if bucket:
        yield bucket


def iter_revolutions_from_lidar(
    port: str, baudrate: int, motor_pwm: int, interrupted
) -> Iterator[List[Tuple[float, float]]]:
    """Yield one (angle, distance) list per revolution from a live sensor.

    Cleanly stops the motor on exit via the caller's `finally` block; we
    do not own the lidar handle here, so this function is a generator
    whose lifetime ends when the caller stops iterating.
    """
    lidar = PyRPlidar()
    try:
        lidar.connect(port=port, baudrate=baudrate, timeout=3.0)
        lidar.set_motor_pwm(motor_pwm)
        scan_gen = lidar.start_scan()

        bucket: List[Tuple[float, float]] = []
        seen_start = False
        for sample in scan_gen():
            if interrupted["flag"]:
                break
            if sample.start_flag:
                if seen_start and bucket:
                    yield bucket
                    bucket = []
                seen_start = True
            if seen_start and sample.distance > 0.0:
                bucket.append((sample.angle, sample.distance))
        if bucket:
            yield bucket
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stack 2D scans on Z (time) as a pseudo-3D cloud.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--revolutions", type=int, default=30,
                        help="Number of stacked slices. Ignored if --input is used.")
    parser.add_argument("--max-distance-m", type=float, default=8.0)
    parser.add_argument("--input", type=Path, default=None,
                        help="Replay from CSV instead of using a live sensor.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    interrupted = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: interrupted.__setitem__("flag", True))

    plt.ion()
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(projection="3d")
    max_m = args.max_distance_m
    ax.set_xlim(-max_m, max_m)
    ax.set_ylim(-max_m, max_m)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("scan index (time)")
    ax.set_title("RPLIDAR C1 - pseudo-3D (2D scans stacked over time)")

    if args.input is not None:
        if not args.input.exists():
            print(f"Input file not found: {args.input}", file=sys.stderr)
            return 2
        rev_iter: Iterator[List[Tuple[float, float]]] = iter_revolutions_from_csv(args.input)
        limit = None
    else:
        port = args.port or auto_detect_port()
        if port is None:
            print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
            return 2
        rev_iter = iter_revolutions_from_lidar(port, args.baudrate, args.motor_pwm, interrupted)
        limit = args.revolutions

    all_xs: List[float] = []
    all_ys: List[float] = []
    all_zs: List[float] = []

    try:
        for rev_idx, rev in enumerate(rev_iter):
            if interrupted["flag"]:
                break
            for angle, dist in rev:
                theta = math.radians(angle)
                all_xs.append(dist * math.cos(theta) / 1000.0)
                all_ys.append(dist * math.sin(theta) / 1000.0)
                all_zs.append(float(rev_idx))

            ax.cla()
            ax.set_xlim(-max_m, max_m)
            ax.set_ylim(-max_m, max_m)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("scan index (time)")
            ax.set_title("RPLIDAR C1 - pseudo-3D (2D scans stacked over time)")
            ax.scatter(all_xs, all_ys, all_zs, s=2, c=all_zs, cmap="viridis")
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

            if not plt.fignum_exists(fig.number):
                break
            if limit is not None and rev_idx + 1 >= limit:
                break

        # Block until the user closes the window so they can interact
        # with the final pseudo-3D cloud.
        plt.ioff()
        if plt.fignum_exists(fig.number):
            plt.show()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        plt.close("all")
    return 0


if __name__ == "__main__":
    sys.exit(main())
