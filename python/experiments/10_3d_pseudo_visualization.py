"""
10_3d_pseudo_visualization.py
=============================

Purpose
-------
Stack consecutive 2D scans along a Z axis and render them with
matplotlib's 3D scatter. The Z axis here represents *time*, not height.
This experiment makes the limits of a 2D LiDAR concrete: without a tilt
mechanism or motion through space, "3D" from a C1 is really just "many
2D scans drawn on the same canvas".

Run
---
    python experiments/10_3d_pseudo_visualization.py --revolutions 30
    python experiments/10_3d_pseudo_visualization.py --input data/sample_room_01.csv
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

# 3D toolkit is registered as a side effect of importing this submodule.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

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


def iter_revolutions_from_csv(
    csv_path: Path,
) -> Iterator[List[Tuple[float, float]]]:
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
    with RPLidarC1(port=port, baudrate=baudrate, timeout=2.0) as lidar:
        lidar.set_motor_pwm(motor_pwm)
        bucket: List[Tuple[float, float]] = []
        seen_start = False
        for sample in lidar.iter_scans():
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stack 2D scans on Z (time) as a pseudo-3D cloud.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--revolutions", type=int, default=30)
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
