"""
05_realtime_viz_matplotlib.py
=============================

Purpose
-------
Display a live, 2D top-down view of the room as the sensor sees it. The
sensor sits at the origin and points are drawn around it, refreshing
each revolution. Close the matplotlib window or press Ctrl+C in the
terminal to stop.

What this teaches
-----------------
- Real-time visualization in matplotlib using a single Figure that we
  reuse across frames. We update the scatter data in place rather than
  creating a new plot each tick, which avoids flicker and memory
  growth.
- That a 2D LiDAR scan is just a set of points in the plane. The "room"
  emerges from their distribution.

Run
---
    python experiments/05_realtime_viz_matplotlib.py
    python experiments/05_realtime_viz_matplotlib.py --max-distance-m 6

Expected output
---------------
    A matplotlib window opens. As you carry the sensor around or move
    objects in the room, the dots redraw at roughly 10 Hz. The terminal
    prints one short status line per revolution.
"""

from __future__ import annotations

import argparse
import glob
import math
import platform
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live 2D LiDAR visualization.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--max-distance-m", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    max_m = args.max_distance_m
    ax.set_xlim(-max_m, max_m)
    ax.set_ylim(-max_m, max_m)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("RPLIDAR C1 - live scan (body frame)")
    scatter = ax.scatter([], [], s=4)
    ax.plot(0, 0, marker="x", color="red", markersize=10)

    try:
        with RPLidarC1(port=port, baudrate=args.baudrate, timeout=2.0) as lidar:
            lidar.set_motor_pwm(args.motor_pwm)

            current: List[Tuple[float, float]] = []
            for sample in lidar.iter_scans():
                if sample.start_flag and current:
                    xs = np.array([p[0] for p in current]) / 1000.0  # mm -> m
                    ys = np.array([p[1] for p in current]) / 1000.0
                    scatter.set_offsets(np.column_stack([xs, ys]))
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

                    max_d = (max(math.hypot(*p) for p in current) / 1000.0
                             if current else 0.0)
                    print(f"revolution: {len(current):4d} valid points, "
                          f"max range {max_d:5.2f} m")

                    current = []
                    if not plt.fignum_exists(fig.number):
                        break

                if sample.distance > 0.0:
                    theta = math.radians(sample.angle)
                    current.append((
                        sample.distance * math.cos(theta),
                        sample.distance * math.sin(theta),
                    ))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print(f"Scan failed: {exc}", file=sys.stderr)
        return 1
    finally:
        plt.close("all")
    return 0


if __name__ == "__main__":
    sys.exit(main())
