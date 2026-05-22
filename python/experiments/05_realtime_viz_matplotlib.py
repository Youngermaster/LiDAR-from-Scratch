"""
05_realtime_viz_matplotlib.py
=============================

Purpose
-------
Display a live, 2D top-down view of the room as the sensor sees it. The
sensor sits at the origin and points are drawn around it, refreshing each
revolution. Close the matplotlib window or press Ctrl+C in the terminal
to stop.

What this teaches
-----------------
- Real-time visualization in matplotlib using a single Figure that we
  reuse across frames. We update the scatter data in place rather than
  creating a new plot each tick, which avoids flicker and memory growth.
- That a 2D LiDAR scan is just a set of points in the plane. The "room"
  emerges from their distribution.
- Decimation: at high sample rates you do not need to plot every point
  to see the shape of the room. Drop a fraction if the plot lags behind
  the sensor.

Run
---
    python experiments/05_realtime_viz_matplotlib.py
    python experiments/05_realtime_viz_matplotlib.py --max-distance-m 6

Expected output
---------------
    A matplotlib window opens. As you carry the sensor around or move
    objects in the room, the dots redraw at roughly 10 Hz. The terminal
    prints one short status line per revolution showing the number of
    valid points and the maximum distance in that scan.

Common failures
---------------
- "No display available" on headless Linux: matplotlib needs a display.
  Either run X-forwarded over SSH or save scans with experiment 08 and
  visualize on a graphical machine with experiment 09.
- Plot is empty: every sample is invalid. See experiment 03's failure
  notes; the sensor is not actually seeing anything.
- Plot lags behind real time: lower --max-distance-m to crop the axes
  cheaply, or run experiment 03 to confirm the sensor itself is keeping
  up.
"""

from __future__ import annotations

import argparse
import glob
import math
import platform
import sys
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live 2D LiDAR visualization.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--max-distance-m", type=float, default=8.0,
                        help="Crop the plot to this radius, in metres.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    # Set up the figure once and update its data each revolution. This is
    # the standard matplotlib pattern for real-time visualization that
    # neither flickers nor leaks.
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
    # A small marker at the origin to remind the reader where the sensor is.
    ax.plot(0, 0, marker="x", color="red", markersize=10)

    lidar = PyRPlidar()
    try:
        lidar.connect(port=port, baudrate=args.baudrate, timeout=3.0)
        lidar.set_motor_pwm(args.motor_pwm)

        scan_gen = lidar.start_scan()
        current: List[Tuple[float, float]] = []

        for sample in scan_gen():
            # When start_flag is True we have begun a new revolution. We
            # flush the previous one to the plot and start collecting
            # the new one.
            if sample.start_flag and current:
                xs = np.array([p[0] for p in current]) / 1000.0  # mm -> m
                ys = np.array([p[1] for p in current]) / 1000.0
                scatter.set_offsets(np.column_stack([xs, ys]))
                fig.canvas.draw_idle()
                fig.canvas.flush_events()

                # One status line per revolution to spot stalls quickly.
                if len(current) > 0:
                    max_d = max(math.hypot(*p) for p in current) / 1000.0
                else:
                    max_d = 0.0
                print(f"revolution: {len(current):4d} valid points, "
                      f"max range {max_d:5.2f} m")

                current = []

                # If the user closed the plot window, exit gracefully.
                if not plt.fignum_exists(fig.number):
                    break

            if sample.distance > 0.0:
                theta = math.radians(sample.angle)
                x = sample.distance * math.cos(theta)
                y = sample.distance * math.sin(theta)
                current.append((x, y))

    except KeyboardInterrupt:
        print("\nInterrupted.")
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
        plt.close("all")
    return 0


if __name__ == "__main__":
    sys.exit(main())
