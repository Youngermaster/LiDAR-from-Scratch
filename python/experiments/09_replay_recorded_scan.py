"""
09_replay_recorded_scan.py
==========================

Purpose
-------
Load a CSV recorded by experiment 08 and replay it through the same live
visualization used in experiment 05, at original or accelerated speed.
No LiDAR required. This is what lets you keep iterating on processing
logic on a plane or in a coffee shop.

What this teaches
-----------------
- The value of separating capture from processing. Once you have a few
  representative CSV files in `data/`, the rest of the pipeline can be
  developed without a sensor plugged in.
- Replaying does not need to match real time exactly. Speeding the
  playback up is fine for skimming a long recording; slowing it down is
  useful for catching transient events.
- The CSV format defined in experiment 08 is enough to reconstruct what
  the live experiment saw. If a later experiment needs more (for example
  raw bytes), the CSV format is the thing to upgrade.

Run
---
    python experiments/09_replay_recorded_scan.py --input data/sample_room_01.csv
    python experiments/09_replay_recorded_scan.py --input data/sample_room_01.csv --speed 4

Expected output
---------------
    Loaded 24013 rows from data/sample_room_01.csv  (50 revolutions)
    A matplotlib window opens. Each revolution is drawn once and the
    plot advances at `--speed` times original speed.

Common failures
---------------
- "FileNotFoundError": the path is wrong, or you have not recorded a
  sample yet. Run experiment 08 first.
- "KeyError" on a CSV column: the file was produced by an older version
  of experiment 08. Re-record or hand-edit the header.
- Plot freezes briefly between revolutions: matplotlib is rendering. Try
  --speed 1 if you need accurate timing, --speed 0 (no sleep) if you
  just want to skim.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a recorded scan CSV.")
    parser.add_argument("--input", type=Path, required=True,
                        help="Path to a CSV produced by experiment 08.")
    parser.add_argument("--max-distance-m", type=float, default=8.0,
                        help="Crop the plot to this radius, in metres.")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier. 0 disables sleeping.")
    return parser.parse_args()


def load_revolutions(csv_path: Path) -> List[List[Tuple[float, float, float]]]:
    """Return a list of revolutions, each a list of (angle, dist, ts) tuples."""
    revs: Dict[int, List[Tuple[float, float, float]]] = {}
    with csv_path.open() as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            scan_index = int(row["scan_index"])
            angle = float(row["angle_deg"])
            distance = float(row["distance_mm"])
            ts = float(row["timestamp_unix_s"])
            revs.setdefault(scan_index, []).append((angle, distance, ts))
    return [revs[k] for k in sorted(revs.keys())]


def main() -> int:
    args = parse_args()
    if not args.input.exists():
        print(f"Input file not found: {args.input}", file=sys.stderr)
        return 2

    revolutions = load_revolutions(args.input)
    total_rows = sum(len(r) for r in revolutions)
    print(f"Loaded {total_rows} rows from {args.input}  "
          f"({len(revolutions)} revolutions)")

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    max_m = args.max_distance_m
    ax.set_xlim(-max_m, max_m)
    ax.set_ylim(-max_m, max_m)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Replay: {args.input.name}")
    scatter = ax.scatter([], [], s=4)
    ax.plot(0, 0, marker="x", color="red", markersize=10)

    # We use the timestamps in the file to control playback pacing. For
    # each revolution we compute how long the original capture took, then
    # sleep that interval divided by --speed.
    try:
        for rev_idx, rev in enumerate(revolutions):
            if not plt.fignum_exists(fig.number):
                break

            xs: List[float] = []
            ys: List[float] = []
            for angle, dist, _ts in rev:
                if dist <= 0.0:
                    continue
                theta = math.radians(angle)
                xs.append(dist * math.cos(theta) / 1000.0)
                ys.append(dist * math.sin(theta) / 1000.0)

            scatter.set_offsets(np.column_stack([np.array(xs), np.array(ys)])
                                if xs else np.empty((0, 2)))
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            print(f"replay rev {rev_idx + 1}/{len(revolutions)}: "
                  f"{len(xs)} valid points")

            if args.speed > 0.0 and rev:
                dt = rev[-1][2] - rev[0][2]
                if dt > 0:
                    time.sleep(dt / args.speed)
                else:
                    # Approximate one nominal revolution = 100 ms (10 Hz).
                    time.sleep(0.1 / args.speed)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        plt.close("all")
    return 0


if __name__ == "__main__":
    sys.exit(main())
