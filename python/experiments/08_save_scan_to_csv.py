"""
08_save_scan_to_csv.py
======================

Purpose
-------
Capture N complete revolutions and write them to a CSV file in
`data/`. The CSV is the format used by experiment 09 for offline replay,
so this is what lets a teammate without a LiDAR work with the rest of
the repo.

What this teaches
-----------------
- Data persistence at the file level: a plain CSV, not a binary format,
  so it can be opened in any spreadsheet or text editor.
- The minimum schema you need for replay: timestamp, scan index, angle,
  distance, quality, start_flag. That is enough to reconstruct the live
  visualization downstream.
- That recording is its own concern, separate from visualization or
  processing. The CSV is the contract between capture and replay.

Run
---
    # Record 50 revolutions to data/recordings/<auto-named>.csv:
    python experiments/08_save_scan_to_csv.py --revolutions 50

    # Specify an output path explicitly:
    python experiments/08_save_scan_to_csv.py --revolutions 20 \
        --output data/sample_room_01.csv

Expected output
---------------
    Recording 50 revolutions to data/recordings/scan_20260522_180342.csv
      rev  1: 482 samples
      rev  2: 478 samples
      ...
    Wrote 24013 rows to data/recordings/scan_20260522_180342.csv

Common failures
---------------
- "data/ does not exist": the file path in --output points into a
  directory that does not exist. Create it or use --output with an
  existing parent.
- Empty CSV: the scan produced zero samples. Same root cause as in
  experiment 03; check that the motor actually spun up.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import glob
import platform
import signal
import sys
import time
from pathlib import Path
from typing import Optional

from pyrplidar import PyRPlidar


DEFAULT_BAUDRATE = 460800
DEFAULT_MOTOR_PWM = 500
DEFAULT_REVOLUTIONS = 50


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


def default_output_path() -> Path:
    """Build a timestamped path under data/recordings/."""
    repo_root = Path(__file__).resolve().parent.parent.parent
    out_dir = repo_root / "data" / "recordings"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return out_dir / f"scan_{stamp}.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record N revolutions to CSV.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--revolutions", type=int, default=DEFAULT_REVOLUTIONS,
                        help="Number of full 360-deg scans to record.")
    parser.add_argument("--output", type=Path, default=None,
                        help="CSV path. Defaults to data/recordings/scan_<ts>.csv")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    output_path: Path = args.output or default_output_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    interrupted = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: interrupted.__setitem__("flag", True))

    lidar = PyRPlidar()
    rows_written = 0
    try:
        lidar.connect(port=port, baudrate=args.baudrate, timeout=3.0)
        lidar.set_motor_pwm(args.motor_pwm)
        scan_gen = lidar.start_scan()

        print(f"Recording {args.revolutions} revolutions to {output_path}")

        with output_path.open("w", newline="") as fh:
            writer = csv.writer(fh)
            # Header. Schema documented in the file docstring above.
            writer.writerow(["timestamp_unix_s", "scan_index", "angle_deg",
                             "distance_mm", "quality", "start_flag"])

            scan_idx = -1
            per_rev_count = 0
            for sample in scan_gen():
                if interrupted["flag"]:
                    break
                if sample.start_flag:
                    if scan_idx >= 0:
                        print(f"  rev {scan_idx + 1:3d}: {per_rev_count} samples")
                    scan_idx += 1
                    per_rev_count = 0
                    if scan_idx >= args.revolutions:
                        break
                if scan_idx < 0:
                    # Skip the partial fragment before the first start_flag.
                    continue
                writer.writerow([
                    f"{time.time():.6f}",
                    scan_idx,
                    f"{sample.angle:.4f}",
                    f"{sample.distance:.2f}",
                    sample.quality,
                    int(bool(sample.start_flag)),
                ])
                rows_written += 1
                per_rev_count += 1

        print(f"Wrote {rows_written} rows to {output_path}")

    except Exception as exc:
        print(f"Recording failed: {exc}", file=sys.stderr)
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
