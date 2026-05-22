"""
08_save_scan_to_csv.py
======================

Purpose
-------
Capture N complete revolutions and write them to a CSV file in `data/`.
The CSV is the format used by experiment 09 for offline replay, so
this is what lets a teammate without a LiDAR work with the rest of the
repo.

Run
---
    python experiments/08_save_scan_to_csv.py --revolutions 50
    python experiments/08_save_scan_to_csv.py --revolutions 20 \\
        --output data/sample_room_01.csv
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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lib.rplidar_c1 import RPLidarC1, DEFAULT_MOTOR_PWM  # noqa: E402


DEFAULT_BAUDRATE = 460800
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
    parser.add_argument("--revolutions", type=int, default=DEFAULT_REVOLUTIONS)
    parser.add_argument("--output", type=Path, default=None)
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

    rows_written = 0
    try:
        with RPLidarC1(port=port, baudrate=args.baudrate, timeout=2.0) as lidar:
            lidar.set_motor_pwm(args.motor_pwm)
            print(f"Recording {args.revolutions} revolutions to {output_path}")

            with output_path.open("w", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(["timestamp_unix_s", "scan_index", "angle_deg",
                                 "distance_mm", "quality", "start_flag"])

                scan_idx = -1
                per_rev_count = 0
                for sample in lidar.iter_scans():
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
                        continue
                    writer.writerow([
                        f"{time.time():.6f}",
                        scan_idx,
                        f"{sample.angle:.4f}",
                        f"{sample.distance:.2f}",
                        sample.quality,
                        int(sample.start_flag),
                    ])
                    rows_written += 1
                    per_rev_count += 1

            print(f"Wrote {rows_written} rows to {output_path}")
    except Exception as exc:
        print(f"Recording failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
