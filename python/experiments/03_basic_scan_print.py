"""
03_basic_scan_print.py
======================

Purpose
-------
Start the motor, request a stream of scan samples, print the first N of
them, and stop the motor cleanly. This is the first experiment where
the sensor actually spins and produces data.

What this teaches
-----------------
- The scan data structure: each sample is (quality, angle_deg,
  distance_mm, start_flag). The start_flag bit marks the first sample
  of a new revolution.
- That distance == 0 is a special value meaning "no return". You will
  see plenty of zeros, especially indoors with windows or open spaces.
- That the motor must be stopped on exit, including on Ctrl+C. The
  RPLidarC1 context manager guarantees this.

Run
---
    # Print 200 samples then stop:
    python experiments/03_basic_scan_print.py --count 200

    # Print continuously, Ctrl+C to stop:
    python experiments/03_basic_scan_print.py --count 0

Expected output
---------------
    Port: /dev/cu.usbserial-1130  (baudrate=460800)
    Starting motor at PWM 660.
    Scanning. Press Ctrl+C to stop.
    [    0]  start=1  q=15  angle=  0.84  dist= 2410.0
    [    1]  start=0  q=15  angle=  1.55  dist= 2406.5
    ...

Common failures
---------------
- All samples have distance=0 and quality=0: the sensor is connected
  but not seeing anything. Cover the head with your hand; the next
  revolution should show a small distance.
- "Bad response descriptor sync bytes": a previous run left the sensor
  streaming. Replug and try again.
"""

from __future__ import annotations

import argparse
import glob
import platform
import signal
import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lib.rplidar_c1 import RPLidarC1, DEFAULT_MOTOR_PWM  # noqa: E402


DEFAULT_BAUDRATE = 460800
DEFAULT_SCAN_COUNT = 200


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
    parser = argparse.ArgumentParser(description="Print raw scan samples.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM,
                        help=f"PWM 0..1023. C1 default: {DEFAULT_MOTOR_PWM}.")
    parser.add_argument("--count", type=int, default=DEFAULT_SCAN_COUNT,
                        help="Number of samples to print. 0 = run until Ctrl+C.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    print(f"Port: {port}  (baudrate={args.baudrate})")

    interrupted = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: interrupted.__setitem__("flag", True))

    try:
        with RPLidarC1(port=port, baudrate=args.baudrate, timeout=2.0) as lidar:
            print(f"Starting motor at PWM {args.motor_pwm}.")
            lidar.set_motor_pwm(args.motor_pwm)

            print("Scanning. Press Ctrl+C to stop.")
            for idx, sample in enumerate(lidar.iter_scans()):
                if interrupted["flag"]:
                    break
                print(
                    f"[{idx:5d}]  "
                    f"start={int(sample.start_flag)}  "
                    f"q={sample.quality:3d}  "
                    f"angle={sample.angle:7.2f}  "
                    f"dist={sample.distance:7.1f}"
                )
                if args.count > 0 and idx + 1 >= args.count:
                    break
    except Exception as exc:
        print(f"Scan failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
