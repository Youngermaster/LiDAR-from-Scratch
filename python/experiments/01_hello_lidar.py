"""
01_hello_lidar.py
=================

Purpose
-------
The smallest possible end-to-end LiDAR program. Find the serial port,
connect, ask the sensor who it is, then disconnect cleanly. The motor
is not started here; we only need the protocol handshake to prove that
the host can talk to the sensor.

What this teaches
-----------------
- How to discover the LiDAR's serial port on macOS and Linux.
- That talking to a USB-serial device is just opening a file and
  reading and writing bytes at an agreed baud rate.
- The shape of a clean connect / use / disconnect lifecycle. Every
  later experiment uses the same shape, with a motor and a scan in the
  middle.

Run
---
    python experiments/01_hello_lidar.py
    python experiments/01_hello_lidar.py --port /dev/cu.usbserial-1130

Expected output
---------------
    Auto-detected port: /dev/cu.usbserial-1130
    Connecting at 460800 baud...
    Connected.
    Device info:
      model:        65          (0x41, RPLIDAR C1)
      firmware:     1.02
      hardware:     18
      serialnumber: F34CE0F8C2E29AD2C1819FF500FD4E1E
    Done.

Common failures
---------------
- "No candidate ports found": the OS does not see the adapter. On
  macOS, also check Privacy & Security -> "Allow accessories to
  connect" (see docs/hardware-setup.md).
- "Bad response descriptor sync bytes": another program is reading the
  port, or a previous run left the sensor in a streaming state.
  Disconnect and reconnect the LiDAR and try again.
"""

from __future__ import annotations

import argparse
import glob
import platform
import sys
from pathlib import Path
from typing import Optional

# Allow `from lib.rplidar_c1 import ...` when running this file directly.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lib.rplidar_c1 import RPLidarC1  # noqa: E402


DEFAULT_BAUDRATE = 460800


def auto_detect_port() -> Optional[str]:
    """Return the first plausible RPLIDAR port on this OS, or None."""
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
    parser = argparse.ArgumentParser(description="Hello, LiDAR.")
    parser.add_argument("--port", default=None,
                        help="Serial port path. If omitted, auto-detect.")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE,
                        help=f"Default: {DEFAULT_BAUDRATE} (C1 default).")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    port = args.port or auto_detect_port()
    if port is None:
        print(
            "No candidate ports found. Run scripts/detect_lidar_port.sh\n"
            "and pass the result with --port.",
            file=sys.stderr,
        )
        return 2
    print(f"Auto-detected port: {port}" if args.port is None else f"Using port: {port}")

    print(f"Connecting at {args.baudrate} baud...")
    try:
        with RPLidarC1(port=port, baudrate=args.baudrate, timeout=2.0) as lidar:
            print("Connected.")
            info = lidar.get_info()
            print("Device info:")
            model_note = "  (0x41, RPLIDAR C1)" if info.model == 0x41 else ""
            print(f"  {'model:':<14}{info.model}{model_note}")
            print(f"  {'firmware:':<14}{info.firmware}")
            print(f"  {'hardware:':<14}{info.hardware}")
            print(f"  {'serialnumber:':<14}{info.serial_hex}")
    except Exception as exc:
        print(f"Failed to talk to the LiDAR on {port}: {exc}", file=sys.stderr)
        return 1

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
