"""
01_hello_lidar.py
=================

Purpose
-------
The smallest possible end-to-end LiDAR program. Find the serial port,
connect, ask the sensor who it is, then disconnect cleanly. The motor is
not started in this experiment; we only need the protocol handshake to
prove that the host can talk to the sensor.

What this teaches
-----------------
- How to discover the LiDAR's serial port on macOS and Linux.
- That talking to a USB-serial device is just opening a file and reading
  and writing bytes at an agreed baud rate.
- The shape of a clean connect / use / disconnect lifecycle. Every later
  experiment uses the same shape, with a motor and a scan in the middle.

Run
---
    python experiments/01_hello_lidar.py
    python experiments/01_hello_lidar.py --port /dev/cu.usbserial-0001

Expected output
---------------
    Auto-detected port: /dev/cu.usbserial-0001
    Connecting at 460800 baud...
    Connected.
    Device info:
      model:        ...
      firmware:     ...
      hardware:     ...
      serialnumber: ...
    Disconnecting.
    Done.

Common failures
---------------
- "No candidate ports found": the OS does not see the adapter. Try a
  different USB cable. On Linux, confirm your user is in 'dialout'.
- "Failed to connect": the port exists but the protocol handshake did
  not complete. Most often, the baud rate is wrong or another program
  has the port open.
"""

from __future__ import annotations

import argparse
import glob
import platform
import sys
from typing import Optional

from pyrplidar import PyRPlidar


# The C1 default baud rate. Other RPLIDAR models use 115200 (A1) or
# 256000 (S/T series); if you reuse this code on a different unit, this
# is the first constant to revisit.
DEFAULT_BAUDRATE = 460800


def auto_detect_port() -> Optional[str]:
    """Return the first plausible RPLIDAR port on this OS, or None.

    On macOS we look for /dev/cu.usbserial-* and /dev/cu.SLAB_USBtoUART.
    On Linux we look for /dev/ttyUSB*. We deliberately do not open the
    port here; that happens later, with proper error handling.
    """
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
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port path. If omitted, auto-detect.",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"Serial baud rate. Default: {DEFAULT_BAUDRATE} (C1 default).",
    )
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

    lidar = PyRPlidar()
    print(f"Connecting at {args.baudrate} baud...")
    try:
        # The timeout here is for the underlying serial read. A few
        # seconds is plenty for the small INFO response.
        lidar.connect(port=port, baudrate=args.baudrate, timeout=3.0)
        print("Connected.")

        info = lidar.get_info()
        # pyrplidar returns objects with __str__; we print fields by name
        # so the output is grep-friendly and stable.
        print("Device info:")
        for field in ("model", "firmware", "hardware", "serialnumber"):
            value = getattr(info, field, None)
            print(f"  {field + ':':<14}{value}")

    except Exception as exc:
        # We deliberately do not silently swallow this. Bubble enough
        # detail to diagnose, but keep the message single-line-readable.
        print(f"Failed to talk to the LiDAR on {port}: {exc}", file=sys.stderr)
        return 1
    finally:
        # Always release the port. Doing this in finally means it runs
        # on success, on exceptions, and on Ctrl+C.
        try:
            print("Disconnecting.")
            lidar.disconnect()
        except Exception as exc:
            print(f"Warning: disconnect failed: {exc}", file=sys.stderr)

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
