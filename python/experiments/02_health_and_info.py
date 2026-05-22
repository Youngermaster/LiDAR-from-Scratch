"""
02_health_and_info.py
=====================

Purpose
-------
Pretty-print everything our minimal driver can extract from the sensor:
identity (model, firmware, hardware, serial) and health status. This is
the diagnostic that comes before any serious work in later experiments.

What this teaches
-----------------
- The difference between INFO (immutable identity) and HEALTH (current
  state) requests.
- How to interpret the health code: 0 = OK, 1 = Warning, 2 = Error.
  The error case is rare but you want to see it before you ignore
  broken data downstream.

Note: this driver does not implement `GET_LIDAR_CONF` (scan-mode
enumeration). The C1's legacy SCAN command is what we use everywhere
else, so the scan-mode list is not strictly required. If you want to
inspect modes, the SLAMTEC vendor tool (rplidar_frame_grabber) prints
them.

Run
---
    python experiments/02_health_and_info.py

Expected output
---------------
    Port: /dev/cu.usbserial-1130  (baudrate=460800)

    INFO
      model:        65   (0x41, RPLIDAR C1)
      firmware:     1.02
      hardware:     18
      serialnumber: F34CE0F8C2E29AD2C1819FF500FD4E1E

    HEALTH
      status:       OK
      error_code:   0

Common failures
---------------
- A "Warning" or "Error" in HEALTH means the sensor reports an internal
  problem. Sometimes a power cycle fixes it. If it persists across a
  reboot, the sensor likely needs RMA.
"""

from __future__ import annotations

import argparse
import glob
import platform
import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lib.rplidar_c1 import RPLidarC1  # noqa: E402


DEFAULT_BAUDRATE = 460800

HEALTH_STATUS_NAMES = {0: "OK", 1: "Warning", 2: "Error"}


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
    parser = argparse.ArgumentParser(description="Print health and info.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    print(f"Port: {port}  (baudrate={args.baudrate})")
    try:
        with RPLidarC1(port=port, baudrate=args.baudrate, timeout=2.0) as lidar:
            info = lidar.get_info()
            print("\nINFO")
            model_note = "   (0x41, RPLIDAR C1)" if info.model == 0x41 else ""
            print(f"  {'model:':<14}{info.model}{model_note}")
            print(f"  {'firmware:':<14}{info.firmware}")
            print(f"  {'hardware:':<14}{info.hardware}")
            print(f"  {'serialnumber:':<14}{info.serial_hex}")

            health = lidar.get_health()
            status_name = HEALTH_STATUS_NAMES.get(health.status, str(health.status))
            print("\nHEALTH")
            print(f"  {'status:':<14}{status_name}")
            print(f"  {'error_code:':<14}{health.error_code}")
    except Exception as exc:
        print(f"Failed to query the sensor on {port}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
