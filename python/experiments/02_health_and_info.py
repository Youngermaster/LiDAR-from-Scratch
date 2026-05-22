"""
02_health_and_info.py
=====================

Purpose
-------
Pretty-print everything the sensor will tell us about itself before we
start scanning: device info, health status, sample rate, and the list of
supported scan modes. This is the diagnostic that comes before any
serious work in later experiments.

What this teaches
-----------------
- The difference between INFO (immutable identity) and HEALTH (current
  state) requests.
- That the sensor offers several scan modes, each with its own per-sample
  time budget. Picking a mode is a tradeoff between angular resolution
  and sensitivity.
- How to interpret the health code. 0 = OK, 1 = Warning, 2 = Error. The
  error case is rare but you want to see it before you ignore broken
  data downstream.

Run
---
    python experiments/02_health_and_info.py
    python experiments/02_health_and_info.py --port /dev/cu.usbserial-0001

Expected output
---------------
    Port: /dev/cu.usbserial-0001  (baudrate=460800)

    INFO
      model:        ...
      firmware:     ...
      hardware:     ...
      serialnumber: ...

    HEALTH
      status:       OK
      error_code:   0

    SAMPLERATE
      standard:     ... us per sample
      express:      ... us per sample

    SCAN MODES
      [ID 0] Standard           sample_us=... max_distance_m=... ans_type=...
      [ID 1] Express            ...
      [ID 2] Boost              ...

Common failures
---------------
- A "Warning" or "Error" in HEALTH means the sensor reports an internal
  problem. Sometimes a power cycle fixes it. If it persists across a
  reboot, the sensor likely needs RMA.
- An empty SCAN MODES list usually means the protocol negotiation failed,
  which on the C1 typically means the baud rate is wrong.
"""

from __future__ import annotations

import argparse
import glob
import platform
import sys
from typing import Optional

from pyrplidar import PyRPlidar


DEFAULT_BAUDRATE = 460800

# pyrplidar exposes health.status as an int code; map for human display.
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
    parser = argparse.ArgumentParser(description="Print health, info, and scan modes.")
    parser.add_argument("--port", default=None, help="Serial port. Auto-detect if omitted.")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    print(f"Port: {port}  (baudrate={args.baudrate})")
    lidar = PyRPlidar()
    try:
        lidar.connect(port=port, baudrate=args.baudrate, timeout=3.0)

        info = lidar.get_info()
        print("\nINFO")
        for field in ("model", "firmware", "hardware", "serialnumber"):
            print(f"  {field + ':':<14}{getattr(info, field, None)}")

        health = lidar.get_health()
        # Some firmware revisions return status as an int, others as an
        # object with a .status attribute. Be defensive.
        status_code = getattr(health, "status", health)
        error_code = getattr(health, "error_code", 0)
        print("\nHEALTH")
        print(f"  {'status:':<14}{HEALTH_STATUS_NAMES.get(int(status_code), status_code)}")
        print(f"  {'error_code:':<14}{error_code}")

        rate = lidar.get_samplerate()
        # Fields are typically t_standard and t_express, in microseconds.
        std_us = getattr(rate, "t_standard", None)
        exp_us = getattr(rate, "t_express", None)
        print("\nSAMPLERATE")
        print(f"  {'standard:':<14}{std_us} us per sample")
        print(f"  {'express:':<14}{exp_us} us per sample")

        modes = lidar.get_scan_modes()
        print("\nSCAN MODES")
        if not modes:
            print("  (no modes reported)")
        for mode in modes:
            mode_id = getattr(mode, "id", "?")
            name = getattr(mode, "name", "?")
            sample_us = getattr(mode, "us_per_sample", "?")
            max_d = getattr(mode, "max_distance", "?")
            ans = getattr(mode, "ans_type", "?")
            print(f"  [ID {mode_id}] {name:<16} sample_us={sample_us} "
                  f"max_distance_m={max_d} ans_type={ans}")

    except Exception as exc:
        print(f"Failed to query the sensor on {port}: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            lidar.disconnect()
        except Exception as exc:
            print(f"Warning: disconnect failed: {exc}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
