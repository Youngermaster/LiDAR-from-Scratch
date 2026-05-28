"""
Scan source abstractions.

The mapper does not care whether scans come from a real sensor or
from a CSV file recorded earlier. Both sources expose the same
iterator interface: yielding one full revolution at a time as a
NumPy Nx2 array of (x, y) points in metres, in the sensor body frame.

This split keeps the rest of the pipeline easy to test (just feed it
canned CSVs) and easy to demo on a flight or at a coffee shop with
the LiDAR at home.
"""

from __future__ import annotations

import csv
import glob
import platform
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator, Optional

import numpy as np

from .driver import DEFAULT_MOTOR_PWM, RPLidarC1
from .geometry import polar_to_cartesian_mm


# Schema for recorded CSVs. Must match python/experiments/08_save_scan_to_csv.py
# so a recording made here is replayable by either project.
RECORDING_CSV_HEADER = [
    "timestamp_unix_s", "scan_index", "angle_deg",
    "distance_mm", "quality", "start_flag",
]


@dataclass
class ScanRevolution:
    """One full LiDAR revolution.

    Attributes
    ----------
    points_body_m : (N, 2) float64
        XY of each valid hit in metres, in the sensor body frame.
        Zero-distance ("no return") samples are filtered out.
    qualities : (N,) uint8
        Per-point quality flag from the C1 (0..63).
    angles_deg : (N,) float64
        Original angle in degrees (kept for the distance dashboard).
    distances_mm : (N,) float64
        Original distance in millimetres.
    """

    points_body_m: np.ndarray
    qualities: np.ndarray
    angles_deg: np.ndarray
    distances_mm: np.ndarray

    def __len__(self) -> int:
        return int(self.points_body_m.shape[0])


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


class LiveLidarSource:
    """Yield revolutions from a connected RPLIDAR C1.

    Use as a context manager. Stopping iteration (returning from the
    `for` loop or letting an exception propagate) is the signal to
    stop the motor, close the optional recording file, and release
    the serial port.

    Optional recording: if `record_path` is set, every sample seen
    on the wire is written to that CSV using the same schema as
    `python/experiments/08_save_scan_to_csv.py`. Replays from either
    project can read the result.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 460800,
        motor_pwm: int = DEFAULT_MOTOR_PWM,
        timeout: float = 2.0,
        record_path: Optional[Path] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.motor_pwm = motor_pwm
        self.timeout = timeout
        self.record_path = record_path
        self._lidar: Optional[RPLidarC1] = None
        self._record_fh: Optional[Any] = None
        self._record_writer: Optional[Any] = None

    def __enter__(self) -> "LiveLidarSource":
        self._lidar = RPLidarC1(
            port=self.port, baudrate=self.baudrate, timeout=self.timeout
        )
        self._lidar.set_motor_pwm(self.motor_pwm)
        if self.record_path is not None:
            self.record_path.parent.mkdir(parents=True, exist_ok=True)
            self._record_fh = self.record_path.open("w", newline="")
            self._record_writer = csv.writer(self._record_fh)
            self._record_writer.writerow(RECORDING_CSV_HEADER)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._record_fh is not None:
            self._record_fh.close()
            self._record_fh = None
            self._record_writer = None
        if self._lidar is not None:
            self._lidar.close()
            self._lidar = None

    def iter_revolutions(self) -> Iterator[ScanRevolution]:
        if self._lidar is None:
            raise RuntimeError("LiveLidarSource must be entered as a context manager")

        angles: list[float] = []
        distances: list[float] = []
        qualities: list[int] = []
        seen_start = False
        scan_index = -1

        for sample in self._lidar.iter_scans():
            if sample.start_flag:
                if seen_start and angles:
                    yield _pack_revolution(angles, distances, qualities)
                    angles.clear()
                    distances.clear()
                    qualities.clear()
                seen_start = True
                scan_index += 1
            if not seen_start:
                # Skip the partial fragment before the first start_flag.
                continue
            if self._record_writer is not None:
                self._record_writer.writerow([
                    f"{time.time():.6f}",
                    scan_index,
                    f"{sample.angle:.4f}",
                    f"{sample.distance:.2f}",
                    int(sample.quality),
                    int(bool(sample.start_flag)),
                ])
            if sample.distance > 0.0:
                angles.append(sample.angle)
                distances.append(sample.distance)
                qualities.append(sample.quality)
        if angles:
            yield _pack_revolution(angles, distances, qualities)


class CsvReplaySource:
    """Yield revolutions from a CSV produced by python/experiments/08_save_scan_to_csv.py.

    The CSV is the contract; if its schema ever changes both this
    and that experiment must be updated together.
    """

    def __init__(self, csv_path: Path) -> None:
        self.csv_path = Path(csv_path)
        if not self.csv_path.exists():
            raise FileNotFoundError(self.csv_path)

    def __enter__(self) -> "CsvReplaySource":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        pass

    def iter_revolutions(self) -> Iterator[ScanRevolution]:
        current_idx: Optional[int] = None
        angles: list[float] = []
        distances: list[float] = []
        qualities: list[int] = []

        with self.csv_path.open() as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                scan_idx = int(row["scan_index"])
                angle = float(row["angle_deg"])
                distance = float(row["distance_mm"])
                quality = int(row["quality"])
                if current_idx is None:
                    current_idx = scan_idx
                if scan_idx != current_idx:
                    if angles:
                        yield _pack_revolution(angles, distances, qualities)
                    angles.clear()
                    distances.clear()
                    qualities.clear()
                    current_idx = scan_idx
                if distance > 0.0:
                    angles.append(angle)
                    distances.append(distance)
                    qualities.append(quality)
        if angles:
            yield _pack_revolution(angles, distances, qualities)


def _pack_revolution(
    angles: list[float],
    distances: list[float],
    qualities: list[int],
) -> ScanRevolution:
    angles_arr = np.array(angles, dtype=np.float64)
    distances_arr = np.array(distances, dtype=np.float64)
    qualities_arr = np.array(qualities, dtype=np.uint8)
    points = polar_to_cartesian_mm(angles_arr, distances_arr)
    return ScanRevolution(
        points_body_m=points,
        qualities=qualities_arr,
        angles_deg=angles_arr,
        distances_mm=distances_arr,
    )
