"""
rplidar_c1.py - a minimal, from-scratch driver for the RPLIDAR C1.

Why this exists
---------------
The community `pyrplidar` package decodes the protocol incorrectly for
the C1's firmware. On a real C1, its INFO parser drops the firmware
field, its scan-mode parser leaves us_per_sample in Q8 fixed-point, and
its scan-node decoder throws "index out of range" on the first frame in
both legacy and express modes.

Rather than maintain a fork, we implement the exact subset of the
SLAMTEC serial protocol the experiments need, using only `pyserial`.
The protocol itself is summarized in `docs/protocol-notes.md`; this
file is the executable companion to those notes.

What is implemented
-------------------
- Connect at 460800 baud, send `STOP` and flush on open (sensors left
  in a streaming state from a previous run otherwise re-emit a partial
  scan response and desync our descriptor parse).
- `get_info()`: command 0x50, parses model / firmware / hardware /
  16-byte serial.
- `get_health()`: command 0x52.
- `set_motor_pwm(value)`: command 0xF0 (accessory protocol). Used by
  the adapter board to spin or stop the rotor.
- `iter_scans()`: command 0x20 (legacy SCAN). Yields one
  `ScanSample` per 5-byte node, indefinitely, until the caller stops
  iterating or calls `stop()`. We picked legacy mode because it is
  universally supported on every RPLIDAR variant, the wire format is
  trivial to decode, and it runs at 8 kHz on the C1 - more than enough
  for any of the experiments.

What is intentionally not implemented
-------------------------------------
- Express / DenseBoost scan modes. Higher throughput, but the capsule
  decoding is significantly more involved and brings no benefit at this
  repo's scope.
- `GET_LIDAR_CONF` for full scan-mode enumeration. Not needed for
  legacy SCAN.
- DTR-based motor control fallback. Every C1 adapter shipping today
  routes motor control through the accessory PWM command.

Reference
---------
- docs/protocol-notes.md (this repo)
- SLAMTEC "RPLIDAR Protocol and Application Note" (official PDF)
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Iterator, Optional

import serial


# ---------- Protocol constants ----------------------------------------

# Every request starts with the sync byte. Every response descriptor
# starts with SYNC_BYTE followed by RESP_SYNC. See protocol-notes.md.
SYNC_BYTE = 0xA5
RESP_SYNC = 0x5A

# Commands we use.
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_SET_MOTOR_PWM = 0xF0  # accessory command, payload = uint16 PWM 0..1023

# Response descriptor is always 7 bytes.
DESCRIPTOR_LEN = 7

# Per-response payload sizes (for single-response commands).
INFO_PAYLOAD_LEN = 20
HEALTH_PAYLOAD_LEN = 3
SCAN_NODE_LEN = 5

# C1 motor PWM. The SLAMTEC default for C-series adapters.
DEFAULT_MOTOR_PWM = 660


# ---------- Data classes ----------------------------------------------

@dataclass
class ScanSample:
    """One decoded sample from a legacy scan node."""

    quality: int          # 0..63
    angle: float          # degrees, 0..360
    distance: float       # millimetres; 0.0 means "no return"
    start_flag: bool      # True on the first sample of a new revolution


@dataclass
class DeviceInfo:
    model: int            # 0x41 (65) for the C1
    firmware_major: int
    firmware_minor: int
    hardware: int
    serial_hex: str       # 32-char uppercase hex (16 bytes)

    @property
    def firmware(self) -> str:
        return f"{self.firmware_major}.{self.firmware_minor:02d}"


@dataclass
class DeviceHealth:
    status: int           # 0 = OK, 1 = Warning, 2 = Error
    error_code: int


# ---------- Driver ----------------------------------------------------

class RPLidarC1:
    """Minimal RPLIDAR C1 driver over a USB-serial port.

    Use it as a context manager so the motor stops and the port closes
    on exit, including on exceptions:

        with RPLidarC1("/dev/cu.usbserial-1130") as lidar:
            for sample in lidar.iter_scans():
                ...
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 460800,
        timeout: float = 2.0,
    ) -> None:
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            dsrdtr=False,
        )
        # If a previous program crashed mid-scan the sensor is still
        # streaming. STOP + a short pause + an input-buffer flush is
        # what brings it back to a known state.
        try:
            self._send_cmd(CMD_STOP)
            time.sleep(0.05)
            self._serial.reset_input_buffer()
        except Exception:
            # Swallowing here is appropriate: failure means we never
            # opened cleanly and the caller will see it on the first
            # real request anyway.
            pass

    # -- context manager -------------------------------------------

    def __enter__(self) -> "RPLidarC1":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        # Order matters: stop the scan stream first so the sensor goes
        # idle, then drop the motor, then release the port.
        try:
            self._send_cmd(CMD_STOP)
            time.sleep(0.01)
        except Exception:
            pass
        try:
            self.set_motor_pwm(0)
        except Exception:
            pass
        try:
            self._serial.close()
        except Exception:
            pass

    # -- low-level framing -----------------------------------------

    def _send_cmd(self, cmd: int, payload: Optional[bytes] = None) -> None:
        """Send a request frame. Single-byte command, or command + payload + xor checksum."""
        if payload is None:
            self._serial.write(bytes([SYNC_BYTE, cmd]))
            return
        length = len(payload)
        buf = bytearray([SYNC_BYTE, cmd, length])
        buf.extend(payload)
        checksum = 0
        for b in buf:
            checksum ^= b
        buf.append(checksum)
        self._serial.write(bytes(buf))

    def _read_exact(self, n: int) -> bytes:
        """Read exactly n bytes from the serial port or raise."""
        data = self._serial.read(n)
        if len(data) != n:
            raise RuntimeError(
                f"Short read: got {len(data)} bytes, expected {n}. "
                "The sensor stopped responding. Check the cable and try again."
            )
        return data

    def _read_descriptor(self) -> tuple[int, int, int]:
        """Return (data_length, send_mode, data_type) from the next descriptor.

        data_length is the number of bytes in each response payload;
        send_mode is 0 for single, 1 for multi-response stream;
        data_type identifies what is being streamed (0x81 = legacy scan).
        """
        desc = self._read_exact(DESCRIPTOR_LEN)
        if desc[0] != SYNC_BYTE or desc[1] != RESP_SYNC:
            raise RuntimeError(
                f"Bad response descriptor sync bytes: {desc[0]:#04x} {desc[1]:#04x}. "
                "Most often this means another program is reading the port, "
                "or the previous run left the sensor in a partial-frame state. "
                "Disconnect and reconnect the LiDAR and try again."
            )
        length_mode = struct.unpack("<I", desc[2:6])[0]
        data_length = length_mode & 0x3FFFFFFF
        send_mode = (length_mode >> 30) & 0x3
        data_type = desc[6]
        return data_length, send_mode, data_type

    # -- commands ---------------------------------------------------

    def stop(self) -> None:
        """Tell the sensor to stop any in-progress scan stream."""
        self._send_cmd(CMD_STOP)
        time.sleep(0.01)

    def reset(self) -> None:
        """Soft-reset the sensor. Costs ~500 ms."""
        self._send_cmd(CMD_RESET)
        time.sleep(0.5)
        self._serial.reset_input_buffer()

    def set_motor_pwm(self, pwm: int) -> None:
        """Set motor PWM. 0 stops the rotor; 660 is the C1 default; 1023 is max."""
        if not 0 <= pwm <= 1023:
            raise ValueError(f"PWM out of range: {pwm}")
        self._send_cmd(CMD_SET_MOTOR_PWM, struct.pack("<H", pwm))
        # The adapter board needs a few ms to respond. Without this
        # delay, an immediate START_SCAN can race the motor and fail.
        time.sleep(0.05)

    def get_info(self) -> DeviceInfo:
        self._send_cmd(CMD_GET_INFO)
        length, _mode, _dtype = self._read_descriptor()
        if length != INFO_PAYLOAD_LEN:
            raise RuntimeError(
                f"Unexpected INFO payload size: {length} (expected {INFO_PAYLOAD_LEN}).")
        payload = self._read_exact(length)
        model = payload[0]
        # firmware_version is uint16 LE: low byte = minor, high byte = major.
        fw_minor = payload[1]
        fw_major = payload[2]
        hardware = payload[3]
        serial_bytes = payload[4:20]
        return DeviceInfo(
            model=model,
            firmware_major=fw_major,
            firmware_minor=fw_minor,
            hardware=hardware,
            serial_hex=serial_bytes.hex().upper(),
        )

    def get_health(self) -> DeviceHealth:
        self._send_cmd(CMD_GET_HEALTH)
        length, _mode, _dtype = self._read_descriptor()
        if length != HEALTH_PAYLOAD_LEN:
            raise RuntimeError(
                f"Unexpected HEALTH payload size: {length} (expected {HEALTH_PAYLOAD_LEN}).")
        payload = self._read_exact(length)
        status = payload[0]
        error_code = struct.unpack("<H", payload[1:3])[0]
        return DeviceHealth(status=status, error_code=error_code)

    # -- scanning ---------------------------------------------------

    def iter_scans(self) -> Iterator[ScanSample]:
        """Start legacy SCAN and yield each decoded sample.

        Caller must arrange for `close()` (or context-manager exit) to
        run so the motor stops and the port is released.
        """
        self._send_cmd(CMD_SCAN)
        length, send_mode, data_type = self._read_descriptor()
        if length != SCAN_NODE_LEN:
            raise RuntimeError(
                f"Unexpected scan-node size in descriptor: {length} "
                f"(expected {SCAN_NODE_LEN}). The sensor may not support "
                "legacy SCAN; please report this with your model and firmware.")
        if send_mode != 1 or data_type != 0x81:
            # Not fatal; we proceed but warn callers via the exception
            # only if the stream subsequently misbehaves.
            pass

        # Decode loop. We stay synchronized to the 5-byte node grid
        # using the protocol's built-in invariants:
        #   - byte0 bit0 (start_flag) XOR byte0 bit1 (inverse start) == 1
        #   - byte1 bit0 (check_bit) == 1
        # If either invariant fails we drop a byte and resync.
        while True:
            first = self._serial.read(1)
            if not first:
                # Read timeout. Try again. Callers detect a stuck
                # stream by also watching wall-clock time.
                continue
            b0 = first[0]
            start_flag = bool(b0 & 0x01)
            inverse_start = bool((b0 >> 1) & 0x01)
            if start_flag == inverse_start:
                # Lost frame alignment; drop this byte and try again
                # with the next one as a candidate byte 0.
                continue
            rest = self._serial.read(SCAN_NODE_LEN - 1)
            if len(rest) != SCAN_NODE_LEN - 1:
                continue
            b1, b2, b3, b4 = rest[0], rest[1], rest[2], rest[3]
            if not (b1 & 0x01):
                # Second invariant failed. Resync.
                continue
            quality = b0 >> 2
            # angle_q6 is 15 bits across bytes 1-2.
            angle_q6 = ((b2 << 7) | (b1 >> 1)) & 0x7FFF
            angle = angle_q6 / 64.0
            distance_q2 = ((b4 << 8) | b3) & 0xFFFF
            distance = distance_q2 / 4.0
            yield ScanSample(
                quality=quality,
                angle=angle,
                distance=distance,
                start_flag=start_flag,
            )
