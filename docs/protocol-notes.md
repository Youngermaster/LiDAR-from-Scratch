# RPLIDAR serial protocol notes

These are personal notes built up while reading the SLAMTEC RPLIDAR Protocol
and Application Notes document and observing the sensor's behavior on a wire
analyzer. They are not a copy of the official spec and are not authoritative.
The official documentation is the source of truth.

The intent here is to summarize the parts you need to understand to make
sense of what `pyrplidar` and the official C++ SDK are actually doing under
the hood.

## Frame format

All host-to-sensor commands and sensor-to-host responses are wrapped in a
small framing structure. The exact bytes depend on the command, but the
shape is consistent.

### Request frame

```
+---------+---------+----------+---------------+---------+
| 0xA5    | Command | (Length) | (Payload ...) | (Chksum)|
+---------+---------+----------+---------------+---------+
  1 byte    1 byte    optional   optional         optional
```

- `0xA5` is the start byte. Every request starts with it.
- The command byte selects what to do (GET_INFO, GET_HEALTH, START_SCAN,
  STOP, RESET, and so on).
- Length, payload, and checksum are only present for commands that carry
  parameters (for example express scan modes).

### Response frame (descriptor)

For commands that return data, the sensor first emits a 7-byte response
descriptor and then the data itself:

```
+------+------+--------------+------+------+
| 0xA5 | 0x5A | Data length  | Mode | Type |
+------+------+--------------+------+------+
  1B     1B    4 bytes (LE)    1B     1B
```

- `0xA5 0x5A` is the response start marker (different from the request's
  `0xA5`).
- The 4 length bytes are little-endian and pack two fields together: 30
  bits of data length and 2 bits of "send mode" (single response or
  multiple-response stream).
- The mode byte tells you whether the data is a single answer (INFO, HEALTH)
  or a stream (SCAN).
- The type byte identifies what is being streamed (legacy scan node, express
  scan node, ultra-capsuled, and so on).

After the descriptor, the sensor either sends one fixed-size payload (for
single-response commands) or starts streaming until you send `STOP`.

## Commands the experiments care about

| Command          | Code  | Purpose                                              |
| ---------------- | ----- | ---------------------------------------------------- |
| STOP             | 0x25  | Halt any in-progress scan stream. Always safe.       |
| RESET            | 0x40  | Reset the sensor. Useful after a protocol mismatch.  |
| GET_INFO         | 0x50  | Model, firmware version, hardware version, serial.   |
| GET_HEALTH       | 0x52  | Status flag and error code, if any.                  |
| GET_SAMPLERATE   | 0x59  | Microseconds per sample in standard/express modes.   |
| START_SCAN       | 0x20  | Legacy scan, returns one node per sample at 4 kHz.   |
| EXPRESS_SCAN     | 0x82  | Higher-throughput packed scan format.                |
| SET_MOTOR_PWM    | (DTR) | On some adapters, motor speed control via DTR pin.   |

On the C1 specifically:

- The legacy `START_SCAN` works and is what you want for understanding the
  data flow first.
- The faster mode is `EXPRESS_SCAN` with the appropriate scan mode ID, which
  you can list via `GET_SCAN_MODES` (a separate command, 0xA8). Pick the
  highest sample-rate mode the sensor reports.
- `SET_MOTOR_PWM` is implemented as a DTR signal on the C1 adapter, not as
  a normal protocol command. Libraries hide this difference behind a
  uniform API.

## Scan data: legacy format

Each legacy scan node is 5 bytes:

```
byte 0:  bit 0       start_flag (1 = first sample of a new revolution)
         bit 1       inverse_start_flag (must be NOT start_flag, sanity)
         bits 2-7    quality (0..63)
byte 1:  bit 0       check_bit (must be 1)
         bits 1-7    angle_q6 low 7 bits
byte 2:  bits 0-7    angle_q6 high 8 bits     (angle_q6 = angle_deg * 64)
byte 3:  distance_q2 low 8 bits                (distance_q2 = distance_mm * 4)
byte 4:  distance_q2 high 8 bits
```

To recover human-readable values:

- `quality = byte0 >> 2`
- `angle_deg = ((byte2 << 7) | (byte1 >> 1)) / 64.0`
- `distance_mm = ((byte4 << 8) | byte3) / 4.0`
- `start_flag = bool(byte0 & 0x01)`

A `distance_mm` of `0.0` means the sample was invalid (out of range, or no
return at all). Treat zero distance as "no obstacle in that direction at
this rotation" rather than "obstacle at the origin".

The `start_flag` bit is how you know one full revolution has ended and the
next has begun. Group samples between consecutive start_flag samples to
get a "scan" (a full 360-degree revolution worth of measurements).

## Scan data: express format (overview)

Express scan packs several samples into each frame to use the serial
bandwidth more efficiently. The packing involves an angle delta against a
base angle, and a "cabin" structure that interleaves distance and angle
correction values for two samples per cabin.

You almost never need to decode this by hand. `pyrplidar.start_scan_express`
and the official SDK do it for you. The reason to understand it exists is so
the higher per-second rates that the C1 advertises (compared with the
legacy A1) are not surprising: the data path is denser, not just faster.

## Why the experiments use a library

You can implement everything above in pure `pyserial` and the result is
about 200 lines of bit-banging. We will not. The high-level libraries
(`pyrplidar` for Python, `rplidar_sdk` for C++) are battle-tested across
many sensor models and revisions, and reimplementing the wire format would
distract from the actual lesson, which is what a 2D LiDAR is and what you
do with its output.

If you want to study the wire format, set aside an evening with the
official SLAMTEC application note and a USB-serial sniffer. It is a
worthwhile exercise but it is not on the main path of this repo.
