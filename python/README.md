# Python track

The Python track is the primary entry point for this repo. Each program
under `experiments/` is a single-file lesson: read it top to bottom, run
it, observe the result, then move on to the next one.

## Setup

Python 3.11 or newer is required. Use a per-project virtual environment so
the dependencies do not pollute your system Python.

```bash
cd python
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

On Linux substitute `python3` for `python3.11` if your distro ships 3.11+
as the default.

To leave the venv later:

```bash
deactivate
```

## Running an experiment

All experiments default to auto-detecting the serial port. You can override
with `--port`:

```bash
python experiments/01_hello_lidar.py
python experiments/01_hello_lidar.py --port /dev/cu.usbserial-0001
```

Stop any running experiment with Ctrl+C. The cleanup handlers stop the
motor before exiting; if you hear the rotor still spinning after a clean
exit, that is a bug worth reporting.

## Experiments

| #  | File                                | Teaches                                              |
| -- | ----------------------------------- | ---------------------------------------------------- |
| 01 | `01_hello_lidar.py`                 | Auto-detect port, connect, print info, disconnect.   |
| 02 | `02_health_and_info.py`             | Pretty-printed firmware, model, serial, health.      |
| 03 | `03_basic_scan_print.py`            | Start motor, stream scan tuples, stop cleanly.       |
| 04 | `04_polar_to_cartesian.py`          | Convert (angle, distance) to (x, y) with explanation.|
| 05 | `05_realtime_viz_matplotlib.py`     | Live 2D plot of the scan, sensor at origin.          |
| 06 | `06_distance_at_angle.py`           | Live distance at a chosen bearing, with smoothing.   |
| 07 | `07_obstacle_detector.py`           | Warn when an angular sector goes below a threshold.  |
| 08 | `08_save_scan_to_csv.py`            | Record N scans to a CSV under `data/`.               |
| 09 | `09_replay_recorded_scan.py`        | Replay a CSV through a live plot. No hardware needed.|
| 10 | `10_3d_pseudo_visualization.py`     | Stack 2D scans on a time/Z axis as a pseudo-3D cloud.|

## Conventions used in every file

- Standard header comment with: purpose, run command, expected output,
  failure modes.
- `argparse` for CLI arguments. `--port` is always optional; auto-detect
  is the default.
- Every experiment uses `with RPLidarC1(...) as lidar:` so the motor
  is always stopped and the port released, including on Ctrl+C and
  exceptions.
- Type hints on all function signatures.

## The `lib/` package

`lib/rplidar_c1.py` is a small, from-scratch RPLIDAR C1 driver built on
`pyserial`. Per CLAUDE.md the `lib/` directory is reserved for code
that is genuinely shared by three or more experiments; the driver
qualifies (every experiment that talks to the sensor uses it).

We do not depend on the community `pyrplidar` package because its
decoder is broken for the C1's firmware. See the comment block in
`requirements.txt` and the docstring at the top of `lib/rplidar_c1.py`
for the rationale. The protocol it implements is documented in
[`../docs/protocol-notes.md`](../docs/protocol-notes.md).

## Common failures and what they mean

- **"Port not found"**: the OS does not see the adapter. Run
  `../scripts/detect_lidar_port.sh`. Try a different USB cable. On Linux,
  confirm your user is in the `dialout` group.
- **"Operation timed out" / "no data"**: you connected but the sensor is
  not producing samples. The motor probably did not start. Listen for the
  rotor spinning up after the experiment prints "Starting motor".
- **`PermissionError: /dev/ttyUSB0`**: on Linux, see the previous bullet
  about `dialout`.
- **Garbled output / wrong-length frames**: most likely a baud-rate
  mismatch. The C1 default is 460800. Confirm with
  `python experiments/02_health_and_info.py`, which prints the rate it
  used.
