# CLAUDE.md

This file provides guidance to Claude Code when working in this repository.

## Project Overview

**Name:** Lidar from Scratch (working title)
**Goal:** A hands-on learning and validation repository for the SLAMTEC RPLIDAR C1 2D
LIDAR sensor. The repo contains small, focused programs that incrementally explore the
sensor: connectivity checks, raw scan reading, real-time visualization, distance
measurements, basic spatial reasoning, and eventual integration into larger robotics
projects (SLAM, autonomous navigation).

This is intentionally a **multi-language playground**. The same conceptual experiments
are implemented in Python (for fast iteration and learning), C++ (the official SDK and
the path toward ROS 2 and embedded work), and optionally Rust (for the
systems-programming track). Each language lives in its own directory and is fully
self-contained.

**Audience:** A developer transitioning from full-stack web (React/Nest) into robotics
and embedded systems. The code should be readable as both a working tool AND a
learning resource. Heavy commenting and documentation are explicit goals, not
optional polish.

**Non-goals:** This is not a production library. It is not a wrapper to be installed
as a package. It is not an attempt to compete with `rplidar_sdk` or `rplidar_ros`.
It is a curated set of experiments that build understanding.

## Hardware Context

- **Sensor:** SLAMTEC RPLIDAR C1 (DToF, 12 m range, 360 degrees, 10 Hz, 5 kHz sampling)
- **Connection:** USB via the included adapter board (USB-to-serial bridge,
  typically CP210x or CH340)
- **Host development machine:** macOS on Apple Silicon (M3 Pro)
- **Target deployment machines (future):** Raspberry Pi 4 (Ubuntu 24.04 + ROS 2 Jazzy),
  NVIDIA Jetson (older Nano for AI inference)

On macOS, the sensor appears as `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`.
On Linux, it appears as `/dev/ttyUSB*`.

The C1 default baud rate is **460800**.

## Repository Structure

```
lidar-from-scratch/
  README.md                  # Public-facing intro, quickstart, gallery
  CLAUDE.md                  # This file
  LICENSE                    # MIT (default)
  .gitignore                 # Python, C++, Rust, IDE artifacts
  docs/                      # Cross-cutting documentation
    hardware-setup.md        # Wiring, drivers, OS-specific notes
    protocol-notes.md        # Notes on RPLIDAR serial protocol (learned, not copied)
    glossary.md              # ToF, scan, frame, point cloud, etc.
  python/                    # Python experiments (start here)
    README.md
    requirements.txt
    pyproject.toml           # Optional, if we adopt uv/poetry later
    experiments/
      01_hello_lidar.py
      02_health_and_info.py
      03_basic_scan_print.py
      04_polar_to_cartesian.py
      05_realtime_viz_matplotlib.py
      06_distance_at_angle.py
      07_obstacle_detector.py
      08_save_scan_to_csv.py
      09_replay_recorded_scan.py
      10_3d_pseudo_visualization.py
    lib/                     # Shared helpers ONLY if duplication appears
      __init__.py
      lidar_connection.py
      geometry.py
  cpp/                       # C++ experiments (after Python works)
    README.md
    CMakeLists.txt           # Top-level, adds each experiment as a subdir
    experiments/
      01_hello_sdk/
        CMakeLists.txt
        main.cpp
      02_scan_print/
      03_distance_at_angle/
      04_obstacle_detector/
      ...
    third_party/
      rplidar_sdk/           # Git submodule, not vendored copy
  rust/                      # Rust experiments (optional track)
    README.md
    Cargo.toml               # Workspace
    experiments/
      01_hello_serial/
      02_scan_print/
      ...
  data/                      # Recorded scans for offline experiments
    sample_room_01.csv
    sample_corridor_01.csv
  scripts/                   # Shell helpers
    detect_lidar_port.sh
    install_macos_deps.sh
    install_linux_deps.sh
```

## Working Principles for Claude

When asked to add, modify, or refactor code in this repo, follow these rules in
priority order.

### 1. Each experiment is a standalone, runnable file

Every program under `experiments/` must be runnable on its own with a single command
documented at the top of the file. Do not introduce cross-experiment imports unless
the user explicitly asks for shared code. Duplication is acceptable and often
preferable here, because each file is meant to be read end-to-end as a lesson.

If shared code appears in three or more experiments, propose moving it to `lib/`
before doing so unilaterally.

### 2. Heavy, intentional documentation

Each experiment file must include, at the top:

- A docstring or header comment explaining: what the program does, what concept it
  teaches, what hardware behavior to expect, what success looks like, what failure
  modes are common.
- The exact command to run it.
- The expected output format (with a short example).

Inline comments should explain **why**, not just what. Assume the reader is a
strong programmer learning a new domain.

### 3. No emojis, anywhere

Not in code, not in comments, not in docs, not in commit messages, not in README.
This rule is absolute.

### 4. English only

All identifiers, comments, docstrings, commit messages, and documentation are in
English. This is for portfolio visibility.

### 5. Defensive and explicit error handling

When dealing with serial ports and hardware:

- Always check that the port exists before connecting.
- Always stop the motor and disconnect cleanly on exit (including on exceptions and
  Ctrl+C).
- Print actionable error messages. "Port not found at /dev/cu.usbserial-XXXX. Run
  `ls /dev/cu.*` to list available ports."
- Never silently swallow exceptions.

### 6. Modularity through directories, not through over-engineering

Each language directory (`python/`, `cpp/`, `rust/`) is independent. Do not create
abstractions that span languages. Do not introduce package managers, build
orchestrators, or meta-tools that try to unify them. The unifying layer is the
documentation, not the build system.

### 7. Minimal dependencies

For Python: prefer the standard library plus a small set (`pyserial`,
`pyrplidar` OR direct `pyserial`, `numpy`, `matplotlib`). Do not add web frameworks,
data science stacks, or ML libraries unless an experiment explicitly requires them.

For C++: use the official `rplidar_sdk` as a git submodule. Avoid pulling in Boost,
Qt, or other heavy frameworks unless justified by a specific experiment.

For Rust: use `serialport` and minimal crates. Prefer std where possible.

### 8. macOS-first, Linux-friendly

The primary development environment is macOS on Apple Silicon. All Python and C++
experiments must run on macOS. They should also run on Linux (Ubuntu 24.04) with
at most a port path change. Do not introduce code paths that only work on one OS
without a clear justification and a fallback.

### 9. Visualization is part of the lesson

For any experiment that produces spatial data (scans, distances, points), pair it
with a visualization where feasible. A program that prints numbers is half a
lesson. A program that prints numbers AND draws the room in a matplotlib window is
a full lesson. Visualizations should be live where possible and animated cleanly
(no flicker, no memory leaks across frames).

## Experiment Catalog (intended scope)

These are the experiments the repo aims to contain. Implement them in this order,
in Python first, then port the most instructive ones to C++.

### Python track (primary)

1. **hello_lidar** — Detect the port, connect, print sensor info, disconnect cleanly.
   Teaches: USB-serial basics, the sensor handshake, clean shutdown.
2. **health_and_info** — Query and pretty-print firmware version, model, serial, and
   health state. Teaches: SDK metadata, status interpretation.
3. **basic_scan_print** — Start the motor, request scans, print N points with
   (quality, angle, distance), stop cleanly. Teaches: the scan data structure.
4. **polar_to_cartesian** — Convert each (angle, distance) point to (x, y) in a
   chosen frame. Print and explain the math. Teaches: coordinate frames, the C1
   reference frame, basic trigonometry in robotics.
5. **realtime_viz_matplotlib** — Live 2D plot of the scan in cartesian space. The
   sensor at the origin, points around it, refreshing at sensor rate. Teaches:
   real-time visualization, frame conventions, decimation for performance.
6. **distance_at_angle** — Given an angle (e.g. "directly ahead"), report the live
   distance. Teaches: filtering scan data, angular tolerance, smoothing.
7. **obstacle_detector** — Print a warning when any point within a configurable
   angular sector falls below a threshold distance. Teaches: spatial reasoning,
   the basis of reactive obstacle avoidance.
8. **save_scan_to_csv** — Capture N scans and write them to a CSV (timestamp, angle,
   distance, quality). Teaches: data persistence, reproducibility, building the
   `data/` corpus for offline experiments.
9. **replay_recorded_scan** — Load a CSV from `data/` and replay it through the
   visualization at original or accelerated speed. Teaches: separation of capture
   and processing, the value of recorded data for development without the sensor
   plugged in.
10. **3d_pseudo_visualization** — Stack multiple 2D scans over time on a Z axis (time
    or height) and render with matplotlib 3D or Open3D. The C1 is a 2D sensor, so
    "real" 3D requires a tilt mechanism. This experiment shows what a 2D LIDAR can
    and cannot do dimensionally. Teaches: the limits of 2D sensing, the case for
    depth cameras or 3D LIDAR later.

### C++ track (secondary, follows Python)

The C++ track ports the most instructive Python experiments using the official
`rplidar_sdk` (submodule under `cpp/third_party/`). Each C++ experiment is a small
CMake project under `cpp/experiments/NN_name/`.

Start with: hello_sdk, scan_print, distance_at_angle, obstacle_detector. Defer
visualization in C++ unless the user explicitly asks for it (matplotlib in C++ is
not productive; the lesson is the sensor I/O, not the plotting).

### Rust track (optional, if time permits)

Mirror the C++ track using a Rust serial crate. The goal is to learn ownership and
async I/O in the context of a real device, not to produce a polished library. Two
or three experiments are enough.

## Code Style

### Python

- Python 3.11+ (matches default on recent macOS toolchains).
- Type hints on all function signatures.
- Use `argparse` for CLI arguments. Never hardcode the serial port; default to
  None and auto-detect, with `--port` as override.
- Use `pathlib.Path` for filesystem work.
- Format with `black`, lint with `ruff` (if a linter is set up). No manual style
  arguments.
- Top-level guard: `if __name__ == "__main__":`.

### C++

- C++17 minimum, C++20 acceptable.
- CMake 3.20+.
- Match the style of the `rplidar_sdk` examples for consistency, but improve
  readability with named constants and clear control flow.
- RAII for resources: serial port, motor state, scan iterators. Never leave the
  motor spinning if main exits early.
- Use `std::format` if C++20, otherwise `fmt::format` or `printf` (whichever the
  SDK already uses).

### Rust

- Edition 2021.
- `cargo fmt` and `cargo clippy` clean.
- Prefer `?` over `unwrap()` outside of the most trivial demos.

## Common Tasks and Expected Behavior

When asked to do any of the following, behave as described.

### "Add a new Python experiment for X"

1. Create `python/experiments/NN_descriptive_name.py` where NN is the next number.
2. Include the standard header (purpose, run command, expected output, failure
   modes).
3. Implement with argparse, defensive cleanup, and clear comments.
4. Add a one-line entry under "Experiments" in `python/README.md`.
5. Do not modify other experiments unless the user asks.

### "Port experiment N to C++"

1. Create `cpp/experiments/NN_name/` with `main.cpp` and `CMakeLists.txt`.
2. Use the `rplidar_sdk` headers via the submodule.
3. Mirror the structure of the Python version. Keep variable names and comments
   parallel so a reader can compare files side by side and learn.
4. Add the new subdirectory to the top-level `cpp/CMakeLists.txt`.

### "Add a new dependency"

Ask first if the dependency is not in the minimal set declared in this file.
Justify it. Document it in the relevant language README.

### "Make this production-ready"

Decline politely. This repo is intentionally not a production library. Suggest
that the right next step is to take a specific experiment and extract it into a
separate, focused project.

## What to Do When the User Reports a Failure

Before changing code, ask for or check:

1. Output of `ls /dev/cu.*` (macOS) or `ls /dev/ttyUSB*` (Linux).
2. Whether the motor is spinning audibly.
3. The exact error message.
4. The exact command run.

Most failures in this repo are hardware/driver issues (port name wrong, USB cable
flaky, motor PWM not started, baud rate mismatch), not code bugs. Diagnose at the
hardware layer first.

## Out of Scope (for this repo)

- ROS 2 integration. That belongs in a separate repository.
- SLAM algorithms. Use existing implementations (slam_toolbox) in a separate
  robotics repo.
- Multi-robot coordination, Nav2, Open-RMF. Separate concerns, separate repos.
- Web dashboards, mobile apps, cloud sync. Out of scope.
- Performance benchmarking. The repo is for understanding, not for tuning.

If a request would push the repo into one of these areas, suggest creating a new
dedicated repository instead and explain why.
