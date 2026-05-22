# C++ track

The C++ track ports the most instructive Python experiments using the
official SLAMTEC `rplidar_sdk`. The SDK is vendored as a git submodule
under `third_party/rplidar_sdk/`, never copied in.

The C++ track is the realistic path to ROS 2 nodes, microcontroller
ports, and the embedded targets listed in CLAUDE.md (Raspberry Pi 4,
NVIDIA Jetson). Visualization in C++ is deliberately deferred; the
lesson here is the sensor I/O, not the plotting.

## Prerequisites

- CMake 3.20+
- A C++17 compiler (Apple Clang 14+, GCC 11+, MSVC 2022+)
- git

On macOS, run `./scripts/install_macos_deps.sh` from the repo root.
On Ubuntu 24.04, run `./scripts/install_linux_deps.sh`.

## Get the SDK

The first time you check this repo out, fetch the SDK submodule:

```bash
git submodule update --init --recursive
```

If the submodule is missing (which it is in a fresh repo before anyone
has added it), add it once:

```bash
git submodule add https://github.com/Slamtec/rplidar_sdk \
    cpp/third_party/rplidar_sdk
git submodule update --init --recursive
```

The SDK builds itself with its own makefiles; the top-level CMakeLists
in this directory wires it in via `add_subdirectory` so we get a single
build tree.

## Build

```bash
cd cpp
cmake -S . -B build
cmake --build build --parallel
```

The binaries land in `build/experiments/NN_name/`. Run one:

```bash
./build/experiments/01_hello_sdk/hello_sdk
./build/experiments/01_hello_sdk/hello_sdk /dev/cu.usbserial-0001 460800
```

## Experiments

| #  | Directory                         | Mirror of the Python lesson    |
| -- | --------------------------------- | ------------------------------ |
| 01 | `experiments/01_hello_sdk/`       | `01_hello_lidar.py`            |
| 02 | `experiments/02_scan_print/`      | `03_basic_scan_print.py`       |

Each C++ experiment uses variable names and comments parallel to its
Python counterpart so the two files can be read side by side.

## Conventions

- RAII for resources. The serial port, the motor state, and the SDK
  driver handle are wrapped so that early returns and exceptions still
  stop the motor and free the driver.
- C++17 by default. Use C++20 only when it earns its keep.
- No Boost, no Qt. The SDK is enough.
- Output goes to stdout. Errors and diagnostics go to stderr.

## Why no visualization here

Matplotlib has no real C++ equivalent that is worth pulling in for a
learning repo. If you want a live plot, run experiment 05 in Python
against the same sensor or replay a CSV recorded with experiment 08.
Mixing C++ scan reading with Python visualization through a CSV
boundary is, frankly, the cleanest split for a teaching repository.
