# Rust track (optional)

The Rust track is the smallest of the three. The goal is to learn
ownership, error handling, and async I/O in the context of a real
piece of hardware. It is not a polished library.

Only the first few experiments are ported here. Skip this track
entirely if you have not yet completed the Python and C++ tracks.

## Prerequisites

Install Rust via `rustup`:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
rustup default stable
```

Confirm:

```bash
rustc --version
cargo --version
```

## Build and run

The Rust track is a Cargo workspace. From this directory:

```bash
cargo build
cargo run --bin hello_serial
cargo run --bin hello_serial -- --port /dev/cu.usbserial-0001
```

## Experiments

| #  | Crate / bin                    | Mirror of the Python lesson    |
| -- | ------------------------------ | ------------------------------ |
| 01 | `experiments/01_hello_serial`  | `01_hello_lidar.py` (partial). Lists candidate ports and prints what it finds. Does not yet implement the RPLIDAR protocol. |

## Conventions

- Edition 2021.
- `cargo fmt` and `cargo clippy` clean.
- Prefer `?` over `.unwrap()` outside of trivial demos.
- Use `serialport` for the serial transport. Avoid pulling in async
  runtimes (tokio, async-std) until an experiment actually needs them.

## Why this track is small

Implementing the RPLIDAR protocol in Rust from scratch is a worthwhile
exercise but it is a different scope than the rest of this repo. If you
want a production-quality Rust driver, look at the community crates
listed in the workspace `Cargo.toml`'s comment block, or extract this
work into a focused repository of its own.
