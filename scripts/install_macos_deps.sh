#!/usr/bin/env bash
# =====================================================================
# install_macos_deps.sh
#
# Purpose:
#   Install the host-side dependencies needed to run the experiments in
#   this repo on macOS (Apple Silicon or Intel). The script is idempotent
#   and prints what it is doing before doing it.
#
# What this installs:
#   - Homebrew  (if missing)
#   - python@3.11 (or newer) - the experiments require Python 3.11+
#   - cmake     - needed for the C++ track
#   - git       - used for the rplidar_sdk submodule under cpp/third_party
#
# What this DOES NOT install:
#   - Python packages (pyserial, pyrplidar, numpy, matplotlib). Those go
#     in a per-experiment virtual environment under python/.venv. See
#     python/README.md for the venv recipe.
#   - The Rust toolchain. The Rust track is optional; install rustup
#     separately if you want it.
#
# Usage:
#   ./scripts/install_macos_deps.sh
# =====================================================================

set -euo pipefail

log() {
    printf '[install-macos] %s\n' "$*"
}

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "This script is for macOS. On Linux, use install_linux_deps.sh." 1>&2
    exit 1
fi

# ---------- Homebrew ----------------------------------------------------
if ! command -v brew >/dev/null 2>&1; then
    log "Homebrew not found. Installing from the official installer."
    log "You will be prompted for your password."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    # The installer prints a final hint to add brew to PATH on Apple Silicon.
    # Apply it for the rest of this script's lifetime if needed.
    if [[ -x /opt/homebrew/bin/brew ]]; then
        eval "$(/opt/homebrew/bin/brew shellenv)"
    fi
else
    log "Homebrew already installed."
fi

# ---------- Core formulae ----------------------------------------------
formulae=(python@3.11 cmake git)
for f in "${formulae[@]}"; do
    if brew list --versions "$f" >/dev/null 2>&1; then
        log "$f already installed."
    else
        log "Installing $f"
        brew install "$f"
    fi
done

# ---------- Summary ----------------------------------------------------
log "Versions installed:"
python3.11 --version || true
cmake --version | head -n 1 || true
git --version || true

log "Done. Next steps:"
log "  1) ./scripts/detect_lidar_port.sh        # confirm the OS sees the sensor"
log "  2) cd python && python3.11 -m venv .venv && source .venv/bin/activate"
log "  3) pip install -r requirements.txt"
log "  4) python experiments/01_hello_lidar.py"
