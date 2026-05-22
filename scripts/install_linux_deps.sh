#!/usr/bin/env bash
# =====================================================================
# install_linux_deps.sh
#
# Purpose:
#   Install host-side dependencies on Ubuntu 24.04 to run the experiments
#   in this repo. Other Debian-based distros should also work; for
#   non-apt distros adapt the package list.
#
# What this installs:
#   - python3.11 + python3.11-venv (system package on 24.04 is 3.12; we
#     allow 3.12 too, as the code targets 3.11+)
#   - cmake, build-essential
#   - git
#
# What this DOES NOT do:
#   - Add your user to the dialout group. That requires sudo and a
#     re-login. Do it once by hand:
#       sudo usermod -aG dialout "$USER"
#
# Usage:
#   ./scripts/install_linux_deps.sh
# =====================================================================

set -euo pipefail

log() {
    printf '[install-linux] %s\n' "$*"
}

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "This script is for Linux. On macOS, use install_macos_deps.sh." 1>&2
    exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
    echo "This script targets apt-based distros (Ubuntu, Debian)." 1>&2
    echo "On Fedora/Arch, install the equivalent packages by hand." 1>&2
    exit 1
fi

log "Updating apt index"
sudo apt-get update

log "Installing core packages"
sudo apt-get install -y \
    python3 python3-venv python3-pip \
    cmake build-essential \
    git

log "Versions installed:"
python3 --version || true
cmake --version | head -n 1 || true
git --version || true

log "IMPORTANT - serial port permissions:"
log "  Add yourself to the dialout group so you can open /dev/ttyUSB*:"
log "    sudo usermod -aG dialout \"\$USER\""
log "  Then log out and back in for the group change to take effect."
log "  Confirm with: groups | grep dialout"
log ""
log "  Arch and derivatives use the 'uucp' group instead of 'dialout'."
log "  If ModemManager is installed it may briefly grab the LiDAR on"
log "  connect; remove it (sudo apt remove modemmanager) if that"
log "  causes the first scan to fail. See docs/hardware-setup.md."

log "Done. Next steps:"
log "  1) ./scripts/detect_lidar_port.sh"
log "  2) cd python && python3 -m venv .venv && source .venv/bin/activate"
log "  3) pip install -r requirements.txt"
log "  4) python experiments/01_hello_lidar.py"
