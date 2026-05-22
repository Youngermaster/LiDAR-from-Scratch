#!/usr/bin/env bash
# =====================================================================
# detect_lidar_port.sh
#
# Purpose:
#   Find the most likely serial port for a connected RPLIDAR C1 on either
#   macOS or Linux, and print a one-line, pasteable port path on stdout.
#   If no candidate is found, print a short diagnostic to stderr and exit
#   with a non-zero status.
#
# Usage:
#   ./scripts/detect_lidar_port.sh
#   PORT=$(./scripts/detect_lidar_port.sh) && echo "Found: $PORT"
#
# Notes:
#   - On macOS, prefers /dev/cu.usbserial-* and /dev/cu.SLAB_USBtoUART
#     over /dev/tty.* entries. Always use /dev/cu.* for this device.
#   - On Linux, looks at /dev/ttyUSB* (CP210x or CH340 attach there).
#   - This script does NOT open the port. It only inspects /dev. The
#     experiments themselves verify connectivity via the protocol.
# =====================================================================

set -u

uname_s="$(uname -s)"

print_macos_diagnostic() {
    {
        echo "No RPLIDAR-like serial port found under /dev/cu.*"
        echo
        echo "Visible /dev/cu.* entries:"
        # Listing is informational. We intentionally use ls here.
        ls -1 /dev/cu.* 2>/dev/null || echo "  (none)"
        echo
        echo "Common candidates to look for:"
        echo "  /dev/cu.usbserial-XXXX   (CP210x or CH340 with current driver)"
        echo "  /dev/cu.SLAB_USBtoUART   (older Silicon Labs driver)"
        echo
        echo "Hints:"
        echo "  - On Apple Silicon: check System Settings -> Privacy & Security"
        echo "    -> 'Allow accessories to connect'. New USB devices are blocked"
        echo "    until allowed; the default 'Ask' prompt is easy to miss when"
        echo "    plugging through a hub. Set to 'Always' temporarily and replug"
        echo "    DIRECTLY into the Mac. See docs/hardware-setup.md."
        echo "  - Replug the USB cable. Use a known data-capable cable."
        echo "  - If the LED on the adapter board does not light, the cable"
        echo "    is most likely charge-only."
        echo "  - For CH340-based adapters, install the WCH driver:"
        echo "    https://www.wch-ic.com/downloads/CH34XSER_MAC_ZIP.html"
    } 1>&2
}

print_linux_diagnostic() {
    {
        echo "No RPLIDAR-like serial port found under /dev/ttyUSB*"
        echo
        echo "Visible /dev/ttyUSB* entries:"
        ls -1 /dev/ttyUSB* 2>/dev/null || echo "  (none)"
        echo
        echo "Hints:"
        echo "  - Run 'dmesg | tail' and look for 'cp210x' or 'ch341'."
        echo "  - Permission denied? Add yourself to the dialout group:"
        echo "      sudo usermod -aG dialout \$USER"
        echo "    then log out and back in. Arch uses 'uucp' instead."
        echo "  - ModemManager can grab USB-serial devices briefly on connect."
        echo "    If the first scan fails right after plug-in, consider:"
        echo "      sudo apt remove modemmanager"
        echo "  - Full troubleshooting: docs/hardware-setup.md."
    } 1>&2
}

if [[ "$uname_s" == "Darwin" ]]; then
    # macOS: prefer cu.usbserial-* first, then SLAB_USBtoUART.
    # The first matching device wins. We deliberately avoid /dev/tty.*.
    candidates=(/dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART)
    for candidate in "${candidates[@]}"; do
        if [[ -e "$candidate" ]]; then
            echo "$candidate"
            exit 0
        fi
    done
    print_macos_diagnostic
    exit 1

elif [[ "$uname_s" == "Linux" ]]; then
    # Linux: ttyUSB* is where both cp210x and ch341 attach.
    candidates=(/dev/ttyUSB*)
    for candidate in "${candidates[@]}"; do
        if [[ -e "$candidate" ]]; then
            echo "$candidate"
            exit 0
        fi
    done
    print_linux_diagnostic
    exit 1

else
    echo "Unsupported platform: $uname_s" 1>&2
    echo "Supported: Darwin (macOS), Linux." 1>&2
    exit 2
fi
