// =====================================================================
// 01_hello_sdk
// =====================================================================
//
// Purpose
// -------
// The C++ counterpart of python/experiments/01_hello_lidar.py. Open a
// serial port to the RPLIDAR C1 via the official SLAMTEC SDK, query
// device INFO and HEALTH, then disconnect cleanly. No motor, no scan
// stream; this is the protocol-handshake-only sanity check.
//
// What this teaches
// -----------------
// - The shape of an rplidar_sdk session in C++: create a driver, call
//   connect, query, disconnect, dispose.
// - RAII as a substitute for Python's try/finally. We wrap the driver
//   in a struct whose destructor disposes of it, so even an early
//   return or thrown exception cannot leak the resource.
//
// Build
// -----
//     cd cpp
//     cmake -S . -B build
//     cmake --build build --target hello_sdk
//
// Run
// ---
//     ./build/experiments/01_hello_sdk/hello_sdk
//     ./build/experiments/01_hello_sdk/hello_sdk /dev/cu.usbserial-0001 460800
//
// Expected output
// ---------------
//     Port: /dev/cu.usbserial-0001  (baud=460800)
//     Connecting...
//     INFO   model=...  fw=...  hw=...  serial=...
//     HEALTH status=0 (OK)  error_code=0
//     Disconnecting.
//     Done.
// =====================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// SDK headers. The exact include paths follow the layout used by the
// official rplidar_sdk submodule. If the submodule is not initialized
// the build will fail with a clear "file not found" error pointing here.
#include "rplidar.h"  // NOLINT

using rp::standalone::rplidar::RPlidarDriver;

namespace {

// Pick a reasonable default port for each platform. The user can always
// override on the command line.
constexpr const char* default_port() {
#if defined(__APPLE__)
    return "/dev/cu.usbserial-0001";
#elif defined(__linux__)
    return "/dev/ttyUSB0";
#else
    return "COM3";
#endif
}

constexpr uint32_t kDefaultBaudrate = 460800;

// RAII wrapper. CreateDriver returns a raw pointer that the caller owns
// and must release via DisposeDriver. Wrapping it in a struct with a
// destructor ensures we never leak it.
struct DriverHandle {
    RPlidarDriver* driver = nullptr;
    explicit DriverHandle(RPlidarDriver* d) : driver(d) {}
    ~DriverHandle() {
        if (driver) {
            RPlidarDriver::DisposeDriver(driver);
            driver = nullptr;
        }
    }
    DriverHandle(const DriverHandle&) = delete;
    DriverHandle& operator=(const DriverHandle&) = delete;
};

const char* health_status_name(int code) {
    switch (code) {
        case 0: return "OK";
        case 1: return "Warning";
        case 2: return "Error";
        default: return "Unknown";
    }
}

}  // namespace

int main(int argc, char** argv) {
    const char* port = (argc >= 2) ? argv[1] : default_port();
    uint32_t baudrate = kDefaultBaudrate;
    if (argc >= 3) {
        baudrate = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
    }

    std::printf("Port: %s  (baud=%u)\n", port, baudrate);

    DriverHandle handle{RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT)};
    if (!handle.driver) {
        std::fprintf(stderr, "Failed to create RPlidarDriver instance.\n");
        return 1;
    }

    std::printf("Connecting...\n");
    if (IS_FAIL(handle.driver->connect(port, baudrate))) {
        std::fprintf(stderr,
            "Failed to connect on %s at %u baud. "
            "Run scripts/detect_lidar_port.sh and confirm the port.\n",
            port, baudrate);
        return 1;
    }

    rplidar_response_device_info_t info{};
    if (IS_FAIL(handle.driver->getDeviceInfo(info))) {
        std::fprintf(stderr, "getDeviceInfo() failed.\n");
        handle.driver->disconnect();
        return 1;
    }
    // Serial number is a 16-byte array. Pretty-print as hex.
    char serial_hex[33] = {0};
    for (int i = 0; i < 16; ++i) {
        std::snprintf(serial_hex + (i * 2), 3, "%02X", info.serialnum[i]);
    }
    std::printf("INFO   model=%u  fw=%u.%u  hw=%u  serial=%s\n",
        info.model,
        info.firmware_version >> 8, info.firmware_version & 0xFF,
        info.hardware_version,
        serial_hex);

    rplidar_response_device_health_t health{};
    if (IS_FAIL(handle.driver->getHealth(health))) {
        std::fprintf(stderr, "getHealth() failed.\n");
        handle.driver->disconnect();
        return 1;
    }
    std::printf("HEALTH status=%d (%s)  error_code=%d\n",
        health.status, health_status_name(health.status),
        health.error_code);

    std::printf("Disconnecting.\n");
    handle.driver->disconnect();
    std::printf("Done.\n");
    return 0;
}
