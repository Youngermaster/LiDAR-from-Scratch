// =====================================================================
// 02_scan_print
// =====================================================================
//
// Purpose
// -------
// The C++ counterpart of python/experiments/03_basic_scan_print.py.
// Start the motor, request a stream of scan samples, print the first N,
// then stop the motor and disconnect cleanly.
//
// What this teaches
// -----------------
// - The SDK's scan API in legacy mode: startMotor, startScan,
//   grabScanData, stop, stopMotor. Each step has a matching teardown.
// - That a "scan" in the SDK's vocabulary is a full revolution buffered
//   into a fixed-size array of nodes. The host pulls one buffer at a
//   time rather than one sample at a time.
// - Quantized fields. The angle is `angle_q6_checkbit` (degrees << 6
//   plus a check bit) and the distance is `distance_q2` (millimetres
//   << 2). We undo the quantization for display.
//
// Build
// -----
//     cmake --build build --target scan_print
//
// Run
// ---
//     ./build/experiments/02_scan_print/scan_print            # 5 revolutions
//     ./build/experiments/02_scan_print/scan_print /dev/cu.usbserial-0001 460800 20
//
// Expected output
// ---------------
//     Port: /dev/cu.usbserial-0001  (baud=460800)  revolutions=5
//     Starting motor.
//     rev 1: 482 samples (456 valid)
//     rev 2: 478 samples (450 valid)
//     ...
//     Stopping motor.
//     Done.
// =====================================================================

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "rplidar.h"  // NOLINT

using rp::standalone::rplidar::RPlidarDriver;

namespace {

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
constexpr int kDefaultRevolutions = 5;

std::atomic<bool> g_interrupted{false};

void on_sigint(int /*sig*/) { g_interrupted.store(true); }

struct DriverHandle {
    RPlidarDriver* driver = nullptr;
    explicit DriverHandle(RPlidarDriver* d) : driver(d) {}
    ~DriverHandle() {
        if (driver) {
            // Stop everything we may have started. Each of these is
            // safe to call even if it was never started.
            driver->stop();
            driver->stopMotor();
            RPlidarDriver::DisposeDriver(driver);
            driver = nullptr;
        }
    }
    DriverHandle(const DriverHandle&) = delete;
    DriverHandle& operator=(const DriverHandle&) = delete;
};

}  // namespace

int main(int argc, char** argv) {
    const char* port = (argc >= 2) ? argv[1] : default_port();
    uint32_t baudrate = kDefaultBaudrate;
    if (argc >= 3) {
        baudrate = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
    }
    int revolutions = kDefaultRevolutions;
    if (argc >= 4) {
        revolutions = std::atoi(argv[3]);
        if (revolutions <= 0) revolutions = kDefaultRevolutions;
    }

    std::printf("Port: %s  (baud=%u)  revolutions=%d\n",
        port, baudrate, revolutions);

    std::signal(SIGINT, on_sigint);

    DriverHandle handle{RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT)};
    if (!handle.driver) {
        std::fprintf(stderr, "Failed to create driver.\n");
        return 1;
    }

    if (IS_FAIL(handle.driver->connect(port, baudrate))) {
        std::fprintf(stderr,
            "Failed to connect on %s at %u baud.\n", port, baudrate);
        return 1;
    }

    std::printf("Starting motor.\n");
    handle.driver->startMotor();
    // Give the rotor a moment to spin up before requesting samples.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    // RPLIDAR_CONF_SCAN_COMMAND_STD selects the legacy standard scan.
    handle.driver->startScan(/*force=*/false, /*useTypicalScan=*/true);

    rplidar_response_measurement_node_hq_t nodes[8192];
    for (int rev = 0; rev < revolutions; ++rev) {
        if (g_interrupted.load()) {
            std::printf("Interrupted.\n");
            break;
        }
        size_t count = sizeof(nodes) / sizeof(nodes[0]);
        if (IS_FAIL(handle.driver->grabScanDataHq(nodes, count))) {
            std::fprintf(stderr, "grabScanDataHq() failed.\n");
            break;
        }
        // Optional but helpful: sort by angle to make the dump readable.
        handle.driver->ascendScanData(nodes, count);

        int valid = 0;
        for (size_t i = 0; i < count; ++i) {
            if (nodes[i].dist_mm_q2 != 0) ++valid;
        }
        std::printf("rev %d: %zu samples (%d valid)\n", rev + 1, count, valid);

        // Print a few samples per revolution so the dump is concrete
        // without being overwhelming.
        size_t to_print = (count < 5) ? count : 5;
        for (size_t i = 0; i < to_print; ++i) {
            float angle_deg = nodes[i].angle_z_q14 * 90.0f / (1 << 14);
            float distance_mm = nodes[i].dist_mm_q2 / 4.0f;
            std::printf("  [%4zu] q=%u  angle=%7.2f  dist=%8.1f\n",
                i, nodes[i].quality, angle_deg, distance_mm);
        }
    }

    std::printf("Stopping motor.\n");
    handle.driver->stop();
    handle.driver->stopMotor();
    std::printf("Done.\n");
    return 0;
}
