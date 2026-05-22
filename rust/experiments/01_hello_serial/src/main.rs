// =====================================================================
// 01_hello_serial
// =====================================================================
//
// Purpose
// -------
// Enumerate the serial ports the host can see, pick the one most likely
// to be an RPLIDAR C1, and try to open it at 460800 baud. This is the
// Rust counterpart of the auto-detect step in
// python/experiments/01_hello_lidar.py, stopping just short of
// implementing the protocol. The point here is to get a feel for
// Rust's serial-port story, not to reimplement the RPLIDAR driver.
//
// What this teaches
// -----------------
// - The `serialport` crate's API for listing and opening ports.
// - Idiomatic Rust error propagation with `?` and `anyhow`-free code
//   (we use the standard `Result` and print errors at the boundary).
// - Why Rust forces you to think about timeouts and ownership at the
//   serial-port boundary: dropping the handle closes the device.
//
// Build and run
// -------------
//     cargo run --bin hello_serial
//     cargo run --bin hello_serial -- --port /dev/cu.usbserial-0001
//
// Expected output
// ---------------
//     Visible serial ports:
//       /dev/cu.Bluetooth-Incoming-Port  (Bluetooth)
//       /dev/cu.usbserial-0001           (USB: vid=10c4 pid=ea60)
//     Best candidate: /dev/cu.usbserial-0001
//     Opened /dev/cu.usbserial-0001 at 460800 baud.
//     Holding the port open for 1 second...
//     Closed.
//
// Common failures
// ---------------
// - "No candidate found": no USB-serial adapter present. Plug it in.
// - "PermissionDenied" on Linux: add yourself to the dialout group.
// =====================================================================

use std::env;
use std::process::ExitCode;
use std::thread;
use std::time::Duration;

use serialport::{SerialPortInfo, SerialPortType};

const DEFAULT_BAUDRATE: u32 = 460800;

fn looks_like_rplidar(info: &SerialPortInfo) -> bool {
    // On macOS the relevant ports are /dev/cu.usbserial-* or the older
    // /dev/cu.SLAB_USBtoUART. On Linux they are /dev/ttyUSB*. We use
    // both the path and the USB descriptor to decide.
    let name = info.port_name.as_str();
    if name.starts_with("/dev/cu.usbserial-")
        || name == "/dev/cu.SLAB_USBtoUART"
        || name.starts_with("/dev/ttyUSB")
    {
        return true;
    }
    matches!(&info.port_type, SerialPortType::UsbPort(_))
}

fn describe_port(info: &SerialPortInfo) -> String {
    match &info.port_type {
        SerialPortType::UsbPort(usb) => {
            format!(
                "USB: vid={:04x} pid={:04x}{}{}",
                usb.vid,
                usb.pid,
                usb.manufacturer
                    .as_ref()
                    .map(|m| format!(" mfr={m}"))
                    .unwrap_or_default(),
                usb.product
                    .as_ref()
                    .map(|p| format!(" product={p}"))
                    .unwrap_or_default(),
            )
        }
        SerialPortType::BluetoothPort => "Bluetooth".into(),
        SerialPortType::PciPort => "PCI".into(),
        SerialPortType::Unknown => "Unknown".into(),
    }
}

fn parse_args() -> Option<String> {
    // Tiny ad-hoc argument parser: `--port <path>` or nothing. We avoid
    // pulling in `clap` for this single flag.
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--port" {
            return args.next();
        }
    }
    None
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let ports = serialport::available_ports()?;
    println!("Visible serial ports:");
    if ports.is_empty() {
        println!("  (none)");
    }
    for p in &ports {
        println!("  {:<35}({})", p.port_name, describe_port(p));
    }

    let chosen = match parse_args() {
        Some(p) => p,
        None => match ports.iter().find(|p| looks_like_rplidar(p)) {
            Some(p) => p.port_name.clone(),
            None => {
                eprintln!("No candidate RPLIDAR port found. Pass --port <path>.");
                return Err("no candidate".into());
            }
        },
    };

    println!("Best candidate: {chosen}");

    // Open with a short read timeout so a stuck device does not hang
    // the program. Closing happens automatically when `port` is dropped.
    let port = serialport::new(&chosen, DEFAULT_BAUDRATE)
        .timeout(Duration::from_millis(500))
        .open()?;
    println!("Opened {chosen} at {DEFAULT_BAUDRATE} baud.");

    println!("Holding the port open for 1 second...");
    thread::sleep(Duration::from_secs(1));

    drop(port);
    println!("Closed.");
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {e}");
            ExitCode::FAILURE
        }
    }
}
