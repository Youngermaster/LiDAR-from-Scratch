"""
Pygame application: main loop, view switching, threaded LiDAR reader.

Why a thread
------------
Reading the LiDAR over serial is blocking. Running pygame is also
blocking on its own clock. Keeping them on separate threads means a
slow render frame never starves the sensor (the reader's serial
buffer would overflow otherwise) and a slow ICP step never causes
the UI to freeze.

The reader thread owns the sensor and the mapper. It publishes a
single immutable `MapperSnapshot` to the main thread once per
revolution. The main thread reads the snapshot under a lock and
renders the selected view.

Shutdown is via a `threading.Event`. Setting it makes the reader's
`for sample in iter_scans()` loop fall through to the
`LiveLidarSource.__exit__` which stops the motor and releases the
serial port. We then `join(timeout)` from the main thread.
"""

from __future__ import annotations

import argparse
import datetime
import logging
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
import pygame

from .driver import DEFAULT_MOTOR_PWM
from .mapper import Mapper, MapperSnapshot
from .source import (
    CsvReplaySource,
    LiveLidarSource,
    ScanRevolution,
    auto_detect_port,
)
from .views import (
    BG_COLOR,
    ERROR_COLOR,
    HELP_COLOR,
    HUD_COLOR,
    PAUSE_COLOR,
    DistanceView,
    LiveView,
    MapView,
)


LOG = logging.getLogger("house_mapper")

DEFAULT_BAUDRATE = 460800
WINDOW_SIZE_DEFAULT = 1000
TARGET_FPS = 60

# Conventional layout: house_mapper/data/ has two subfolders.
# Replay inputs live in recordings/, snapshot outputs in maps/.
# The paths are resolved relative to this file's location so the
# tool works regardless of the user's cwd.
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = _PROJECT_ROOT / "data"
RECORDINGS_DIR = DATA_DIR / "recordings"
MAPS_DIR = DATA_DIR / "maps"


# ---- shared state between threads ------------------------------------

@dataclass
class SharedState:
    """The reader thread writes this; the main thread reads it."""

    snapshot: Optional[MapperSnapshot] = None
    last_scan: Optional[ScanRevolution] = None
    revolution_count: int = 0
    hz: float = 0.0
    error: Optional[str] = None


# ---- reader thread ---------------------------------------------------

def _reader_loop(
    source_factory,
    mapper: Mapper,
    shared: SharedState,
    lock: threading.Lock,
    stop_event: threading.Event,
    pause_event: threading.Event,
) -> None:
    """Run a scan source and feed revolutions to the Mapper.

    `source_factory` is a zero-arg callable that returns a context
    manager whose `iter_revolutions()` yields `ScanRevolution`. This
    indirection lets us swap live and replay sources without touching
    the thread body.
    """
    try:
        with source_factory() as source:
            last_rev_time = time.time()
            for revolution in source.iter_revolutions():
                if stop_event.is_set():
                    return

                now = time.time()
                dt = now - last_rev_time
                hz = (1.0 / dt) if dt > 0 else 0.0
                last_rev_time = now

                if not pause_event.is_set():
                    mapper.add_scan(revolution.points_body_m)

                snapshot = mapper.snapshot()
                with lock:
                    shared.snapshot = snapshot
                    shared.last_scan = revolution
                    shared.revolution_count += 1
                    shared.hz = hz
    except Exception as exc:
        LOG.exception("reader thread aborted")
        with lock:
            shared.error = str(exc)


# ---- snapshot saving -------------------------------------------------

def save_snapshot(snapshot: MapperSnapshot, out_dir: Path) -> Path:
    """Write a PNG of the current map and a CSV of the trajectory.

    Returns the path of the PNG. Both files share a timestamped
    basename so they sort together.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    png_path = out_dir / f"map_{stamp}.png"
    csv_path = out_dir / f"trajectory_{stamp}.csv"

    # Use pygame to write the PNG since we already have the grayscale
    # image as a numpy array. Pillow would also work; we prefer the
    # dependency we already use.
    image = snapshot.grid_image
    rgb = np.repeat(image[:, :, None], 3, axis=2)
    surf = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    pygame.image.save(surf, str(png_path))

    with csv_path.open("w") as fh:
        fh.write("x_m,y_m,theta_rad\n")
        for pose in snapshot.trajectory:
            fh.write(f"{pose.x:.6f},{pose.y:.6f},{pose.theta:.6f}\n")

    LOG.info("Saved map snapshot to %s and trajectory to %s", png_path, csv_path)
    return png_path


# ---- argparse --------------------------------------------------------

def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Walk-and-scan 2D mapper for the RPLIDAR C1.",
    )
    source = p.add_argument_group("source")
    source.add_argument("--port", default=None,
                        help="Serial port for live LiDAR. Auto-detected if omitted.")
    source.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    source.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM,
                        help="C1 motor PWM 0..1023.")
    source.add_argument("--replay", type=Path, default=None,
                        help="Replay a CSV. Bare filenames resolve under "
                             "data/recordings/.")
    source.add_argument("--record", action="store_true",
                        help="While running live, also write each sample to "
                             "data/recordings/scan_<ts>.csv for later replay.")

    mapping = p.add_argument_group("mapping")
    mapping.add_argument("--grid-size-m", type=float, default=30.0)
    mapping.add_argument("--grid-resolution-m", type=float, default=0.05)
    mapping.add_argument("--icp-max-correspondence-m", type=float, default=0.5)

    ui = p.add_argument_group("ui")
    ui.add_argument("--window-size", type=int, default=WINDOW_SIZE_DEFAULT)
    ui.add_argument("--visible-radius-m", type=float, default=10.0,
                    help="Initial visible radius. Adjust live with +/-.")
    ui.add_argument("--save-dir", type=Path, default=MAPS_DIR,
                    help="Where snapshot saves land. Defaults to data/maps/.")
    ui.add_argument("--start-view", type=int, choices=(1, 2, 3), default=1)

    p.add_argument("--verbose", action="store_true",
                   help="Show INFO-level logs and a timestamp prefix. "
                        "Default is WARNING (quiet).")
    return p.parse_args(argv)


def _resolve_replay_path(arg: Path) -> Optional[Path]:
    """Resolve --replay value to a real CSV path, or None if not found.

    Search order:
      1. The path as given (relative to cwd or absolute).
      2. data/recordings/<basename> under this project.
    """
    if arg.exists():
        return arg
    candidate = RECORDINGS_DIR / arg.name
    if candidate.exists():
        return candidate
    return None


# ---- main ------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    # Quiet by default: only warnings/errors. --verbose for INFO + timestamps.
    if args.verbose:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        )
    else:
        logging.basicConfig(level=logging.WARNING, format="%(message)s")

    # ---- pick a source ----
    if args.replay is not None:
        resolved = _resolve_replay_path(args.replay)
        if resolved is None:
            print(
                f"Replay file not found: {args.replay}\n"
                f"Looked in cwd and {RECORDINGS_DIR}",
                file=sys.stderr,
            )
            return 2
        source_factory = lambda r=resolved: CsvReplaySource(r)
        source_label = f"REPLAY {resolved.name}"
        record_path: Optional[Path] = None  # cannot record from a replay
    else:
        port = args.port or auto_detect_port()
        if port is None:
            print(
                "No candidate ports found. Pass --port explicitly or use --replay.",
                file=sys.stderr,
            )
            return 2
        record_path = None
        if args.record:
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            record_path = RECORDINGS_DIR / f"scan_{stamp}.csv"
            print(f"Recording to {record_path}")
        source_factory = lambda p=port, rp=record_path: LiveLidarSource(
            port=p, baudrate=args.baudrate, motor_pwm=args.motor_pwm,
            record_path=rp,
        )
        source_label = f"LIVE {port}"

    # ---- mapper + thread ----
    mapper = Mapper(
        grid_size_m=args.grid_size_m,
        grid_resolution_m=args.grid_resolution_m,
        icp_max_correspondence_m=args.icp_max_correspondence_m,
    )
    shared = SharedState()
    lock = threading.Lock()
    stop_event = threading.Event()
    pause_event = threading.Event()

    reader = threading.Thread(
        target=_reader_loop,
        args=(source_factory, mapper, shared, lock, stop_event, pause_event),
        daemon=True,
        name="lidar-reader",
    )
    reader.start()

    # ---- pygame ----
    pygame.init()
    screen = pygame.display.set_mode(
        (args.window_size, args.window_size), pygame.DOUBLEBUF,
    )
    pygame.display.set_caption(f"House Mapper - {source_label}")
    clock = pygame.time.Clock()
    hud_font = pygame.font.SysFont(["menlo", "consolas", "monospace"], 14)
    ring_font = pygame.font.SysFont(["menlo", "consolas", "monospace"], 12)

    map_view = MapView()
    live_view = LiveView()
    distance_view = DistanceView()

    ui = _UiState(
        running=True,
        selected_view=args.start_view,
        visible_radius_m=args.visible_radius_m,
        last_save_path=None,
        last_save_time=0.0,
    )

    while ui.running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                ui.running = False
            elif event.type == pygame.KEYDOWN:
                _handle_keydown(event, ui, mapper, pause_event, shared, lock,
                                args.save_dir)

        # Snapshot under lock.
        with lock:
            snapshot = shared.snapshot
            scan = shared.last_scan
            rev_count = shared.revolution_count
            hz = shared.hz
            error = shared.error

        screen.fill(BG_COLOR)

        if ui.selected_view == 1 and snapshot is not None:
            map_view.render(screen, hud_font, ring_font, snapshot, ui.visible_radius_m)
        elif ui.selected_view == 2:
            live_view.render(screen, hud_font, ring_font, scan, ui.visible_radius_m)
        elif ui.selected_view == 3:
            distance_view.render(screen, hud_font, ring_font, scan, ui.visible_radius_m)
        else:
            # Map view requested but no snapshot yet (still spinning up).
            _draw_waiting_overlay(screen, hud_font)

        # Universal HUD footer.
        h = screen.get_size()[1]
        footer_lines: List[tuple[str, tuple[int, int, int]]] = []
        if pause_event.is_set():
            footer_lines.append(("PAUSED (mapping)", PAUSE_COLOR))
        if error:
            footer_lines.append((f"ERROR: {error}", ERROR_COLOR))
        if (ui.last_save_path is not None
                and time.time() - ui.last_save_time < 3.0):
            footer_lines.append((f"Saved: {ui.last_save_path.name}", HUD_COLOR))

        for i, (text, color) in enumerate(footer_lines):
            surf = hud_font.render(text, True, color)
            screen.blit(surf, (12, h - 80 + i * 18))

        help_text = (
            "1/2/3 view    +/-: zoom    SPACE: pause mapping    "
            "R: reset map    S: save    ESC/Q: quit"
        )
        screen.blit(
            hud_font.render(help_text, True, HELP_COLOR),
            (12, h - 26),
        )

        # Top-right meta line.
        meta = f"rev #{rev_count}    {hz:5.1f} Hz"
        meta_surf = hud_font.render(meta, True, HUD_COLOR)
        screen.blit(meta_surf, (screen.get_size()[0] - meta_surf.get_width() - 12, 10))

        pygame.display.flip()
        clock.tick(TARGET_FPS)

    LOG.info("Shutting down")
    stop_event.set()
    reader.join(timeout=3.0)
    pygame.quit()
    return 0


@dataclass
class _UiState:
    """Live UI state mutated by key handlers and read by the render loop."""

    running: bool = True
    selected_view: int = 1
    visible_radius_m: float = 10.0
    last_save_path: Optional[Path] = None
    last_save_time: float = 0.0


def _handle_keydown(
    event: pygame.event.Event,
    ui: _UiState,
    mapper: Mapper,
    pause_event: threading.Event,
    shared: SharedState,
    lock: threading.Lock,
    save_dir: Path,
) -> None:
    """Mutate `ui` (and pause_event / mapper / shared) based on the key."""
    if event.key in (pygame.K_ESCAPE, pygame.K_q):
        ui.running = False
    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
        ui.visible_radius_m = max(0.5, ui.visible_radius_m / 1.5)
    elif event.key == pygame.K_MINUS:
        ui.visible_radius_m = min(50.0, ui.visible_radius_m * 1.5)
    elif event.key == pygame.K_1:
        ui.selected_view = 1
    elif event.key == pygame.K_2:
        ui.selected_view = 2
    elif event.key == pygame.K_3:
        ui.selected_view = 3
    elif event.key == pygame.K_SPACE:
        if pause_event.is_set():
            pause_event.clear()
            LOG.info("Mapping resumed")
        else:
            pause_event.set()
            LOG.info("Mapping paused")
    elif event.key == pygame.K_r:
        # Hold the lock so the reader is not mid-integration.
        with lock:
            mapper.reset()
            shared.snapshot = mapper.snapshot()
        LOG.info("Map reset")
    elif event.key == pygame.K_s:
        with lock:
            snapshot = shared.snapshot
        if snapshot is not None:
            ui.last_save_path = save_snapshot(snapshot, save_dir)
            ui.last_save_time = time.time()


def _draw_waiting_overlay(surface: pygame.Surface, font: pygame.font.Font) -> None:
    w, h = surface.get_size()
    text = "Waiting for first scan..."
    surf = font.render(text, True, HUD_COLOR)
    surface.blit(surf, ((w - surf.get_width()) // 2, h // 2))
