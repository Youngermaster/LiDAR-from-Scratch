"""
indian_guy_stuff.py
===================

An RViz / Nav2-flavored real-time LiDAR visualizer built on pygame.

Why this exists
---------------
The numbered experiments under `python/experiments/` use matplotlib
because matplotlib is the most readable way to teach the math (polar
to cartesian, scan structure, simple obstacle detection). It is not
the right tool for a live sensor view: it flickers, drops frames, and
has no easy way to draw the things you actually want to see, like
labeled range rings and a coordinate frame.

This file is a focused viewer, not a lesson. It mirrors how ROS RViz
draws a LaserScan: dark background, concentric range rings labeled in
metres, faint cartesian grid, a red X axis and green Y axis (the ROS
convention), a white robot footprint with a heading arrow at the
origin, and points colour-coded by distance. A heads-up display in
the top-left shows the live scan rate, point count, the maximum range
observed in the current revolution, and any error from the reader.

Architecture
------------
- A background thread runs the LiDAR (`RPLidarC1.iter_scans`) and
  pushes one fully-assembled revolution at a time into a shared dict
  guarded by a `threading.Lock`. This decouples serial I/O from
  rendering so a slow frame never starves the sensor reader.
- The main thread runs pygame at 60 FPS, snapshots the shared dict
  once per frame, and redraws everything.
- Clean shutdown: the main loop sets an Event when it exits; the
  reader thread sees it and falls through the `with RPLidarC1(...)`
  context manager, which stops the motor and releases the port.

Run
---
    pip install pygame
    python stuff/indian_guy_stuff.py
    python stuff/indian_guy_stuff.py --max-range-m 5 --motor-pwm 660

Keyboard controls
-----------------
    +  /  =       zoom in  (smaller visible range)
    -             zoom out (larger visible range)
    R             toggle range rings
    G             toggle cartesian grid
    T             toggle short trail of recent scans
    SPACE         pause / resume rendering
    ESC  /  Q     quit (motor stops cleanly on exit)

Common failures
---------------
- "Bad response descriptor sync bytes": a previous run left the
  sensor streaming. Unplug-replug and try again.
- Window opens but no points appear: the reader thread errored. The
  HUD shows the message; the most common cause is the wrong --port.
"""

from __future__ import annotations

import argparse
import glob
import math
import platform
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import pygame

# Allow `from lib.rplidar_c1 import ...` when running this file directly.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lib.rplidar_c1 import RPLidarC1, DEFAULT_MOTOR_PWM  # noqa: E402


# ---- constants -------------------------------------------------------

DEFAULT_BAUDRATE = 460800
WINDOW_SIZE_DEFAULT = 900

# Palette tuned to feel like RViz's default dark theme.
BG_COLOR = (10, 14, 22)
GRID_COLOR = (32, 38, 50)
RING_COLOR = (72, 92, 112)
RING_LABEL_COLOR = (150, 170, 190)
AXIS_X_COLOR = (200, 70, 70)       # ROS X is red
AXIS_Y_COLOR = (70, 200, 70)       # ROS Y is green
ORIGIN_COLOR = (240, 240, 240)
HUD_COLOR = (210, 225, 235)
HELP_COLOR = (110, 130, 150)
ERROR_COLOR = (255, 120, 100)
PAUSE_COLOR = (255, 200, 80)

# Point screen radius in pixels. 2 reads well at typical window sizes.
POINT_RADIUS = 2
TRAIL_MAX = 5                       # how many past scans to fade behind the latest


# ---- port discovery --------------------------------------------------

def auto_detect_port() -> Optional[str]:
    system = platform.system()
    if system == "Darwin":
        candidates = sorted(glob.glob("/dev/cu.usbserial-*"))
        candidates += sorted(glob.glob("/dev/cu.SLAB_USBtoUART"))
    elif system == "Linux":
        candidates = sorted(glob.glob("/dev/ttyUSB*"))
    else:
        return None
    return candidates[0] if candidates else None


# ---- colour gradient -------------------------------------------------

def color_for_distance(d_m: float, max_m: float) -> Tuple[int, int, int]:
    """Three-stop gradient: near = indigo, mid = teal, far = chartreuse.

    Roughly approximates matplotlib's "viridis" without depending on
    matplotlib at runtime. The mapping clamps at the extremes so a
    sample beyond the visible range still renders in a sensible
    colour.
    """
    t = max(0.0, min(1.0, d_m / max_m))
    if t < 0.5:
        u = t * 2.0
        r = int(50 + (0 - 50) * u)
        g = int(0 + (180 - 0) * u)
        b = int(100 + (180 - 100) * u)
    else:
        u = (t - 0.5) * 2.0
        r = int(0 + (220 - 0) * u)
        g = int(180 + (220 - 180) * u)
        b = int(180 + (50 - 180) * u)
    return (r, g, b)


def fade_color(rgb: Tuple[int, int, int], factor: float) -> Tuple[int, int, int]:
    return (int(rgb[0] * factor), int(rgb[1] * factor), int(rgb[2] * factor))


# ---- reader thread ---------------------------------------------------

def reader_loop(
    port: str,
    baudrate: int,
    motor_pwm: int,
    shared: Dict[str, Any],
    lock: threading.Lock,
    stop_event: threading.Event,
) -> None:
    """Run the LiDAR and publish one revolution at a time into `shared`.

    `shared` keys:
      points:   list of (x_m, y_m, distance_mm, quality)
      hz:       scan rate measured between consecutive start_flag samples
      count:    number of valid points in the most recent revolution
      max_d_m:  longest valid range in the most recent revolution
      error:    human-readable error string, or None
    """
    try:
        with RPLidarC1(port=port, baudrate=baudrate, timeout=2.0) as lidar:
            lidar.set_motor_pwm(motor_pwm)
            current: List[Tuple[float, float, float, int]] = []
            last_rev_time = time.time()

            for sample in lidar.iter_scans():
                if stop_event.is_set():
                    return

                if sample.start_flag and current:
                    now = time.time()
                    dt = now - last_rev_time
                    hz = (1.0 / dt) if dt > 0 else 0.0
                    max_d_m = max((p[2] for p in current), default=0.0) / 1000.0
                    with lock:
                        shared["points"] = current
                        shared["hz"] = hz
                        shared["count"] = len(current)
                        shared["max_d_m"] = max_d_m
                    last_rev_time = now
                    current = []

                if sample.distance > 0.0:
                    theta = math.radians(sample.angle)
                    x = sample.distance * math.cos(theta) / 1000.0  # m
                    y = sample.distance * math.sin(theta) / 1000.0  # m
                    current.append((x, y, sample.distance, sample.quality))
    except Exception as exc:
        with lock:
            shared["error"] = str(exc)


# ---- drawing helpers -------------------------------------------------

def pick_ring_step(max_range_m: float) -> float:
    """Pick a tidy ring spacing for the current zoom level."""
    if max_range_m <= 2.0:
        return 0.5
    if max_range_m <= 5.0:
        return 1.0
    if max_range_m <= 10.0:
        return 2.0
    return 5.0


def world_to_screen(
    x_m: float, y_m: float, max_range_m: float,
    center: Tuple[int, int], radius_px: int,
) -> Tuple[int, int]:
    """Project a (x, y) point in metres onto pygame screen pixels.

    Screen Y is inverted (positive Y goes up in the world frame but
    down on screen), so we subtract instead of add.
    """
    scale = radius_px / max_range_m
    return (int(center[0] + x_m * scale),
            int(center[1] - y_m * scale))


def draw_grid(
    surface: pygame.Surface,
    center: Tuple[int, int],
    radius_px: int,
    max_range_m: float,
) -> None:
    """Faint cartesian grid plus emphasized X (red) and Y (green) axes."""
    scale = radius_px / max_range_m
    step_m = pick_ring_step(max_range_m)
    n = int(max_range_m / step_m) + 1
    for i in range(-n, n + 1):
        if i == 0:
            continue
        offset = i * step_m * scale
        x = int(center[0] + offset)
        y = int(center[1] + offset)
        pygame.draw.line(
            surface, GRID_COLOR,
            (x, center[1] - radius_px), (x, center[1] + radius_px), 1,
        )
        pygame.draw.line(
            surface, GRID_COLOR,
            (center[0] - radius_px, y), (center[0] + radius_px, y), 1,
        )

    pygame.draw.line(
        surface, AXIS_X_COLOR,
        (center[0] - radius_px, center[1]),
        (center[0] + radius_px, center[1]), 1,
    )
    pygame.draw.line(
        surface, AXIS_Y_COLOR,
        (center[0], center[1] - radius_px),
        (center[0], center[1] + radius_px), 1,
    )


def draw_range_rings(
    surface: pygame.Surface,
    font: pygame.font.Font,
    center: Tuple[int, int],
    radius_px: int,
    max_range_m: float,
) -> None:
    """Concentric circles labeled with their radius in metres."""
    scale = radius_px / max_range_m
    step = pick_ring_step(max_range_m)
    r = step
    while r <= max_range_m + 1e-6:
        rpx = int(r * scale)
        if rpx >= 6:
            pygame.draw.circle(surface, RING_COLOR, center, rpx, 1)
            label = font.render(f"{r:g} m", True, RING_LABEL_COLOR)
            surface.blit(label, (center[0] + rpx + 4, center[1] - 16))
        r += step

    # Angle ticks every 30 degrees on the outermost visible ring.
    outer_r = int(max_range_m * scale)
    for deg in range(0, 360, 30):
        rad = math.radians(deg)
        outer = (center[0] + outer_r * math.cos(rad),
                 center[1] - outer_r * math.sin(rad))
        inner = (center[0] + (outer_r - 8) * math.cos(rad),
                 center[1] - (outer_r - 8) * math.sin(rad))
        pygame.draw.line(surface, RING_COLOR, inner, outer, 1)


def draw_origin(surface: pygame.Surface, center: Tuple[int, int]) -> None:
    """Small white ring with a forward-heading triangle along +X."""
    pygame.draw.circle(surface, ORIGIN_COLOR, center, 6, 2)
    tip = (center[0] + 22, center[1])
    base_l = (center[0] + 6, center[1] - 7)
    base_r = (center[0] + 6, center[1] + 7)
    pygame.draw.polygon(surface, ORIGIN_COLOR, [tip, base_l, base_r])


def draw_hud(
    surface: pygame.Surface,
    font: pygame.font.Font,
    lines: List[Tuple[str, Tuple[int, int, int]]],
    origin: Tuple[int, int] = (12, 10),
) -> None:
    """Render multi-line text at (x, y) with per-line colour."""
    for i, (text, color) in enumerate(lines):
        surf = font.render(text, True, color)
        surface.blit(surf, (origin[0], origin[1] + i * 18))


# ---- main ------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RViz-like LiDAR live viewer.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--motor-pwm", type=int, default=DEFAULT_MOTOR_PWM)
    parser.add_argument("--max-range-m", type=float, default=8.0,
                        help="Initial visible radius in metres. Adjust live with +/-.")
    parser.add_argument("--window-size", type=int, default=WINDOW_SIZE_DEFAULT,
                        help="Square window edge length in pixels.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or auto_detect_port()
    if port is None:
        print("No candidate ports found. Pass --port explicitly.", file=sys.stderr)
        return 2

    pygame.init()
    screen = pygame.display.set_mode(
        (args.window_size, args.window_size), pygame.DOUBLEBUF
    )
    pygame.display.set_caption("LiDAR C1 - live (RViz style)")
    clock = pygame.time.Clock()

    hud_font = pygame.font.SysFont(["menlo", "consolas", "monospace"], 14)
    ring_font = pygame.font.SysFont(["menlo", "consolas", "monospace"], 12)

    shared: Dict[str, Any] = {
        "points": [],
        "hz": 0.0,
        "count": 0,
        "max_d_m": 0.0,
        "error": None,
    }
    lock = threading.Lock()
    stop_event = threading.Event()

    reader = threading.Thread(
        target=reader_loop,
        args=(port, args.baudrate, args.motor_pwm, shared, lock, stop_event),
        daemon=True,
    )
    reader.start()

    max_range_m = args.max_range_m
    show_rings = True
    show_grid = True
    show_trail = False
    paused = False
    trail: List[List[Tuple[float, float, float, int]]] = []

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    max_range_m = max(0.5, max_range_m / 1.5)
                elif event.key == pygame.K_MINUS:
                    max_range_m = min(20.0, max_range_m * 1.5)
                elif event.key == pygame.K_r:
                    show_rings = not show_rings
                elif event.key == pygame.K_g:
                    show_grid = not show_grid
                elif event.key == pygame.K_t:
                    show_trail = not show_trail
                    if not show_trail:
                        trail.clear()
                elif event.key == pygame.K_SPACE:
                    paused = not paused

        # Snapshot under the lock to keep the rendering pure.
        with lock:
            pts = list(shared["points"])
            hz = shared["hz"]
            count = shared["count"]
            max_d_m = shared["max_d_m"]
            error = shared["error"]

        if not paused:
            if show_trail:
                trail.append(pts)
                trail = trail[-TRAIL_MAX:]
            else:
                trail = [pts]

        # ---- render ----
        screen.fill(BG_COLOR)
        w, h = screen.get_size()
        center = (w // 2, h // 2)
        radius_px = min(w, h) // 2 - 60

        if show_grid:
            draw_grid(screen, center, radius_px, max_range_m)
        if show_rings:
            draw_range_rings(screen, ring_font, center, radius_px, max_range_m)

        layers = trail if trail else [pts]
        n_layers = max(1, len(layers))
        for i, layer in enumerate(layers):
            alpha = (i + 1) / n_layers
            for (x_m, y_m, d_mm, _q) in layer:
                if abs(x_m) > max_range_m or abs(y_m) > max_range_m:
                    continue
                base = color_for_distance(d_mm / 1000.0, max_range_m)
                col = base if alpha >= 0.999 else fade_color(base, alpha)
                sp = world_to_screen(x_m, y_m, max_range_m, center, radius_px)
                pygame.draw.circle(screen, col, sp, POINT_RADIUS)

        draw_origin(screen, center)

        # ---- HUD ----
        hud_lines: List[Tuple[str, Tuple[int, int, int]]] = [
            (f"Port:           {port}", HUD_COLOR),
            (f"Scan rate:      {hz:5.1f} Hz", HUD_COLOR),
            (f"Points / scan:  {count:4d}", HUD_COLOR),
            (f"Max range obs:  {max_d_m:5.2f} m", HUD_COLOR),
            (f"Visible range:  {max_range_m:5.2f} m", HUD_COLOR),
        ]
        if paused:
            hud_lines.append(("PAUSED", PAUSE_COLOR))
        if error:
            hud_lines.append((f"ERROR: {error}", ERROR_COLOR))
        draw_hud(screen, hud_font, hud_lines, origin=(12, 10))

        help_text = (
            "+/-: zoom    R: rings    G: grid    T: trail    "
            "SPACE: pause    ESC/Q: quit"
        )
        help_surf = hud_font.render(help_text, True, HELP_COLOR)
        screen.blit(help_surf, (12, h - 26))

        pygame.display.flip()
        clock.tick(60)

    stop_event.set()
    reader.join(timeout=2.0)
    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
