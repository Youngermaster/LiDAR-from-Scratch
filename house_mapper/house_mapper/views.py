"""
Pygame views for the house mapper.

Three views share a common style (dark background, ROS axis colours,
ring labels in metres) and a common camera (the user can zoom in and
out with +/-, which scales `visible_radius_m`). Each view exposes a
single `render(surface, ...)` method. The app loop picks which view
to call based on the current selection.

Keeping the views in one file is fine because they share a lot of
palette and helper functions. If a fourth view shows up the file
can be split without changing the public API.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pygame

from .geometry import Pose2D
from .mapper import MapperSnapshot
from .source import ScanRevolution


# ---- palette ---------------------------------------------------------

BG_COLOR = (10, 14, 22)
GRID_COLOR = (32, 38, 50)
RING_COLOR = (72, 92, 112)
RING_LABEL_COLOR = (150, 170, 190)
AXIS_X_COLOR = (200, 70, 70)
AXIS_Y_COLOR = (70, 200, 70)
ORIGIN_COLOR = (240, 240, 240)
HUD_COLOR = (210, 225, 235)
HELP_COLOR = (110, 130, 150)
ERROR_COLOR = (255, 120, 100)
PAUSE_COLOR = (255, 200, 80)
TRAJECTORY_COLOR = (255, 215, 100)
LIVE_SCAN_COLOR = (120, 220, 255)
SECTOR_LINE_COLOR = (60, 80, 100)
SECTOR_TEXT_COLOR = (200, 220, 235)
SECTOR_BAR_COLOR = (90, 180, 230)


# ---- shared helpers --------------------------------------------------

def pick_ring_step(visible_radius_m: float) -> float:
    """Choose a tidy ring spacing in metres for the current zoom."""
    if visible_radius_m <= 2.0:
        return 0.5
    if visible_radius_m <= 5.0:
        return 1.0
    if visible_radius_m <= 10.0:
        return 2.0
    if visible_radius_m <= 25.0:
        return 5.0
    return 10.0


def color_for_distance(d_m: float, max_m: float) -> Tuple[int, int, int]:
    """3-stop gradient near=indigo, mid=teal, far=chartreuse."""
    t = max(0.0, min(1.0, d_m / max_m))
    if t < 0.5:
        u = t * 2.0
        return (
            int(50 + (0 - 50) * u),
            int(0 + (180 - 0) * u),
            int(100 + (180 - 100) * u),
        )
    u = (t - 0.5) * 2.0
    return (
        int(0 + (220 - 0) * u),
        int(180 + (220 - 180) * u),
        int(180 + (50 - 180) * u),
    )


def world_to_screen(
    x_m: float,
    y_m: float,
    visible_radius_m: float,
    center: Tuple[int, int],
    radius_px: int,
    camera_xy_m: Tuple[float, float] = (0.0, 0.0),
) -> Tuple[int, int]:
    """Project world metres to screen pixels, optionally panned to `camera_xy_m`."""
    scale = radius_px / visible_radius_m
    return (
        int(center[0] + (x_m - camera_xy_m[0]) * scale),
        int(center[1] - (y_m - camera_xy_m[1]) * scale),
    )


def draw_rings_and_grid(
    surface: pygame.Surface,
    font: pygame.font.Font,
    center: Tuple[int, int],
    radius_px: int,
    visible_radius_m: float,
    show_grid: bool,
    show_rings: bool,
) -> None:
    """Common backdrop: faint cartesian grid + concentric rings + axes."""
    scale = radius_px / visible_radius_m
    step_m = pick_ring_step(visible_radius_m)

    if show_grid:
        n = int(visible_radius_m / step_m) + 1
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

    if show_rings:
        r = step_m
        while r <= visible_radius_m + 1e-6:
            rpx = int(r * scale)
            if rpx >= 6:
                pygame.draw.circle(surface, RING_COLOR, center, rpx, 1)
                label = font.render(f"{r:g} m", True, RING_LABEL_COLOR)
                surface.blit(label, (center[0] + rpx + 4, center[1] - 16))
            r += step_m


def draw_text_lines(
    surface: pygame.Surface,
    font: pygame.font.Font,
    lines: Iterable[Tuple[str, Tuple[int, int, int]]],
    origin: Tuple[int, int],
) -> None:
    for i, (text, color) in enumerate(lines):
        surface.blit(font.render(text, True, color), (origin[0], origin[1] + i * 18))


# ---- map view --------------------------------------------------------

@dataclass
class MapView:
    """Top-down occupancy grid + trajectory + current pose."""

    show_grid: bool = True
    show_rings: bool = True
    show_live_scan_overlay: bool = True

    def render(
        self,
        surface: pygame.Surface,
        font_hud: pygame.font.Font,
        font_ring: pygame.font.Font,
        snapshot: MapperSnapshot,
        visible_radius_m: float,
    ) -> None:
        w, h = surface.get_size()
        center = (w // 2, h // 2)
        radius_px = min(w, h) // 2 - 80

        # The camera follows the sensor so the user always sees the
        # neighborhood of where they are standing right now.
        camera = (snapshot.pose.x, snapshot.pose.y)

        # Blit the grid image first, scaled to the visible radius.
        _blit_grid(
            surface,
            snapshot.grid_image,
            snapshot.grid_size_m,
            snapshot.grid_resolution_m,
            visible_radius_m,
            center,
            radius_px,
            camera,
        )
        draw_rings_and_grid(
            surface, font_ring, center, radius_px,
            visible_radius_m, self.show_grid, self.show_rings,
        )

        # Trajectory.
        if len(snapshot.trajectory) >= 2:
            pts = [
                world_to_screen(p.x, p.y, visible_radius_m, center, radius_px, camera)
                for p in snapshot.trajectory
            ]
            pygame.draw.lines(surface, TRAJECTORY_COLOR, False, pts, 2)

        # Current pose: white ring with heading triangle.
        sx, sy = world_to_screen(
            snapshot.pose.x, snapshot.pose.y,
            visible_radius_m, center, radius_px, camera,
        )
        _draw_pose_marker(surface, (sx, sy), snapshot.pose.theta)

        # Live scan overlay.
        if self.show_live_scan_overlay and snapshot.last_scan_world_m.size:
            for x_m, y_m in snapshot.last_scan_world_m:
                if (abs(x_m - camera[0]) > visible_radius_m
                        or abs(y_m - camera[1]) > visible_radius_m):
                    continue
                spx = world_to_screen(
                    x_m, y_m, visible_radius_m, center, radius_px, camera,
                )
                pygame.draw.circle(surface, LIVE_SCAN_COLOR, spx, 2)

        # HUD.
        pose = snapshot.pose
        draw_text_lines(
            surface, font_hud,
            [
                (f"View 1 / 3:    Map", HUD_COLOR),
                (f"Pose:          ({pose.x:+.2f}, {pose.y:+.2f}) m  "
                 f"theta={math.degrees(pose.theta):+6.1f} deg", HUD_COLOR),
                (f"Scans:         {snapshot.scans_integrated}", HUD_COLOR),
                (f"ICP inliers:   {snapshot.last_icp_inliers}", HUD_COLOR),
                (f"ICP err:       {snapshot.last_icp_error_m2:.4f} m^2", HUD_COLOR),
                (f"Visible:       {visible_radius_m:.2f} m", HUD_COLOR),
            ],
            origin=(12, 10),
        )


def _draw_pose_marker(
    surface: pygame.Surface,
    screen_xy: Tuple[int, int],
    theta_rad: float,
) -> None:
    pygame.draw.circle(surface, ORIGIN_COLOR, screen_xy, 7, 2)
    tip_len = 18
    tip = (
        int(screen_xy[0] + tip_len * math.cos(theta_rad)),
        int(screen_xy[1] - tip_len * math.sin(theta_rad)),
    )
    # Triangle base perpendicular to heading.
    perp = theta_rad + math.pi / 2
    base_offset = 6
    base_l = (
        int(screen_xy[0] + 6 * math.cos(theta_rad) + base_offset * math.cos(perp)),
        int(screen_xy[1] - 6 * math.sin(theta_rad) - base_offset * math.sin(perp)),
    )
    base_r = (
        int(screen_xy[0] + 6 * math.cos(theta_rad) - base_offset * math.cos(perp)),
        int(screen_xy[1] - 6 * math.sin(theta_rad) + base_offset * math.sin(perp)),
    )
    pygame.draw.polygon(surface, ORIGIN_COLOR, [tip, base_l, base_r])


def _blit_grid(
    surface: pygame.Surface,
    grid_image: np.ndarray,
    grid_size_m: float,
    grid_resolution_m: float,
    visible_radius_m: float,
    center: Tuple[int, int],
    radius_px: int,
    camera_xy_m: Tuple[float, float],
) -> None:
    """Scale + blit the occupancy grid so the camera position is at `center`."""
    if grid_image.size == 0:
        return
    cells = grid_image.shape[0]
    grid_origin_cell = cells // 2

    # Pixels-per-metre for the rendered window.
    px_per_m = radius_px / visible_radius_m

    # Slice the grid window we need. We want a square in world space
    # centred on `camera_xy_m` with side 2 * visible_radius_m.
    col_min = int(round((camera_xy_m[0] - visible_radius_m) / grid_resolution_m)) + grid_origin_cell
    col_max = col_min + int(round(2 * visible_radius_m / grid_resolution_m))
    row_max = -int(round((camera_xy_m[1] - visible_radius_m) / grid_resolution_m)) + grid_origin_cell
    row_min = row_max - int(round(2 * visible_radius_m / grid_resolution_m))

    # Clip to grid bounds.
    src_col_min = max(0, col_min)
    src_col_max = min(cells, col_max)
    src_row_min = max(0, row_min)
    src_row_max = min(cells, row_max)
    if src_col_min >= src_col_max or src_row_min >= src_row_max:
        return

    sub = grid_image[src_row_min:src_row_max, src_col_min:src_col_max]
    if sub.size == 0:
        return

    # Build an RGB surface from the grayscale slice.
    h, w = sub.shape
    rgb = np.repeat(sub[:, :, None], 3, axis=2)
    surf = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))

    # Scale to pixels covering the visible window.
    target_w = int(w * grid_resolution_m * px_per_m)
    target_h = int(h * grid_resolution_m * px_per_m)
    if target_w <= 0 or target_h <= 0:
        return
    scaled = pygame.transform.scale(surf, (target_w, target_h))

    # Anchor: cell (src_row_min, src_col_min) -> world coords -> screen.
    anchor_world_x = (src_col_min - grid_origin_cell) * grid_resolution_m
    anchor_world_y = -(src_row_min - grid_origin_cell) * grid_resolution_m
    anchor_screen = (
        int(center[0] + (anchor_world_x - camera_xy_m[0]) * px_per_m),
        int(center[1] - (anchor_world_y - camera_xy_m[1]) * px_per_m),
    )
    surface.blit(scaled, anchor_screen)


# ---- live scan view --------------------------------------------------

@dataclass
class LiveView:
    """The current scan in body frame, colour-coded by distance."""

    show_grid: bool = True
    show_rings: bool = True

    def render(
        self,
        surface: pygame.Surface,
        font_hud: pygame.font.Font,
        font_ring: pygame.font.Font,
        scan: Optional[ScanRevolution],
        visible_radius_m: float,
    ) -> None:
        w, h = surface.get_size()
        center = (w // 2, h // 2)
        radius_px = min(w, h) // 2 - 80

        draw_rings_and_grid(
            surface, font_ring, center, radius_px,
            visible_radius_m, self.show_grid, self.show_rings,
        )

        count = 0
        max_d_m = 0.0
        if scan is not None and len(scan):
            for (x_m, y_m), d_mm in zip(scan.points_body_m, scan.distances_mm):
                if abs(x_m) > visible_radius_m or abs(y_m) > visible_radius_m:
                    continue
                col = color_for_distance(d_mm / 1000.0, visible_radius_m)
                spx = world_to_screen(x_m, y_m, visible_radius_m, center, radius_px)
                pygame.draw.circle(surface, col, spx, 2)
                count += 1
                if d_mm > max_d_m * 1000.0:
                    max_d_m = d_mm / 1000.0

        _draw_pose_marker(surface, center, 0.0)

        draw_text_lines(
            surface, font_hud,
            [
                ("View 2 / 3:    Live scan (body frame)", HUD_COLOR),
                (f"Points / scan: {count}", HUD_COLOR),
                (f"Max range:     {max_d_m:.2f} m", HUD_COLOR),
                (f"Visible:       {visible_radius_m:.2f} m", HUD_COLOR),
            ],
            origin=(12, 10),
        )


# ---- distance view ---------------------------------------------------

@dataclass
class DistanceView:
    """A numeric distance dashboard with 12 sectors and a polar bar plot."""

    sector_count: int = 12

    def render(
        self,
        surface: pygame.Surface,
        font_hud: pygame.font.Font,
        font_ring: pygame.font.Font,
        scan: Optional[ScanRevolution],
        visible_radius_m: float,
    ) -> None:
        w, h = surface.get_size()
        center = (w // 2, h // 2)
        radius_px = min(w, h) // 2 - 100

        draw_rings_and_grid(
            surface, font_ring, center, radius_px,
            visible_radius_m, True, True,
        )

        sector_min = self._sector_min_distances(scan)

        # Polar bar plot: each sector spans 360/sector_count degrees,
        # drawn as a filled wedge whose radial length is the min
        # distance in that sector.
        sector_width = 360.0 / self.sector_count
        for sector_idx, dist_m in enumerate(sector_min):
            if dist_m is None or not math.isfinite(dist_m):
                continue
            angle_centre_deg = sector_idx * sector_width
            angle_l = math.radians(angle_centre_deg - sector_width / 2)
            angle_r = math.radians(angle_centre_deg + sector_width / 2)
            r_px = min(radius_px, int(dist_m / visible_radius_m * radius_px))
            pts = [center]
            steps = max(2, int(sector_width / 3))
            for i in range(steps + 1):
                a = angle_l + (angle_r - angle_l) * i / steps
                pts.append((
                    int(center[0] + r_px * math.cos(a)),
                    int(center[1] - r_px * math.sin(a)),
                ))
            pygame.draw.polygon(surface, SECTOR_BAR_COLOR, pts)
            pygame.draw.polygon(surface, SECTOR_LINE_COLOR, pts, 1)

        # Numeric readout: 12 sectors labeled by their centre angle.
        hud_lines: List[Tuple[str, Tuple[int, int, int]]] = [
            ("View 3 / 3:    Distance dashboard", HUD_COLOR),
            ("", HUD_COLOR),
        ]
        for sector_idx, dist_m in enumerate(sector_min):
            angle_centre_deg = sector_idx * sector_width
            label = self._compass_label(angle_centre_deg)
            value = f"{dist_m:.2f} m" if dist_m is not None else "  no return"
            hud_lines.append(
                (f"  {angle_centre_deg:5.0f} deg  ({label:>3}):  {value}", SECTOR_TEXT_COLOR)
            )

        if scan is not None and len(scan):
            valid = scan.distances_mm[scan.distances_mm > 0]
            if valid.size:
                near_idx = int(np.argmin(valid))
                far_idx = int(np.argmax(valid))
                hud_lines.append(("", HUD_COLOR))
                hud_lines.append((
                    f"Closest: {valid[near_idx] / 1000.0:.2f} m at "
                    f"angle {scan.angles_deg[scan.distances_mm > 0][near_idx]:6.1f} deg",
                    SECTOR_TEXT_COLOR,
                ))
                hud_lines.append((
                    f"Furthest: {valid[far_idx] / 1000.0:.2f} m at "
                    f"angle {scan.angles_deg[scan.distances_mm > 0][far_idx]:6.1f} deg",
                    SECTOR_TEXT_COLOR,
                ))

        draw_text_lines(surface, font_hud, hud_lines, origin=(12, 10))

        _draw_pose_marker(surface, center, 0.0)

    def _sector_min_distances(
        self, scan: Optional[ScanRevolution],
    ) -> Sequence[Optional[float]]:
        """Return the minimum valid distance per sector, in metres."""
        if scan is None or len(scan) == 0:
            return [None] * self.sector_count
        sector_width = 360.0 / self.sector_count
        # Shift so sector 0 is centred on 0 degrees.
        shifted = (scan.angles_deg + sector_width / 2.0) % 360.0
        sector_idx = (shifted // sector_width).astype(int)
        mins: List[Optional[float]] = [None] * self.sector_count
        distances_m = scan.distances_mm / 1000.0
        for i in range(self.sector_count):
            mask = sector_idx == i
            if not mask.any():
                continue
            vals = distances_m[mask]
            vals = vals[vals > 0]
            if vals.size:
                mins[i] = float(vals.min())
        return mins

    @staticmethod
    def _compass_label(angle_deg: float) -> str:
        """8-point compass label for a bearing in degrees (CCW from +X)."""
        # +X is "east" by ROS convention; +Y is "north".
        # The sensor body frame uses 0 deg along the cable-out side
        # which we map to "front" for the readout's sake.
        labels = ["F", "FL", "L", "BL", "B", "BR", "R", "FR"]
        idx = int(((angle_deg + 22.5) % 360) // 45)
        return labels[idx]
