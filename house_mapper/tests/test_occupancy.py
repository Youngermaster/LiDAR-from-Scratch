"""
Tests for house_mapper.occupancy.

These check the small but easy-to-break parts:
  - world coordinate to cell index round-trip
  - cells along a ray are nudged toward "free"
  - the endpoint cell is nudged toward "occupied"
  - clipping prevents runaway accumulation
"""

from __future__ import annotations

import numpy as np

from house_mapper.occupancy import OccupancyGrid


def test_origin_maps_to_center_cell() -> None:
    grid = OccupancyGrid(size_m=10.0, resolution_m=0.1)
    row, col = grid.world_to_cell(0.0, 0.0)
    assert (row, col) == (grid.origin_cell, grid.origin_cell)


def test_in_bounds_clipping() -> None:
    grid = OccupancyGrid(size_m=2.0, resolution_m=0.1)
    # Origin is well in bounds.
    assert grid.in_bounds(grid.origin_cell, grid.origin_cell)
    # Far outside is not.
    assert not grid.in_bounds(-1, -1)
    assert not grid.in_bounds(grid.cells, grid.cells)


def test_integrate_marks_hit_occupied_and_ray_free() -> None:
    grid = OccupancyGrid(size_m=10.0, resolution_m=0.1)
    # Sensor at origin; one hit at (1, 0) m -> 10 cells along +X.
    grid.integrate_scan((0.0, 0.0), np.array([[1.0, 0.0]]))
    hit_row, hit_col = grid.world_to_cell(1.0, 0.0)
    # Endpoint cell is positive (occupied).
    assert grid.grid[hit_row, hit_col] > 0
    # A cell halfway along the ray is negative (free).
    mid_row, mid_col = grid.world_to_cell(0.5, 0.0)
    assert grid.grid[mid_row, mid_col] < 0
    # The sensor's own cell is also "free" since the ray starts there.
    origin_row, origin_col = grid.world_to_cell(0.0, 0.0)
    assert grid.grid[origin_row, origin_col] <= 0


def test_log_odds_clip() -> None:
    grid = OccupancyGrid(
        size_m=2.0, resolution_m=0.1,
        log_odds_hit=2.0, log_odds_clip=1.0,
    )
    # Hit the same point many times. Without clipping the cell value
    # would explode; with clipping it caps at log_odds_clip.
    hit = np.array([[0.5, 0.0]])
    for _ in range(20):
        grid.integrate_scan((0.0, 0.0), hit)
    hit_row, hit_col = grid.world_to_cell(0.5, 0.0)
    assert grid.grid[hit_row, hit_col] <= 1.0


def test_to_grayscale_shape() -> None:
    grid = OccupancyGrid(size_m=4.0, resolution_m=0.1)
    image = grid.to_grayscale()
    assert image.shape == (grid.cells, grid.cells)
    assert image.dtype == np.uint8


def test_reset_zeros_grid() -> None:
    grid = OccupancyGrid(size_m=4.0, resolution_m=0.1)
    grid.integrate_scan((0.0, 0.0), np.array([[1.0, 0.0]]))
    assert grid.grid.any()
    grid.reset()
    assert not grid.grid.any()
