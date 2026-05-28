"""
Log-odds 2D occupancy grid.

Concept
-------
The world is a 2D grid of cells. Each cell stores a single
floating-point number, the log-odds of "this cell is occupied". When
a new scan arrives we update the grid:

- The straight line of cells between the sensor and the laser
  return is "probably free" (subtract from each cell).
- The cell where the laser landed is "probably occupied" (add to
  that cell).

Log-odds is the right representation here because additions and
subtractions correspond to multiplications and divisions of the
underlying probability, which is what Bayes' rule wants.

Conversions, for reference:
    log_odds = log(p / (1 - p))
    p = sigmoid(log_odds)

We clip log-odds to a fixed range so a long run of consistent
observations does not push a cell so far that it cannot recover when
the world changes.

Coordinate system
-----------------
The grid is centred at (0, 0) in world coordinates. Cell indices
(row, col) come from the helper `world_to_cell`. We deliberately put
row=0 at the top so the image we hand to pygame can be blitted
directly without a flip.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


# Default log-odds increments. Empirically reasonable for indoor
# LiDAR at 10 Hz: each hit nudges a cell strongly toward occupied,
# each free-traverse nudges weakly toward free. The asymmetry is
# what makes thin walls stick.
DEFAULT_LOG_ODDS_HIT = 0.85
DEFAULT_LOG_ODDS_MISS = -0.40
DEFAULT_LOG_ODDS_CLIP = 5.0


@dataclass
class OccupancyGrid:
    """A 2D log-odds occupancy grid centred at the world origin."""

    size_m: float = 30.0
    resolution_m: float = 0.05
    log_odds_hit: float = DEFAULT_LOG_ODDS_HIT
    log_odds_miss: float = DEFAULT_LOG_ODDS_MISS
    log_odds_clip: float = DEFAULT_LOG_ODDS_CLIP
    grid: np.ndarray = field(init=False)
    cells: int = field(init=False)
    origin_cell: int = field(init=False)

    def __post_init__(self) -> None:
        self.cells = int(round(self.size_m / self.resolution_m))
        self.origin_cell = self.cells // 2
        self.grid = np.zeros((self.cells, self.cells), dtype=np.float32)

    # -- coordinate conversions --------------------------------------

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        """Convert world metres to (row, col). Row increases downward."""
        col = int(round(x_m / self.resolution_m)) + self.origin_cell
        row = -int(round(y_m / self.resolution_m)) + self.origin_cell
        return row, col

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.cells and 0 <= col < self.cells

    def reset(self) -> None:
        self.grid.fill(0.0)

    # -- updates -----------------------------------------------------

    def integrate_scan(
        self,
        sensor_xy_m: tuple[float, float],
        hits_xy_m: np.ndarray,
    ) -> None:
        """Update the grid with one full scan.

        Parameters
        ----------
        sensor_xy_m : (x, y) in metres in the world frame
        hits_xy_m   : Nx2 array of laser-return endpoints in the world frame
        """
        if hits_xy_m.size == 0:
            return
        s_row, s_col = self.world_to_cell(*sensor_xy_m)
        for hit in hits_xy_m:
            h_row, h_col = self.world_to_cell(float(hit[0]), float(hit[1]))
            self._raytrace_free(s_row, s_col, h_row, h_col)
            if self.in_bounds(h_row, h_col):
                self.grid[h_row, h_col] = min(
                    self.log_odds_clip,
                    self.grid[h_row, h_col] + self.log_odds_hit,
                )

    def _raytrace_free(
        self, r0: int, c0: int, r1: int, c1: int,
    ) -> None:
        """Bresenham line from (r0, c0) to (r1, c1), excluding the endpoint.

        Each visited cell is nudged toward "free". The endpoint cell
        is left to the caller because the endpoint is a "hit".
        """
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0
        clip_neg = -self.log_odds_clip
        miss = self.log_odds_miss
        # Walk until the cell before the endpoint.
        while not (r == r1 and c == c1):
            if self.in_bounds(r, c):
                self.grid[r, c] = max(
                    clip_neg, self.grid[r, c] + miss,
                )
            e2 = err * 2
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

    # -- rendering ---------------------------------------------------

    def to_grayscale(self) -> np.ndarray:
        """Render the log-odds grid as a uint8 grayscale image.

        Convention: 0 = black = "occupied", 255 = white = "free",
        128 = gray = "unknown". Matches the convention used by RViz
        and ROS occupancy map tools.
        """
        prob = 1.0 / (1.0 + np.exp(-self.grid))  # sigmoid
        return np.clip(((1.0 - prob) * 255.0), 0, 255).astype(np.uint8)
