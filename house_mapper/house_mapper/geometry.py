"""
Geometry primitives: a 2D pose and the transforms we use everywhere.

The world frame in this project is right-handed: +X to the right,
+Y up (as drawn on screen, axes are flipped in pixel space because
screen Y points down). Angles are stored in radians internally; the
sensor reports degrees, so the polar-to-cartesian helper here
converts on the boundary.

Operations are implemented in NumPy where they will be called on
whole scans (`Pose2D.transform_points`, `polar_to_cartesian`) and in
plain Python where they will be called on single poses
(`Pose2D.compose`, `Pose2D.inverse`). Mixing is intentional: the
single-pose path stays readable, the bulk path stays fast.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Pose2D:
    """A 2D pose: translation (x, y) in metres and orientation theta in radians.

    Composition follows the standard "rigid body" convention: if `a`
    is the pose of frame A in frame W, and `b` is the pose of frame B
    in frame A, then `a.compose(b)` is the pose of frame B in frame W.
    """

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    def compose(self, other: "Pose2D") -> "Pose2D":
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        return Pose2D(
            x=self.x + cos_t * other.x - sin_t * other.y,
            y=self.y + sin_t * other.x + cos_t * other.y,
            theta=_wrap_pi(self.theta + other.theta),
        )

    def inverse(self) -> "Pose2D":
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        # Inverse of (R, t) is (R^T, -R^T t).
        inv_x = -(cos_t * self.x + sin_t * self.y)
        inv_y = -(-sin_t * self.x + cos_t * self.y)
        return Pose2D(x=inv_x, y=inv_y, theta=-self.theta)

    def transform_points(self, points_xy: np.ndarray) -> np.ndarray:
        """Apply this pose to an Nx2 array of points.

        Equivalent to (R @ p) + t for each point. Returns a new Nx2
        array; the input is not modified.
        """
        if points_xy.size == 0:
            return points_xy
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        # 2x2 rotation matrix applied via column stacking.
        rotated_x = cos_t * points_xy[:, 0] - sin_t * points_xy[:, 1]
        rotated_y = sin_t * points_xy[:, 0] + cos_t * points_xy[:, 1]
        out = np.empty_like(points_xy)
        out[:, 0] = rotated_x + self.x
        out[:, 1] = rotated_y + self.y
        return out


def _wrap_pi(theta: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


def polar_to_cartesian_mm(
    angles_deg: np.ndarray,
    distances_mm: np.ndarray,
) -> np.ndarray:
    """Convert polar samples (degrees, millimetres) to cartesian metres.

    The result is Nx2; column 0 is x_m, column 1 is y_m. Both inputs
    must be 1-D arrays of the same length. Use this on whole scans
    rather than calling `math.cos` per sample.
    """
    if angles_deg.shape != distances_mm.shape:
        raise ValueError(
            f"shape mismatch: angles {angles_deg.shape} vs distances "
            f"{distances_mm.shape}"
        )
    angles_rad = np.radians(angles_deg)
    distances_m = distances_mm * 0.001
    out = np.empty((angles_rad.shape[0], 2), dtype=np.float64)
    out[:, 0] = distances_m * np.cos(angles_rad)
    out[:, 1] = distances_m * np.sin(angles_rad)
    return out
