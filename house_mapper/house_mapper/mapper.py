"""
The Mapper class: glue between scan source, ICP, and occupancy grid.

A single `add_scan` call does the whole pipeline for one revolution:
  1. If this is the first scan, anchor the pose at the origin.
  2. Otherwise, run ICP between the new scan (in body frame) and the
     previous scan (in world frame, using the previous pose as the
     initial guess).
  3. If the ICP result looks reasonable, accept it; otherwise fall
     back to "no motion" so a bad scan match never throws the map.
  4. Project the scan into the world frame and integrate it into the
     occupancy grid.
  5. Record the new pose in the trajectory.

Threading
---------
The Mapper itself is not thread-safe. The app keeps reads behind a
lock and pushes integrations from a single reader thread, which is
enough for our use case. If we ever need multi-source fusion this
becomes the place to add an explicit queue.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .geometry import Pose2D
from .icp import icp_2d
from .occupancy import OccupancyGrid


# Reject ICP estimates that imply unrealistic motion between
# consecutive 10 Hz scans. At a brisk walk (1.5 m/s) one scan covers
# ~0.15 m and ~0.5 rad in worst-case spin. We give it 5x headroom.
MAX_TRANSLATION_PER_SCAN_M = 0.75
MAX_ROTATION_PER_SCAN_RAD = math.radians(45.0)


@dataclass
class MapperSnapshot:
    """A safe-to-render copy of the mapper's state at one instant."""

    pose: Pose2D
    trajectory: List[Pose2D]
    grid_image: np.ndarray              # uint8 grayscale image of the grid
    grid_size_m: float
    grid_resolution_m: float
    last_scan_world_m: np.ndarray       # Nx2 of the most recent scan in world frame
    scans_integrated: int
    last_icp_inliers: int
    last_icp_error_m2: float


@dataclass
class Mapper:
    """Coordinator: ICP-based pose estimation and occupancy mapping."""

    grid_size_m: float = 30.0
    grid_resolution_m: float = 0.05
    icp_max_correspondence_m: float = 0.5

    pose: Pose2D = field(default_factory=Pose2D)
    trajectory: List[Pose2D] = field(default_factory=lambda: [Pose2D()])
    grid: OccupancyGrid = field(init=False)
    _prev_scan_body: Optional[np.ndarray] = None
    scans_integrated: int = 0
    last_icp_inliers: int = 0
    last_icp_error_m2: float = 0.0

    def __post_init__(self) -> None:
        self.grid = OccupancyGrid(
            size_m=self.grid_size_m,
            resolution_m=self.grid_resolution_m,
        )

    def reset(self) -> None:
        """Clear the map and trajectory but keep the configuration."""
        self.pose = Pose2D()
        self.trajectory = [Pose2D()]
        self.grid.reset()
        self._prev_scan_body = None
        self.scans_integrated = 0
        self.last_icp_inliers = 0
        self.last_icp_error_m2 = 0.0

    def add_scan(self, points_body_m: np.ndarray) -> None:
        """Integrate one revolution. `points_body_m` is Nx2 in metres, body frame."""
        if points_body_m.size == 0:
            return

        if self._prev_scan_body is None:
            # First scan anchors the world. No ICP to run.
            scan_world = self.pose.transform_points(points_body_m)
            self.grid.integrate_scan((self.pose.x, self.pose.y), scan_world)
            self._prev_scan_body = points_body_m
            self.scans_integrated += 1
            return

        # Previous scan in world frame is the ICP target.
        prev_world = self.pose.transform_points(self._prev_scan_body)
        result = icp_2d(
            source=points_body_m,
            target=prev_world,
            initial_pose=self.pose,
            max_correspondence_dist=self.icp_max_correspondence_m,
        )
        self.last_icp_inliers = result.inlier_count
        self.last_icp_error_m2 = result.final_error

        if _pose_change_is_reasonable(self.pose, result.pose):
            new_pose = result.pose
        else:
            # ICP went off the rails. Keep the old pose so the map
            # at least does not gain a bogus jump. This sometimes
            # happens after a sudden motion or a brief occlusion.
            new_pose = self.pose

        scan_world = new_pose.transform_points(points_body_m)
        self.grid.integrate_scan((new_pose.x, new_pose.y), scan_world)
        self.pose = new_pose
        self.trajectory.append(new_pose)
        self._prev_scan_body = points_body_m
        self.scans_integrated += 1

    def snapshot(self) -> MapperSnapshot:
        """Build a read-only copy of the current state for rendering."""
        last_scan_world = (
            self.pose.transform_points(self._prev_scan_body)
            if self._prev_scan_body is not None
            else np.empty((0, 2), dtype=np.float64)
        )
        return MapperSnapshot(
            pose=self.pose,
            trajectory=list(self.trajectory),
            grid_image=self.grid.to_grayscale(),
            grid_size_m=self.grid_size_m,
            grid_resolution_m=self.grid_resolution_m,
            last_scan_world_m=last_scan_world,
            scans_integrated=self.scans_integrated,
            last_icp_inliers=self.last_icp_inliers,
            last_icp_error_m2=self.last_icp_error_m2,
        )


def _pose_change_is_reasonable(a: Pose2D, b: Pose2D) -> bool:
    dx = b.x - a.x
    dy = b.y - a.y
    dtheta = (b.theta - a.theta + math.pi) % (2.0 * math.pi) - math.pi
    return (
        math.hypot(dx, dy) <= MAX_TRANSLATION_PER_SCAN_M
        and abs(dtheta) <= MAX_ROTATION_PER_SCAN_RAD
    )
