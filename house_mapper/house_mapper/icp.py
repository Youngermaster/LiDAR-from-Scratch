"""
Point-to-point 2D ICP for scan matching.

Algorithm
---------
Given two point clouds, "source" (the new scan) and "target" (the
previous scan or a small local map), and an initial guess for the
rigid transform that takes source into the same frame as target:

1. Transform source by the current estimate.
2. For each transformed source point, find the nearest target point
   using a kd-tree.
3. Reject correspondences whose distance exceeds a threshold; this
   is the standard outlier filter for ICP and the one that keeps a
   handheld scan from snapping onto random walls.
4. Solve the closed-form rigid alignment between the matched
   source-target pairs using the SVD (Horn / Arun / Kabsch).
5. Iterate until the estimate stops changing.

The implementation uses `scipy.spatial.cKDTree` for nearest-neighbor
queries. Pure NumPy works for small scans but scales poorly.

Conventions
-----------
Both inputs are Nx2 arrays of x,y in metres. The returned pose
satisfies: `pose.transform_points(source) ~= target`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from .geometry import Pose2D


@dataclass
class IcpResult:
    pose: Pose2D
    iterations: int
    final_error: float          # mean squared distance over inlier pairs (m^2)
    inlier_count: int
    converged: bool


def icp_2d(
    source: np.ndarray,
    target: np.ndarray,
    initial_pose: Pose2D | None = None,
    *,
    max_iterations: int = 30,
    convergence_eps: float = 1e-5,
    max_correspondence_dist: float = 0.5,
    min_inliers: int = 20,
) -> IcpResult:
    """Estimate the rigid transform from source to target.

    Parameters
    ----------
    source, target : np.ndarray (N, 2) and (M, 2)
        Point clouds in metres.
    initial_pose : Pose2D, optional
        Starting guess. For consecutive LiDAR scans, pass the
        previous estimated pose so ICP only has to refine a small
        increment.
    max_iterations : int
        Hard cap on the iteration count.
    convergence_eps : float
        Stop when the per-iteration pose change drops below this
        (translation in metres, rotation in radians).
    max_correspondence_dist : float
        Reject point pairs farther apart than this, in metres. This
        is the outlier filter. Smaller values give cleaner matches
        but assume a better initial guess.
    min_inliers : int
        If we end up with fewer accepted pairs than this, abort.
        Returns the last good pose.

    Returns
    -------
    IcpResult
    """
    if source.ndim != 2 or source.shape[1] != 2:
        raise ValueError(f"source must be (N, 2), got {source.shape}")
    if target.ndim != 2 or target.shape[1] != 2:
        raise ValueError(f"target must be (N, 2), got {target.shape}")
    if source.size == 0 or target.size == 0:
        return IcpResult(
            pose=initial_pose or Pose2D(),
            iterations=0,
            final_error=float("inf"),
            inlier_count=0,
            converged=False,
        )

    pose = initial_pose if initial_pose is not None else Pose2D()
    tree = cKDTree(target)
    max_corr_sq = max_correspondence_dist * max_correspondence_dist

    last_error = float("inf")
    inlier_count = 0
    converged = False
    iteration = 0

    for iteration in range(1, max_iterations + 1):
        transformed = pose.transform_points(source)

        # Nearest-neighbour in target for each transformed source point.
        distances, indices = tree.query(transformed, k=1)
        dist_sq = distances * distances
        mask = dist_sq < max_corr_sq
        inlier_count = int(mask.sum())
        if inlier_count < min_inliers:
            break

        src_in = source[mask]
        tgt_in = target[indices[mask]]
        # Closed-form rigid alignment (Horn 1987 / Kabsch via SVD):
        # find R, t that minimise sum |R src_i + t - tgt_i|^2.
        src_mean = src_in.mean(axis=0)
        tgt_mean = tgt_in.mean(axis=0)
        src_c = src_in - src_mean
        tgt_c = tgt_in - tgt_mean
        H = src_c.T @ tgt_c
        U, _S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            # Ensure a proper rotation (det = +1), not a reflection.
            Vt[-1, :] *= -1.0
            R = Vt.T @ U.T
        t = tgt_mean - R @ src_mean
        new_pose = Pose2D(x=float(t[0]), y=float(t[1]),
                          theta=math.atan2(R[1, 0], R[0, 0]))

        # Convergence: pose change small AND error monotonically dropped.
        d_trans = math.hypot(new_pose.x - pose.x, new_pose.y - pose.y)
        d_rot = abs(_angle_diff(new_pose.theta, pose.theta))
        error = float(dist_sq[mask].mean())

        pose = new_pose
        if d_trans < convergence_eps and d_rot < convergence_eps:
            converged = True
            last_error = error
            break
        last_error = error

    return IcpResult(
        pose=pose,
        iterations=iteration,
        final_error=last_error,
        inlier_count=inlier_count,
        converged=converged,
    )


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    return (d + math.pi) % (2.0 * math.pi) - math.pi
