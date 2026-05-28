"""
Tests for house_mapper.icp.

Pattern: generate a known point cloud (target), apply a known
ground-truth Pose2D to get the "source" cloud, run ICP, and check
the recovered pose matches.

We deliberately use a *scattered* point cloud, not parallel walls.
Point-to-point ICP on parallel walls suffers from "wall sliding":
a wall-aligned point under small rotation has its nearest neighbor
slightly shifted along the wall, which biases the recovered
rotation downward. Scattered features have no such ambiguity, so
the tests verify the algorithm itself rather than its sensitivity
to a specific scene geometry. The mapper's real-world performance
in featureless corridors is documented in the README.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from house_mapper.geometry import Pose2D
from house_mapper.icp import icp_2d


def _scattered_features(
    n_points: int = 80,
    half_extent_m: float = 3.0,
    seed: int = 42,
) -> np.ndarray:
    """Random points uniformly distributed in a square area, in metres."""
    rng = np.random.default_rng(seed)
    return rng.uniform(-half_extent_m, half_extent_m, size=(n_points, 2))


def test_icp_recovers_pure_translation() -> None:
    target = _scattered_features()
    truth = Pose2D(x=0.3, y=-0.2, theta=0.0)
    source = truth.inverse().transform_points(target)
    result = icp_2d(source, target)
    assert result.converged
    assert result.pose.x == pytest.approx(truth.x, abs=1e-3)
    assert result.pose.y == pytest.approx(truth.y, abs=1e-3)
    assert result.pose.theta == pytest.approx(truth.theta, abs=1e-3)


def test_icp_recovers_small_rotation() -> None:
    target = _scattered_features()
    truth = Pose2D(x=0.0, y=0.0, theta=math.radians(7.0))
    source = truth.inverse().transform_points(target)
    result = icp_2d(source, target)
    assert result.converged
    assert result.pose.theta == pytest.approx(truth.theta, abs=math.radians(0.5))


def test_icp_recovers_combined_motion_with_initial_guess() -> None:
    """With a reasonable initial guess, ICP handles larger motion."""
    target = _scattered_features()
    truth = Pose2D(x=0.4, y=0.3, theta=math.radians(10.0))
    source = truth.inverse().transform_points(target)
    guess = Pose2D(x=0.35, y=0.25, theta=math.radians(8.0))
    result = icp_2d(source, target, initial_pose=guess)
    assert result.pose.x == pytest.approx(truth.x, abs=1e-3)
    assert result.pose.y == pytest.approx(truth.y, abs=1e-3)
    assert result.pose.theta == pytest.approx(truth.theta, abs=math.radians(0.5))


def test_icp_identity_with_identical_clouds() -> None:
    cloud = _scattered_features()
    result = icp_2d(cloud.copy(), cloud)
    assert result.converged
    assert result.pose.x == pytest.approx(0.0, abs=1e-6)
    assert result.pose.y == pytest.approx(0.0, abs=1e-6)
    assert result.pose.theta == pytest.approx(0.0, abs=1e-6)


def test_icp_empty_clouds() -> None:
    result = icp_2d(np.empty((0, 2)), _scattered_features())
    assert result.inlier_count == 0
    assert not result.converged


def test_icp_rejects_bad_shape() -> None:
    with pytest.raises(ValueError):
        icp_2d(np.array([1.0, 2.0]), _scattered_features())
