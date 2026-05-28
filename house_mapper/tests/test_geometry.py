"""
Tests for house_mapper.geometry.

These check the algebra: composition is associative, inversion is the
group inverse, and transform_points agrees with the matrix form. They
do NOT test floating-point edge cases or wrap-around precision; that
would be a different kind of test.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from house_mapper.geometry import Pose2D, polar_to_cartesian_mm


def test_default_pose_is_identity() -> None:
    p = Pose2D()
    points = np.array([[1.0, 2.0], [-3.0, 4.0]])
    out = p.transform_points(points)
    np.testing.assert_allclose(out, points)


def test_pure_translation() -> None:
    p = Pose2D(x=1.0, y=2.0, theta=0.0)
    points = np.array([[0.0, 0.0], [3.0, -1.0]])
    out = p.transform_points(points)
    expected = np.array([[1.0, 2.0], [4.0, 1.0]])
    np.testing.assert_allclose(out, expected)


def test_pure_rotation_90deg() -> None:
    p = Pose2D(theta=math.pi / 2)
    points = np.array([[1.0, 0.0], [0.0, 1.0]])
    out = p.transform_points(points)
    expected = np.array([[0.0, 1.0], [-1.0, 0.0]])
    np.testing.assert_allclose(out, expected, atol=1e-12)


def test_compose_then_inverse_is_identity() -> None:
    a = Pose2D(x=0.5, y=-1.2, theta=0.3)
    inv = a.inverse()
    composed = a.compose(inv)
    assert composed.x == pytest.approx(0.0, abs=1e-12)
    assert composed.y == pytest.approx(0.0, abs=1e-12)
    assert composed.theta == pytest.approx(0.0, abs=1e-12)


def test_compose_is_associative() -> None:
    a = Pose2D(x=1, y=0, theta=0.1)
    b = Pose2D(x=0, y=1, theta=-0.2)
    c = Pose2D(x=-1, y=-1, theta=0.4)
    left = a.compose(b).compose(c)
    right = a.compose(b.compose(c))
    assert left.x == pytest.approx(right.x, abs=1e-12)
    assert left.y == pytest.approx(right.y, abs=1e-12)
    assert left.theta == pytest.approx(right.theta, abs=1e-12)


def test_transform_points_agrees_with_compose() -> None:
    """Composing two poses must equal transforming the second's
    translation into the first's frame."""
    a = Pose2D(x=1.0, y=2.0, theta=math.radians(30))
    b = Pose2D(x=3.0, y=-1.0, theta=math.radians(10))
    composed = a.compose(b)
    # The translation of `composed` is a.transform of b's origin.
    b_origin_in_a = a.transform_points(np.array([[b.x, b.y]]))[0]
    assert composed.x == pytest.approx(b_origin_in_a[0])
    assert composed.y == pytest.approx(b_origin_in_a[1])


def test_polar_to_cartesian_basic() -> None:
    angles = np.array([0.0, 90.0, 180.0, 270.0])
    distances = np.array([1000.0, 1000.0, 1000.0, 1000.0])
    out = polar_to_cartesian_mm(angles, distances)
    expected = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]])
    np.testing.assert_allclose(out, expected, atol=1e-12)


def test_polar_to_cartesian_shape_mismatch() -> None:
    with pytest.raises(ValueError):
        polar_to_cartesian_mm(np.array([0.0, 90.0]), np.array([1000.0]))


def test_transform_points_empty() -> None:
    p = Pose2D(x=1.0, y=2.0, theta=0.5)
    out = p.transform_points(np.empty((0, 2)))
    assert out.shape == (0, 2)
