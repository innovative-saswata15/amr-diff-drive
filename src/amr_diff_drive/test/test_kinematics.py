import math

import pytest

from amr_diff_drive.kinematics import (
    integrate_pose,
    twist_to_wheels,
    wheels_to_twist,
)

R = 0.056
L = 0.39


def test_straight_unclamped():
    left, right = twist_to_wheels(0.5, 0.0, R, L, 18.0)
    assert left == pytest.approx(0.5 / R)
    assert right == pytest.approx(0.5 / R)


def test_straight_clamped_5mps():
    # 5 m/s -> 89.29 rad/s, clamped to 18 rad/s on both wheels
    left, right = twist_to_wheels(5.0, 0.0, R, L, 18.0)
    assert left == pytest.approx(18.0)
    assert right == pytest.approx(18.0)


def test_turn_in_place_ccw():
    left, right = twist_to_wheels(0.0, 1.0, R, L, 0.0)
    assert left == pytest.approx(-L / 2 / R)
    assert right == pytest.approx(L / 2 / R)


def test_clamp_preserves_curvature():
    v, w = 2.0, 3.0
    left, right = twist_to_wheels(v, w, R, L, 18.0)
    assert max(abs(left), abs(right)) == pytest.approx(18.0)
    v2, w2 = wheels_to_twist(left, right, R, L)
    assert v2 / w2 == pytest.approx(v / w)


def test_fk_ik_roundtrip():
    left, right = twist_to_wheels(0.3, -0.7, R, L, 0.0)
    v, w = wheels_to_twist(left, right, R, L)
    assert v == pytest.approx(0.3)
    assert w == pytest.approx(-0.7)


def test_integrate_straight():
    d = 1.0 / R  # wheel angle for 1 m
    x, y, th = integrate_pose(0, 0, 0, d, d, R, L)
    assert (x, y, th) == pytest.approx((1.0, 0.0, 0.0))


def test_integrate_full_circle_small_steps():
    # Arc of radius 1 m, CCW, full circle in 1000 steps -> back at origin
    radius, n = 1.0, 1000
    arc_total = 2 * math.pi * radius
    ds = arc_total / n
    dth = 2 * math.pi / n
    d_right = (ds + dth * L / 2) / R
    d_left = (ds - dth * L / 2) / R
    x = y = th = 0.0
    for _ in range(n):
        x, y, th = integrate_pose(x, y, th, d_left, d_right, R, L)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert th == pytest.approx(0.0, abs=1e-9)


def test_integrate_quarter_arc_single_step_exact():
    # Single big step: quarter circle radius 1 -> (1, 1, pi/2) exactly
    ds = math.pi / 2
    dth = math.pi / 2
    d_right = (ds + dth * L / 2) / R
    d_left = (ds - dth * L / 2) / R
    x, y, th = integrate_pose(0, 0, 0, d_left, d_right, R, L)
    assert (x, y, th) == pytest.approx((1.0, 1.0, math.pi / 2))
