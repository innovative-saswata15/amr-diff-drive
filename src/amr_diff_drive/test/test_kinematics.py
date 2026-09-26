import math

import pytest

from amr_diff_drive.kinematics import (
    integrate_pose,
    predict_pose,
    select_sample_time,
    stamp_to_ns,
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


# ---------------- Encoder timestamps ----------------

NS = 1_000_000_000
MAX_OFF = NS // 2  # 0.5 s


def test_stamp_to_ns():
    assert stamp_to_ns(1790359259, 594341871) == 1790359259594341871


def test_select_valid_stamp():
    stamp = 1000 * NS
    t, ok, reason = select_sample_time(stamp, stamp + 60_000_000, stamp - 20_000_000, MAX_OFF)
    assert (t, ok, reason) == (stamp, True, '')


def test_select_first_message_valid():
    stamp = 1000 * NS
    t, ok, _ = select_sample_time(stamp, stamp + 1_000_000, None, MAX_OFF)
    assert (t, ok) == (stamp, True)


def test_select_zero_stamp():
    t, ok, reason = select_sample_time(0, 5 * NS, None, MAX_OFF)
    assert (t, ok, reason) == (5 * NS, False, 'zero stamp')


def test_select_repeated_stamp():
    stamp = 1000 * NS
    t, ok, reason = select_sample_time(stamp, stamp + 1000, stamp, MAX_OFF)
    assert (t, ok, reason) == (stamp + 1000, False, 'stamp not increasing')


def test_select_decreasing_stamp():
    stamp = 1000 * NS
    _, ok, reason = select_sample_time(stamp - 1, stamp + 1000, stamp, MAX_OFF)
    assert (ok, reason) == (False, 'stamp not increasing')


def test_select_boot_time_stamp_rejected():
    receive = 1790359259 * NS
    t, ok, reason = select_sample_time(12 * NS, receive, None, MAX_OFF)
    assert (t, ok) == (receive, False)
    assert 'not synced' in reason


def test_select_slightly_future_stamp_accepted():
    receive = 1000 * NS
    t, ok, _ = select_sample_time(receive + 5_000_000, receive, None, MAX_OFF)
    assert (t, ok) == (receive + 5_000_000, True)


def test_predict_zero_or_negative_age_unchanged():
    assert predict_pose(1.0, 2.0, 0.3, 10.0, 10.0, 0.0, R, L, 0.2) == (1.0, 2.0, 0.3)
    assert predict_pose(1.0, 2.0, 0.3, 10.0, 10.0, -0.01, R, L, 0.2) == (1.0, 2.0, 0.3)


def test_predict_age_above_cap_unchanged():
    assert predict_pose(1.0, 2.0, 0.3, 10.0, 10.0, 0.25, R, L, 0.2) == (1.0, 2.0, 0.3)


def test_predict_straight():
    x, y, th = predict_pose(0.0, 0.0, 0.0, 18.0, 18.0, 0.06, R, L, 0.2)
    assert (x, y, th) == pytest.approx((R * 18.0 * 0.06, 0.0, 0.0))


def test_predict_spin():
    wl, wr, age = -3.0, 3.0, 0.1
    x, y, th = predict_pose(0.0, 0.0, 0.0, wl, wr, age, R, L, 0.2)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-12)
    assert th == pytest.approx(R * (wr - wl) / L * age)
