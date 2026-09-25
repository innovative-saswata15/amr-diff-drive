"""Pure differential-drive kinematics (no ROS dependencies, unit-testable).

Conventions (REP-103):
  +x forward, +y left, +yaw counter-clockwise.
  Wheel angular velocity positive => that wheel drives the robot forward.
"""

import math


def twist_to_wheels(v, w, wheel_radius, wheel_separation, max_wheel_speed=0.0):
    """Inverse kinematics: body twist (m/s, rad/s) -> wheel speeds (rad/s).

    If max_wheel_speed > 0 and either wheel exceeds it, BOTH wheels are scaled
    by the same factor, which preserves the path curvature (v/w ratio).
    Returns (left_rad_s, right_rad_s).
    """
    half = wheel_separation / 2.0
    left = (v - w * half) / wheel_radius
    right = (v + w * half) / wheel_radius

    if max_wheel_speed > 0.0:
        peak = max(abs(left), abs(right))
        if peak > max_wheel_speed:
            scale = max_wheel_speed / peak
            left *= scale
            right *= scale
    return left, right


def wheels_to_twist(left, right, wheel_radius, wheel_separation):
    """Forward kinematics: wheel speeds (rad/s) -> body twist (m/s, rad/s)."""
    v = wheel_radius * (right + left) / 2.0
    w = wheel_radius * (right - left) / wheel_separation
    return v, w


def integrate_pose(x, y, theta, d_left, d_right, wheel_radius, wheel_separation):
    """Advance the pose by wheel angle increments d_left/d_right (rad).

    Uses exact arc integration (the robot moves along a circular arc when both
    wheel speeds are constant over the interval), falling back to the
    midpoint (2nd-order Runge-Kutta) formula for near-straight motion to avoid
    dividing by ~0. Returns (x, y, theta) with theta wrapped to [-pi, pi].
    """
    ds = wheel_radius * (d_right + d_left) / 2.0
    dtheta = wheel_radius * (d_right - d_left) / wheel_separation

    if abs(dtheta) < 1e-6:
        mid = theta + dtheta / 2.0
        x += ds * math.cos(mid)
        y += ds * math.sin(mid)
    else:
        radius = ds / dtheta
        x += radius * (math.sin(theta + dtheta) - math.sin(theta))
        y -= radius * (math.cos(theta + dtheta) - math.cos(theta))

    theta = math.atan2(math.sin(theta + dtheta), math.cos(theta + dtheta))
    return x, y, theta


def yaw_to_quaternion(yaw):
    """Planar yaw -> (x, y, z, w) quaternion."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
