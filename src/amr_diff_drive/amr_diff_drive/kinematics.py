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


def stamp_to_ns(sec, nanosec):
    """builtin_interfaces/Time fields -> integer nanoseconds."""
    return int(sec) * 1_000_000_000 + int(nanosec)


def select_sample_time(stamp_ns, receive_ns, last_stamp_ns, max_offset_ns):
    """Pick the time an encoder sample was measured.

    The sender's stamp is trusted only if it is non-zero, strictly newer than
    the previous message's stamp (last_stamp_ns, None if there is none), and
    within max_offset_ns of the local receive time (i.e. the clocks look
    synchronised). Otherwise the receive time is used.
    Returns (sample_ns, accepted, reason); reason is '' when accepted.
    """
    if stamp_ns <= 0:
        return receive_ns, False, 'zero stamp'
    if last_stamp_ns is not None and stamp_ns <= last_stamp_ns:
        return receive_ns, False, 'stamp not increasing'
    offset_ns = receive_ns - stamp_ns
    if abs(offset_ns) > max_offset_ns:
        return (receive_ns, False,
                f'stamp offset {offset_ns * 1e-9:.3f} s (ESP32 clock not synced to Pi?)')
    return stamp_ns, True, ''


def predict_pose(x, y, theta, left_vel, right_vel, age,
                 wheel_radius, wheel_separation, max_age):
    """Extrapolate a pose forward by `age` seconds at constant wheel speeds.

    Returns the pose unchanged when age <= 0 or age > max_age (no trustworthy
    prediction possible).
    """
    if age <= 0.0 or age > max_age:
        return x, y, theta
    return integrate_pose(x, y, theta, left_vel * age, right_vel * age,
                          wheel_radius, wheel_separation)
