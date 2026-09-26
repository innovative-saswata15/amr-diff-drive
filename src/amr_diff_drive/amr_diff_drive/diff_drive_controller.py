"""Differential drive controller for the AMR.

Subscribes:
  /cmd_vel             geometry_msgs/Twist     (m/s, rad/s)
  /encoder_telemetry   sensor_msgs/JointState  (position rad, velocity rad/s)

Publishes (all at `publish_rate`, default 50 Hz, from a single timer):
  /left_vel            std_msgs/Float64        wheel command, rad/s
  /right_vel           std_msgs/Float64        wheel command, rad/s
  /odom                nav_msgs/Odometry       odom -> base_link
  /tf                  odom -> base_link transform
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from amr_diff_drive.kinematics import (
    integrate_pose,
    predict_pose,
    select_sample_time,
    stamp_to_ns,
    twist_to_wheels,
    wheels_to_twist,
    yaw_to_quaternion,
)


class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        # ---------------- Parameters ----------------
        p = self.declare_parameter
        self.wheel_radius = p('wheel_radius', 0.056).value            # m
        self.wheel_separation = p('wheel_separation', 0.39).value     # m
        self.max_wheel_speed = p('max_wheel_speed', 18.0).value       # rad/s, <=0 disables
        self.cmd_vel_timeout = p('cmd_vel_timeout', 0.5).value        # s
        self.publish_rate = p('publish_rate', 50.0).value             # Hz
        self.encoder_timeout = p('encoder_timeout', 0.5).value        # s, warning only
        self.max_wheel_delta = p('max_wheel_delta', 10.0).value       # rad per encoder msg
        self.use_encoder_stamp = p('use_encoder_stamp', True).value
        self.max_stamp_offset = p('max_stamp_offset', 0.5).value      # s
        self.max_extrapolation_time = p('max_extrapolation_time', 0.2).value  # s
        self.latency_report_period = p('latency_report_period', 10.0).value   # s, 0 disables
        self.left_joint = p('left_joint_name', 'left_wheel').value
        self.right_joint = p('right_joint_name', 'right_wheel').value
        self.invert_left_cmd = p('invert_left_cmd', False).value
        self.invert_right_cmd = p('invert_right_cmd', False).value
        self.invert_left_enc = p('invert_left_encoder', False).value
        self.invert_right_enc = p('invert_right_encoder', False).value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.publish_tf = p('publish_tf', True).value
        cmd_topic = p('cmd_vel_topic', '/cmd_vel').value
        enc_topic = p('encoder_topic', '/encoder_telemetry').value
        self.pose_cov_diag = list(p(
            'pose_covariance_diagonal',
            [0.001, 0.001, 1e6, 1e6, 1e6, 0.01]).value)
        self.twist_cov_diag = list(p(
            'twist_covariance_diagonal',
            [0.001, 0.001, 1e6, 1e6, 1e6, 0.01]).value)

        if self.wheel_radius <= 0.0 or self.wheel_separation <= 0.0:
            raise ValueError('wheel_radius and wheel_separation must be > 0')
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        if len(self.pose_cov_diag) != 6 or len(self.twist_cov_diag) != 6:
            raise ValueError('covariance diagonals must have 6 elements')
        if min(self.max_stamp_offset, self.max_extrapolation_time,
               self.latency_report_period) < 0.0:
            raise ValueError('max_stamp_offset, max_extrapolation_time and '
                             'latency_report_period must be >= 0')

        # ---------------- State ----------------
        self.cmd_left = 0.0
        self.cmd_right = 0.0
        self.last_cmd_time = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.last_enc_time = None       # local receive time (watchdog)

        # Encoder timing: the pose above is the pose at sample_ns.
        self.sample_ns = None           # measurement time of the stored pose
        self.last_stamp_ns = None       # previous non-zero header stamp
        self.left_vel_meas = 0.0        # rad/s, sign-corrected
        self.right_vel_meas = 0.0
        self.lat_min = math.inf         # receive - stamp stats, s
        self.lat_max = -math.inf
        self.lat_sum = 0.0
        self.lat_count = 0
        self.stamp_rejected = 0

        # ---------------- ROS interfaces ----------------
        self.left_pub = self.create_publisher(Float64, '/left_vel', 10)
        self.right_pub = self.create_publisher(Float64, '/right_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, cmd_topic, self.cmd_vel_cb, QoSProfile(depth=10))
        # Best-effort subscription is compatible with both reliable and
        # best-effort publishers.
        self.create_subscription(
            JointState, enc_topic, self.encoder_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)
        if self.use_encoder_stamp and self.latency_report_period > 0.0:
            self.create_timer(self.latency_report_period, self.report_latency)

        self.get_logger().info(
            f'diff_drive_controller up: r={self.wheel_radius} m, '
            f'L={self.wheel_separation} m, max_wheel_speed={self.max_wheel_speed} rad/s, '
            f'rate={self.publish_rate} Hz, {self.odom_frame}->{self.base_frame}, '
            f'cmd="{cmd_topic}", encoders="{enc_topic}", '
            f'use_encoder_stamp={self.use_encoder_stamp}')

    # ------------------------------------------------------------------
    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().warn('Non-finite cmd_vel ignored', throttle_duration_sec=1.0)
            return
        left, right = twist_to_wheels(
            v, w, self.wheel_radius, self.wheel_separation, self.max_wheel_speed)
        self.cmd_left = -left if self.invert_left_cmd else left
        self.cmd_right = -right if self.invert_right_cmd else right
        self.last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------
    def encoder_cb(self, msg: JointState):
        try:
            li = msg.name.index(self.left_joint)
            ri = msg.name.index(self.right_joint)
        except ValueError:
            self.get_logger().warn(
                f'Encoder msg missing "{self.left_joint}"/"{self.right_joint}" '
                f'(got {list(msg.name)})', throttle_duration_sec=2.0)
            return
        if len(msg.position) <= max(li, ri):
            self.get_logger().warn('Encoder msg has no positions', throttle_duration_sec=2.0)
            return

        left_pos = msg.position[li]
        right_pos = msg.position[ri]
        if not (math.isfinite(left_pos) and math.isfinite(right_pos)):
            self.get_logger().warn('Non-finite encoder position ignored',
                                   throttle_duration_sec=1.0)
            return
        if self.invert_left_enc:
            left_pos = -left_pos
        if self.invert_right_enc:
            right_pos = -right_pos

        # Wheel velocities for the odom twist (fallback: 0 if not provided).
        left_vel = right_vel = 0.0
        if len(msg.velocity) > max(li, ri):
            lv, rv = msg.velocity[li], msg.velocity[ri]
            if math.isfinite(lv) and math.isfinite(rv):
                left_vel = -lv if self.invert_left_enc else lv
                right_vel = -rv if self.invert_right_enc else rv

        now = self.get_clock().now()
        self.last_enc_time = now

        # When was this sample measured? ESP32 stamp if plausible, else now.
        receive_ns = now.nanoseconds
        if self.use_encoder_stamp:
            stamp_ns = stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
            self.sample_ns, accepted, reason = select_sample_time(
                stamp_ns, receive_ns, self.last_stamp_ns,
                int(self.max_stamp_offset * 1e9))
            if stamp_ns > 0:
                self.last_stamp_ns = stamp_ns
            if accepted:
                latency = (receive_ns - stamp_ns) * 1e-9
                self.lat_min = min(self.lat_min, latency)
                self.lat_max = max(self.lat_max, latency)
                self.lat_sum += latency
                self.lat_count += 1
            else:
                self.stamp_rejected += 1
                self.get_logger().warn(
                    f'Encoder stamp not used ({reason}); using receive time',
                    throttle_duration_sec=2.0)
        else:
            self.sample_ns = receive_ns
        self.left_vel_meas = left_vel
        self.right_vel_meas = right_vel

        # First sample: just latch the reference positions.
        if self.prev_left_pos is None:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.get_logger().info(
                f'First encoder sample: left={left_pos:.4f} rad, right={right_pos:.4f} rad')
            return

        d_left = left_pos - self.prev_left_pos
        d_right = right_pos - self.prev_right_pos
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos

        # Encoder reset / glitch: re-latch without moving the pose.
        if abs(d_left) > self.max_wheel_delta or abs(d_right) > self.max_wheel_delta:
            self.get_logger().warn(
                f'Encoder jump ignored (dL={d_left:.3f}, dR={d_right:.3f} rad); '
                'reference re-latched')
            return

        self.x, self.y, self.theta = integrate_pose(
            self.x, self.y, self.theta, d_left, d_right,
            self.wheel_radius, self.wheel_separation)
        self.v, self.w = wheels_to_twist(
            left_vel, right_vel, self.wheel_radius, self.wheel_separation)

    # ------------------------------------------------------------------
    def report_latency(self):
        """Periodic log of encoder latency (receive time - ESP32 stamp)."""
        total = self.lat_count + self.stamp_rejected
        if total == 0:
            return
        if self.lat_count > 0:
            self.get_logger().info(
                f'encoder latency ms: min {self.lat_min * 1e3:.1f} / '
                f'mean {self.lat_sum / self.lat_count * 1e3:.1f} / '
                f'max {self.lat_max * 1e3:.1f}, '
                f'stamps rejected {self.stamp_rejected}/{total}')
        else:
            self.get_logger().warn(
                f'encoder stamps rejected {self.stamp_rejected}/{total}; '
                'running on receive time (no latency compensation)')
        self.lat_min = math.inf
        self.lat_max = -math.inf
        self.lat_sum = 0.0
        self.lat_count = 0
        self.stamp_rejected = 0

    # ------------------------------------------------------------------
    def update(self):
        """50 Hz: publish wheel commands, odometry and TF."""
        now = self.get_clock().now()

        # --- Wheel commands (with cmd_vel watchdog) ---
        if (self.last_cmd_time is None or
                (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_vel_timeout):
            self.cmd_left = 0.0
            self.cmd_right = 0.0
        self.left_pub.publish(Float64(data=float(self.cmd_left)))
        self.right_pub.publish(Float64(data=float(self.cmd_right)))

        # --- Encoder staleness: report zero twist, keep last pose ---
        stale = (self.last_enc_time is None or
                 (now - self.last_enc_time).nanoseconds * 1e-9 > self.encoder_timeout)
        if stale:
            self.v = 0.0
            self.w = 0.0
            if self.last_enc_time is not None:
                self.get_logger().warn('Encoder telemetry stale', throttle_duration_sec=2.0)

        # --- Pose predicted from the measurement time to now (publish only;
        # never written back, so prediction errors cannot accumulate) ---
        x, y, theta = self.x, self.y, self.theta
        if self.use_encoder_stamp and not stale and self.sample_ns is not None:
            age = (now.nanoseconds - self.sample_ns) * 1e-9
            x, y, theta = predict_pose(
                x, y, theta, self.left_vel_meas, self.right_vel_meas, age,
                self.wheel_radius, self.wheel_separation,
                self.max_extrapolation_time)

        stamp = now.to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(theta)

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        for i in range(6):
            odom.pose.covariance[i * 7] = float(self.pose_cov_diag[i])
            odom.twist.covariance[i * 7] = float(self.twist_cov_diag[i])
        self.odom_pub.publish(odom)

        # --- TF odom -> base_link ---
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort: command a stop on shutdown.
        try:
            node.left_pub.publish(Float64(data=0.0))
            node.right_pub.publish(Float64(data=0.0))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
