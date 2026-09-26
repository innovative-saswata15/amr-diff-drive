#!/usr/bin/env python3
"""cmd_vel relay: Nav2 output (/cmd_vel, geometry_msgs/Twist) -> diff_drive_controller.

Why this exists
---------------
On ROS 2 Humble, Nav2 publishes plain `Twist` on /cmd_vel, while the standard
ros2_control `diff_drive_controller` listens on its own namespaced topic:
  * use_stamped_vel: true  (Humble default) -> /diff_drive_controller/cmd_vel  (TwistStamped)
  * use_stamped_vel: false                  -> /diff_drive_controller/cmd_vel_unstamped (Twist)

This node bridges the two so the package works without editing the robot bringup.
TwistStamped messages are stamped with the current time, because the controller
compares the stamp against its cmd_vel_timeout (a zero stamp = "too old" = robot won't move).

Parameters
----------
  input_topic   (string) default "/cmd_vel"
  output_topic  (string) default "/diff_drive_controller/cmd_vel"
  stamped       (bool)   default true   -> publish TwistStamped; false -> Twist
  frame_id      (string) default "base_link" (only used when stamped)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')
        in_topic = self.declare_parameter('input_topic', '/cmd_vel').value
        out_topic = self.declare_parameter(
            'output_topic', '/diff_drive_controller/cmd_vel').value
        self.stamped = self.declare_parameter('stamped', True).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        if in_topic == out_topic:
            raise ValueError('input_topic and output_topic must differ (would loop)')

        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, out_topic, 10)
        self.create_subscription(Twist, in_topic, self.cb, 10)
        self.get_logger().info(
            f'Relaying {in_topic} (Twist) -> {out_topic} '
            f'({"TwistStamped" if self.stamped else "Twist"})')

    def cb(self, msg: Twist):
        if self.stamped:
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            out.twist = msg
            self.pub.publish(out)
        else:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
