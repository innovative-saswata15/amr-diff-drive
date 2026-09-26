"""MAPPING: SLAM Toolbox only (drive with teleop, then save the map).

Usage:
  ros2 launch amr_navigation slam.launch.py
  ros2 launch amr_navigation slam.launch.py slam_params_file:=/abs/path/my_slam.yaml

Requires the robot bringup (ros2_control + diff_drive_controller, LiDAR driver,
robot_state_publisher) to be running already.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_navigation')

    slam_params = LaunchConfiguration('slam_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg, 'config', 'slam_toolbox_params.yaml'),
            description='SLAM Toolbox parameter file'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': False}],
        ),
    ])
