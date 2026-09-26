"""NAVIGATION WHILE MAPPING: SLAM Toolbox + Nav2 (no saved map, no AMCL).

Easiest way to test Nav2 first: SLAM provides /map and map->odom, Nav2 plans on
the map as it is being built.

Usage:
  ros2 launch amr_navigation nav_slam.launch.py
  # if diff_drive_controller has use_stamped_vel: false
  ros2 launch amr_navigation nav_slam.launch.py relay_stamped:=false \
      relay_output_topic:=/diff_drive_controller/cmd_vel_unstamped
  # if your bringup already remaps the controller input to /cmd_vel
  ros2 launch amr_navigation nav_slam.launch.py use_relay:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('amr_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    slam_params = LaunchConfiguration('slam_params_file')
    use_relay = LaunchConfiguration('use_relay')
    relay_out = LaunchConfiguration('relay_output_topic')
    relay_stamped = LaunchConfiguration('relay_stamped')
    use_composition = LaunchConfiguration('use_composition')
    nav2_delay = LaunchConfiguration('nav2_start_delay')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': use_composition,
        }.items(),
    )

    relay = Node(
        package='amr_navigation',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        condition=IfCondition(use_relay),
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': relay_out,
            'stamped': ParameterValue(relay_stamped, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg, 'config', 'slam_toolbox_params.yaml'),
            description='SLAM Toolbox parameter file'),
        DeclareLaunchArgument(
            'use_relay', default_value='true',
            description='Start the /cmd_vel -> diff_drive_controller relay'),
        DeclareLaunchArgument(
            'relay_output_topic', default_value='/diff_drive_controller/cmd_vel',
            description='Topic diff_drive_controller listens on'),
        DeclareLaunchArgument(
            'relay_stamped', default_value='true',
            description='true = TwistStamped (use_stamped_vel: true), false = Twist'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Run Nav2 servers in one process (saves RAM, harder to debug)'),
        DeclareLaunchArgument(
            'nav2_start_delay', default_value='5.0',
            description='Seconds to wait after SLAM starts before starting Nav2'),

        slam,
        relay,
        TimerAction(period=nav2_delay, actions=[nav2]),
    ])
