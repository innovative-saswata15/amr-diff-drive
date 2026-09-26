"""NAVIGATION ON A SAVED MAP: map_server + AMCL + Nav2 (normal operation).

Usage:
  ros2 launch amr_navigation nav_map.launch.py map:=/home/<user>/maps/lab.yaml
  # default map = <this package>/maps/map.yaml (copy your map there and rebuild)

After start: in RViz click "2D Pose Estimate" at the robot's real position,
then send goals with "Nav2 Goal".
Relay arguments are the same as in nav_slam.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('amr_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_relay = LaunchConfiguration('use_relay')
    relay_out = LaunchConfiguration('relay_output_topic')
    relay_stamped = LaunchConfiguration('relay_stamped')
    use_composition = LaunchConfiguration('use_composition')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
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
            'map', default_value=os.path.join(pkg, 'maps', 'map.yaml'),
            description='Full path to the saved map .yaml'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument('use_relay', default_value='true'),
        DeclareLaunchArgument(
            'relay_output_topic', default_value='/diff_drive_controller/cmd_vel'),
        DeclareLaunchArgument('relay_stamped', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),

        relay,
        nav2,
    ])
