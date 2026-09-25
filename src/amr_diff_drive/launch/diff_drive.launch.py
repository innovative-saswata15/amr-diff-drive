import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('amr_diff_drive'), 'config', 'diff_drive.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the diff drive parameter file')

    node = Node(
        package='amr_diff_drive',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, node])
