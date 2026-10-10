import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('robotarm_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tool',
            default_value='none',
            description='file name in robotarm_description/urdf/tools/ without .xacro',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'robotarm.launch.py')),
            launch_arguments={
                'use_hardware': 'false',
                'tool': LaunchConfiguration('tool'),
            }.items(),
        ),
    ])
