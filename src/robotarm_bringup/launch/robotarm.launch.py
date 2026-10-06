import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('robotarm_description')
    pkg_bringup = get_package_share_directory('robotarm_bringup')

    urdf_file = os.path.join(pkg_description, 'urdf', 'robotarm.urdf.xacro')
    controller_config = os.path.join(pkg_bringup, 'config', 'controllers.yaml')

    use_hardware = LaunchConfiguration('use_hardware')
    tool = LaunchConfiguration('tool')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_hardware',
            default_value='false',
            description='true: moteus hardware, false: mock_components',
        ),
        DeclareLaunchArgument(
            'tool',
            default_value='none',
            description='file name in robotarm_description/urdf/tools/ without .xacro',
        ),
    ]

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            urdf_file, ' ',
            'use_hardware:=', use_hardware, ' ',
            'tool:=', tool,
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='both',
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    teleop_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['teleop_controller', '--inactive'],
        output='screen'
    )

    cartesian_jog_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_jog_controller', '--inactive'],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        teleop_controller_spawner,
        cartesian_jog_controller_spawner,
    ])
