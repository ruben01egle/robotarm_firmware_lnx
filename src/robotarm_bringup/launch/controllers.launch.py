"""
Spawners of all controllers in controllers.yaml, shared by robotarm.launch.py and the MuJoCo sim.

The joint_state_broadcaster starts active, every other controller is only loaded (inactive).
Hardware, mock and sim load the same set, so a controller switch that works in one works in all.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

INACTIVE_CONTROLLERS = [
    # upstream controllers chained to ff_controller
    'joint_stream_controller',
    'cartesian_jog_controller',
    'joint_trajectory_controller',
    # ff
    'ff_controller',
    # pure compliance
    'gc_controller',
    'cartesian_impedance_controller',
    # upstream controllers chained to cartesian_impedance_controller (chain_to in controllers.yaml)
    'joint_stream_controller_impedance',
    'cartesian_jog_controller_impedance',
]


def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    inactive_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--inactive'],
            output='screen',
        )
        for controller in INACTIVE_CONTROLLERS
    ]

    return LaunchDescription([joint_state_broadcaster_spawner, *inactive_spawners])
