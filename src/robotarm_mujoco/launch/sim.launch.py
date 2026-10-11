import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robotarm_mujoco import mjcf


def launch_setup(context):
    pkg_bringup = get_package_share_directory('robotarm_bringup')
    pkg_description = get_package_share_directory('robotarm_description')

    controller_config = os.path.join(pkg_bringup, 'config', 'controllers.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'urdf_config.rviz')

    tool = LaunchConfiguration('tool').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    params_file = LaunchConfiguration('sim_params').perform(context) or None
    # optional yaml on top of bringup's controllers.yaml, e.g. gains to try in the sim only
    controller_params = LaunchConfiguration('controller_params').perform(context)
    controller_configs = [controller_config] + ([controller_params] if controller_params else [])

    # regenerate the MJCF on every start, so it always matches the current URDF, tool and params
    ros_home = os.environ.get('ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros'))
    scene = mjcf.generate(os.path.join(ros_home, 'robotarm_mujoco'), tool, params_file)

    robot_description = {
        'robot_description': ParameterValue(
            mjcf.xacro_urdf(tool, mujoco_model=scene, headless=headless), value_type=str)
    }
    sim_time = {'use_sim_time': True}
    # Mesa software OpenGL (llvmpipe) for the MuJoCo viewer and RViz: in the dev container,
    # hardware rendering through /dev/dri hangs on VS Code's forwarded X display
    gl_env = {}
    if LaunchConfiguration('software_rendering').perform(context) == 'true':
        gl_env = {'LIBGL_ALWAYS_SOFTWARE': '1'}

    # mujoco_ros2_control's own ros2_control_node (required by it, same parameters as upstream)
    control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        parameters=[robot_description, *controller_configs, sim_time],
        output='both',
        emulate_tty=True,
        additional_env=gl_env,
        on_exit=Shutdown(),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, sim_time],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # effort only: teleop and cartesian jog (position/velocity) cannot run in the sim
    cartesian_impedance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_impedance_controller', '--inactive'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[sim_time],
        output='screen',
        additional_env=gl_env,
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        cartesian_impedance_controller_spawner,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tool',
            default_value='none',
            description='file name in robotarm_description/urdf/tools/ without .xacro',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true: no MuJoCo viewer window',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='true: also start RViz',
        ),
        DeclareLaunchArgument(
            'software_rendering',
            default_value='true',
            description='true: Mesa software OpenGL for viewer and RViz (dev container needs it)',
        ),
        DeclareLaunchArgument(
            'sim_params',
            default_value='',
            description='sim parameter yaml, empty: the installed config/sim_params.yaml',
        ),
        DeclareLaunchArgument(
            'controller_params',
            default_value='',
            description='yaml applied after robotarm_bringup controllers.yaml (sim overrides)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
