import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robotarm_mujoco import mjcf
import yaml


def launch_setup(context):
    pkg_bringup = get_package_share_directory('robotarm_bringup')
    pkg_description = get_package_share_directory('robotarm_description')

    controller_config = os.path.join(pkg_bringup, 'config', 'controllers.yaml')
    plugin_config = os.path.join(mjcf.share_dir(), 'config', 'sim_plugins.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'urdf_config.rviz')

    tool = LaunchConfiguration('tool').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    params_file = LaunchConfiguration('sim_params').perform(context) or None
    pids_file = LaunchConfiguration('sim_pids').perform(context)
    # optional yaml on top of bringup's controllers.yaml, e.g. gains to try in the sim only
    controller_params = LaunchConfiguration('controller_params').perform(context)

    # regenerate the MJCF on every start, so it always matches the current URDF, tool and params
    ros_home = os.environ.get('ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros'))
    out_dir = os.path.join(ros_home, 'robotarm_mujoco')
    scene = mjcf.generate(out_dir, tool, params_file)

    # joint state rate as a controller parameter file, it has to reach the broadcaster's own node
    joint_state_config = os.path.join(out_dir, 'joint_state_rate.yaml')
    with open(joint_state_config, 'w') as f:
        yaml.safe_dump({'joint_state_broadcaster': {'ros__parameters': {
            'publish_rate': float(LaunchConfiguration('joint_state_rate').perform(context))}}}, f)

    controller_configs = [controller_config, plugin_config, joint_state_config]
    controller_configs += [controller_params] if controller_params else []

    robot_description = {
        'robot_description': ParameterValue(
            mjcf.xacro_urdf(tool, mujoco_model=scene, headless=headless,
                            pids_config_file=pids_file),
            value_type=str)
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

    # the same controllers as on the real robot and the mock
    controller_spawners = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'controllers.launch.py')))

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
        controller_spawners,
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
            'sim_pids',
            default_value='',
            description='position/velocity PID gains yaml, empty: installed config/sim_pids.yaml',
        ),
        DeclareLaunchArgument(
            'controller_params',
            default_value='',
            description='yaml applied after robotarm_bringup controllers.yaml (sim overrides)',
        ),
        DeclareLaunchArgument(
            'joint_state_rate',
            default_value='50',
            description='Hz, /joint_states rate. 500 for push_test, 50 is too coarse for it',
        ),
        OpaqueFunction(function=launch_setup),
    ])
