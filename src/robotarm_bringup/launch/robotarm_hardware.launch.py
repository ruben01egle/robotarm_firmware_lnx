import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    pkg_description = get_package_share_directory('robotarm_description')
    pkg_bringup = get_package_share_directory('robotarm_bringup')
    
    urdf_file = os.path.join(pkg_description, 'urdf', 'robotarm.urdf.xacro')
    controller_config = os.path.join(pkg_bringup, 'config', 'controllers.yaml')

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ", 
            urdf_file, " ", 
            "use_hardware:=true"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
    ])