
import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    robot_name = "robot"
    package_name = "ros2_humanoid_virtual_twin"
    robot_description = os.path.join(get_package_share_directory(
        package_name), "description", robot_name + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(
            package_name), "controllers", "controllers.yaml"
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description",
                   "-entity", "Melman"],
        output="screen"
    )

    # Launching gazebo.launch.py is comsumed more than 1 minute somehow...
    joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "configured",
             "joint_state_broadcaster"],
        output="screen"
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "configured",
             "joint_trajectory_controller"],
        output="screen"
    )

    velocity_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "configured",
             "velocity_controller"],
        output="screen"
    )

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="both"),
            
        # Node(
        #     package="controller_manager",
        #     executable="ros2_control_node",
        #     parameters=[
        #         {"robot_description": robot_description_config.toxml()}, controller_config],
        #     output="both",
        # ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="both",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="both",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="both",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"]),
                launch_arguments={'pause': 'true'}.items()
        ),

        spawn_entity
    ])
