# Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    robot_name = "robot"
    package_name = "ros2_humanoid_virtual_twin"
    rviz_config = os.path.join(get_package_share_directory(
        package_name), "launch", "melman.rviz")
    robot_description = os.path.join(get_package_share_directory(
        package_name), "description", robot_name + ".urdf.xacro")
    doc = xacro.parse(open(robot_description))
    xacro.process_doc(doc, mappings={'use_gazebo_link_physics_coefficients': 'false', 'use_gazebo_joint_physics_coefficients': 'false', 'use_URDF_joint_dynamics_coefficients': 'false', 'use_gazebo': 'false'})
    params = {'robot_description': doc.toxml()}

    controller_config = os.path.join(
        get_package_share_directory(
            package_name), "controllers", "real_control.yaml"
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": doc.toxml()}, controller_config],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": doc.toxml()}],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )

    ])
