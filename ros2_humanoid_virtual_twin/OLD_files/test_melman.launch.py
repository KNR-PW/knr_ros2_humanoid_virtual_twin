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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    load_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )    

    load_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        # load_controller_manager,

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_controller_manager,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_trajectory_controller,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),

        node_robot_state_publisher,
        load_rviz,
    ])