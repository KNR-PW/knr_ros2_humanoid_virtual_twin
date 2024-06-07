import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'ros2_humanoid_virtual_twin'
    file_subpath = 'description/robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_gazebo_link_physics_coefficients': 'false', 'use_gazebo_joint_physics_coefficients': 'false', 'use_URDF_joint_dynamics_coefficients': 'false', 'use_gazebo': 'true'})


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': doc.toxml(),
        'use_sim_time': False}] # add other parameters here if required
    )

    node_jointpubGUI = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )    

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'launch/melman.rviz')],
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        node_jointpubGUI,
        node_rviz,
    ])
