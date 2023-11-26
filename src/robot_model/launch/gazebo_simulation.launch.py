from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the URDF/Xacro file
    pkg_share = get_package_share_directory('robot_model')
    default_model_path = os.path.join(pkg_share, 'urdf', 'simple_walker.urdf')

    # Launch argument to allow the user to specify the path to the URDF file
    urdf_launch_arg = DeclareLaunchArgument(
        'urdf_file', default_value=default_model_path,
        description='/home/ubuntu/JK/ASU_Classes/MAE547/Project_ws/src/robot_model/urdf/simple_walker.urdf')

    # Node to publish the robot state (joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')])}]
    )


    # Execute Gazebo with the empty world for starters
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Use the 'spawn_entity.py' script to spawn the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'biped_robot', '-file', LaunchConfiguration('urdf_file')],
        output='screen'
    )

    return LaunchDescription([
        urdf_launch_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity
    ])

