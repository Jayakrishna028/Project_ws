import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('robot_model')
    urdf_file = os.path.join(pkg_dir,'urdf','simple_walker.urdf')

    #Launch Arguments
    urdf_launch_arg = DeclareLaunchArgument(
        'urdf_file', default_value=urdf_file,
        description='/home/ubuntu/JK/ASU_Classes/MAE547/Project_ws/src/robot_model/urdf/simple_walker.urdf'
    )
    
    # RViz node
    rviz_config_file = os.path.join(pkg_dir, 'launch', 'model_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    #joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name = 'robot_joint_state_publisher',
        parameters=[{
            'use_gui':True},
            {'rate': 30.0
        }]
    )

    return LaunchDescription([
        urdf_launch_arg,
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
