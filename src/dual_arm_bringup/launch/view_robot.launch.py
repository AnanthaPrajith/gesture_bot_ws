import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Find our description package
    urdf_file = os.path.join(get_package_share_directory('dual_arm_description'), 'urdf', 'dual_ur5.urdf.xacro')
    
    # Process the Xacro file
    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    return LaunchDescription([
        # The Robot State Publisher (calculates my TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        # The Joint State Publisher GUI (gives you sliders to move my arms!)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])