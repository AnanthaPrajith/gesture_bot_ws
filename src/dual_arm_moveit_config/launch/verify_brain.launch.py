import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command 
from launch_ros.parameter_descriptions import ParameterValue 

def load_file(package_name, file_path):
    absolute_file_path = os.path.join(
        get_package_share_directory(package_name), file_path
    )
    with open(absolute_file_path, 'r') as f:
        return f.read()

def load_yaml(package_name, file_path):
    absolute_file_path = os.path.join(
        get_package_share_directory(package_name), file_path
    )
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('dual_arm_description'),
        'urdf', 'dual_ur5.urdf.xacro'
    )

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    srdf       = load_file('dual_arm_moveit_config', 'config/dual_ur5_bot.srdf')
    kinematics = load_yaml('dual_arm_moveit_config', 'config/kinematics.yaml')
    ompl_yaml  = load_yaml('dual_arm_moveit_config', 'config/ompl_planning.yaml')
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'ompl': ompl_yaml,      
    }
    
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {'robot_description': robot_description_content},
                {'robot_description_semantic': srdf},
                {'robot_description_kinematics': kinematics},
                planning_pipelines,
                {'publish_robot_description_semantic': True},
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'robot_description': robot_description_content},
                {'robot_description_semantic': srdf},
                {'robot_description_kinematics': kinematics},
                planning_pipelines,               
            ]
        )
    ])