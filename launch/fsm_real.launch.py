from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('orca_fsm')
    
    config_file = os.path.join(
        pkg_dir,
        'config',
        'real_params.yaml'
    )
    
    decision_node = Node(
        package='orca_fsm',
        executable='fsm25',
        name='auv_decision_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,  # For proper log output
        remappings=[
            # Additional remappings if needed
        ]
    )
    
    return LaunchDescription([
        decision_node,
    ])