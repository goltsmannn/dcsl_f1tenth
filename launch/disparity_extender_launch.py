from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    de_yaml = os.path.join(
        get_package_share_directory('dcsl_f1tenth'),
        'config',
        'disparity_extender.yaml'
    )

    disparity_extender_node = Node(
        package='dcsl_f1tenth',   
        executable='disparity_extender',  # matches the entry point in setup.py
        name='disparity_extender',
        output='screen',
        parameters=[de_yaml]
    )

    return LaunchDescription([
        disparity_extender_node
    ])
