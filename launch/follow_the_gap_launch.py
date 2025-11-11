from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ftg_yaml = os.path.join(
        get_package_share_directory('dcsl_f1tenth'),
        'config',
        'follow_the_gap.yaml'
    )

    follow_the_gap_node = Node(
        package='dcsl_f1tenth',   
        executable='follow_the_gap',  # matches the entry point in setup.py
        name='follow_the_gap',
        output='screen',
        parameters=[ftg_yaml]
    )

    return LaunchDescription([
        follow_the_gap_node
    ])
