from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nokolat2024',
            executable='imu_to_tf_node',
            name='imu_to_tf_node'
        )
    ])