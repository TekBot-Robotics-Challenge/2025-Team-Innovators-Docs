from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_data_evaluation',
            executable='publisher_node',
            name='sensor_publisher'
        ),
        Node(
            package='sensor_data_evaluation',
            executable='subscriber_node',
            name='sensor_subscriber'
        )
    ])
