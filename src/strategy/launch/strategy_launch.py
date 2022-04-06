from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='strategy',
            namespace='strategy',
            executable='strategy_publisher',
            name='publisher'
        ),
        Node(
            package='strategy',
            namespace='strategy',
            executable='strategy_subscriber',
            name='subscriber'
        )
    ])