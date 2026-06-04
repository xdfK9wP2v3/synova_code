import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    radar_cpp_publisher_node = Node(
        package='radar_driver_cpp',
        executable='publisher',
        name='radar_publisher_cpp',
        output='screen',
        parameters=[{'awr2243_config_select': 'max15'}, ]
    )

    radar_cpp_subscriber_node = Node(
        package='radar_driver_cpp',
        executable='subscriber',
        name='radar_subscriber_cpp',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        radar_cpp_publisher_node,
        # radar_cpp_subscriber_node
    ])