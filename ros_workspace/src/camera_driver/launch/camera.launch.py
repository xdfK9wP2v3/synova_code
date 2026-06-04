import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_publisher_node = Node(
        package='camera_driver',
        executable='publisher',
        name='camera_publisher',
        output='screen',
        parameters=[]
    )

    camera_subscriber_node = Node(
        package='camera_driver',
        executable='subscriber',
        name='camera_subscriber',
        output='screen',
        parameters=[]
    )

    sync_node = Node(
        package='tcp_sensor_driver',
        executable='sync_main',
        name='sync',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        camera_publisher_node,
        sync_node
    ])