import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sync_node = Node(
        package='tcp_sensor_driver',
        executable='sync_main',
        name='sync',
        output='screen',
        parameters=[]
    )

    rtk_node = Node(
    	package='tcp_sensor_driver',
    	executable='rtk_main',
    	name='rtk',
    	output='screen',
    	parameters=[]
    )

    adis_node = Node(
    	package='tcp_sensor_driver',
    	executable='adis_main',
    	name='adis',
    	output='screen',
    	parameters=[]
    )

    mag_node = Node(
        package='tcp_sensor_driver',
        executable='mag_main',
        name='mag',
        output='screen',
        parameters=[]
    )

    lps22_node = Node(
        package='tcp_sensor_driver',
        executable='lps22_main',
        name='lps22',
        output='screen',
        parameters=[]
    )

    lps28_node = Node(
        package='tcp_sensor_driver',
        executable='lps28_main',
        name='lps28',
        output='screen',
        parameters=[]
    )

    rtcm_node = Node(
        package='tcp_sensor_driver',
        executable='rtcm_main',
        name='rtcm',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        sync_node,
        adis_node,
        rtk_node,
        mag_node,
        lps22_node,
        lps28_node,
        rtcm_node
    ])