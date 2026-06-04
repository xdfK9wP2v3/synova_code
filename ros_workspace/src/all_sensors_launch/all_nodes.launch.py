import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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

    camera_publisher_node = Node(
        package='camera_driver',
        executable='publisher',
        name='camera_publisher',
        output='screen',
        parameters=[]
    )

    camera_publisher_node2 = Node(
        package='camera_driver2',
        executable='publisher',
        name='camera_publisher2',
        output='screen',
        parameters=[]
    )

    radar_publisher_node = Node(
        package='radar_driver_cpp',
        executable='publisher',
        name='radar_publisher',
        output='screen',
        parameters=[{'awr2243_config_select': 'max30'}, ]
    )

    # Lidar
    livox_launch_filepath = os.path.join(
        get_package_share_directory('livox_ros2_driver'),
        'launch',
        'livox_lidar_launch.py'
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_filepath)
    )

    cam_viewer_node = Node(
        package='viewer',
        executable='cam_viewer',
        name='cam_viewer',
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
        camera_publisher_node,
        camera_publisher_node2,
        livox_launch,
        radar_publisher_node,
        cam_viewer_node
    ])
