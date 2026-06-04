import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Lidar
    livox_rviz_launch_filepath = os.path.join(
        get_package_share_directory('livox_ros2_driver'),
        'launch',
        'livox_lidar_rviz_launch.py'
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_rviz_launch_filepath)
    )

    # Camera
    camera_launch_filepath = os.path.join(
        get_package_share_directory('camera_driver'),
        'launch',
        'camera.launch.py'
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_filepath)
    )

    # IMU, RTK, Sync
    tcp_sensor_launch_filepath = os.path.join(
        get_package_share_directory('tcp_sensor_driver'),
        'launch',
        'tcp_sensors.launch.py'
    )

    tcp_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tcp_sensor_launch_filepath)
    )

    # Radar
    radar_launch_filepath = os.path.join(
        get_package_share_directory('radar_driver_cpp'),
        'launch',
        'radar_cpp.launch.py'
    )

    radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(radar_launch_filepath)
    )

    return LaunchDescription([
        camera_launch,
        livox_launch,
        tcp_sensor_launch,
        radar_launch
    ])