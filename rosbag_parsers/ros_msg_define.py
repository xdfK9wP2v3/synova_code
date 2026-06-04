from collections import namedtuple

rtk_topic = '/rtk_data/sensor_msg'
adis_topic = '/adis_data/sensor_msg'
mag_topic = "/mag_data/sensor_msg"
lps28_topic = "/lps28_data/sensor_msg"
lps22_topic = "/lps22_data/sensor_msg"
camera_topic = "/cam/raw_img"
camera2_topic = "/cam2/raw_img"
radar_topic = '/radar/sensor_data'
lidar_topic = "/livox/lidar"

topic_map = {
    rtk_topic: 'rtk',
    adis_topic: 'adis',
    mag_topic: 'mag',
    lps28_topic: 'lps28',
    lps22_topic: 'lps22',
    camera_topic: 'camera',
    camera2_topic: 'camera2',
    radar_topic: 'radar',
    lidar_topic: 'lidar'
}

Timestamp = namedtuple('Timestamp',
                       ['pwr_sec', 'utc_sec', 'tick', 'tick_per_sec',
                        'subtick', 'subtick_per_tick', 'clk', 'clk_per_sec'])

# radar parameters
DATA_IN_PACKET = 1456
BYTES_IN_PACKET = 1466
