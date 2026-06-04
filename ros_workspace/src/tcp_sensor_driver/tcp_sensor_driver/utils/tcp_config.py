import os
from enum import Enum

class tcpIP(Enum):
    subnet_id = os.getenv('SUBNET', '12')
    RTK_NMEA = f'192.168.{subnet_id}.11'
    ADIS     = f'192.168.{subnet_id}.11'
    MAG      = f'192.168.{subnet_id}.11'
    LPS28    = f'192.168.{subnet_id}.11'
    LPS22    = f'192.168.{subnet_id}.11'
    SYNC     = f'192.168.{subnet_id}.12'
    RTK_RTCM = f'192.168.12.11'

class tcpPorts(Enum):
    RTK_NMEA = 5000
    ADIS     = 5001
    MAG      = 5002
    LPS28    = 5003
    LPS22    = 5004
    RTK_RTCM = 5005
    SYNC     = 6000

class trOutCh(Enum):
    TMS  = 1
    ADIS = 2
    RADAR = 7

class capInCh(Enum):
    ADIS  = 2
    MAG   = 3
    LPS28 = 4
    LPS22 = 5

class topicName(Enum):
    RTK    = '/rtk_data'
    ADIS   = '/adis_data'
    MAG    = '/mag_data'
    LPS22  = '/lps22_data'
    LPS28  = '/lps28_data'

class pkgCycle(Enum):
    RTK    = None
    ADIS   = 10000
    MAG    = 10000
    LPS28  = 10000
    LPS22  = 10000
    SYNC   = 128

class sensorFps(Enum):
    RTK    = 10
    ADIS   = 2000
    MAG    = 80
    LPS28  = 10
    LPS22  = 10

class FrameType(Enum):
    rtk = 'rtk_link'
    imu = 'imu_link'
    cam = 'cam_link'
    lid = 'lid_link'
    rad = 'rad_link'

TICKS_PER_SEC: int = 12000