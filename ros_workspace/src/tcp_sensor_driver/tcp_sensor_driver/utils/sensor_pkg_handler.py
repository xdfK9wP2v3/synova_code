import numpy as np

from .device_time import SyncDeviceTime, SensorDeviceTime, DeviceTime
from .tcp_config import FrameType
from my_msgs.msg import TimeStamp, State, IMU, Mag, Baro, Rtk

class SensorPkgHandler:
    def __init__(self):
        pass

    @staticmethod
    def handle_rtk_pkg(rtk_msg: Rtk, msg: str) -> tuple[Rtk, State | None]:
        match msg.split(','):
            case 'GNGGA', utc_time, lat, lat_dir, lon, lon_dir, state, satellite_num, hdop_acc, altitude, 'M', undulation, 'M', rtcm_age, bs_id:
                # TODO: convert time to sync time base, set ts
                rtk_msg.frame_id = FrameType.rtk.value

                rtk_msg.utc_hour = int(utc_time[:2])
                rtk_msg.utc_min = int(utc_time[2:4])
                rtk_msg.utc_sec = float(utc_time[4:])

                rtk_msg.lat = {'S': -1, 'N': 1}[lat_dir] * (float(lat[:2]) + float(lat[2:]) / 60.0)
                rtk_msg.lon = {'W': -1, 'E': 1}[lon_dir] * (float(lon[:3]) + float(lon[3:]) / 60.0)
                rtk_msg.alt = float(altitude) + float(undulation)

                if bs_id:
                    rtk_msg.bs_id = int(bs_id)
                if rtcm_age:
                    rtk_msg.rtcm_age = float(rtcm_age)

                state_msg = State()
                state_msg.state = state
                rtk_msg.state = int(state)

                return rtk_msg, state_msg
            case 'GNHDT', heading, 'T':
                rtk_msg.heading = float(heading)
                return rtk_msg, None
            case _:
                return rtk_msg, None

    @staticmethod
    def handle_adis_pkg(msg: str, sync_ts: TimeStamp) -> IMU | None:
        match msg.split(','):
            case 'ADIS', msg_sn, msg_ts, cntr, temperature, 'C', gX, 'gX', gY, 'gY', gZ, 'gZ', aX, 'aX', aY, 'aY', aZ, 'aZ':
                gn = 9.79469967
                aX, aY, aZ = float(aX) / 800 * gn, float(aY) / 800 * gn, float(aZ) / 800 * gn  # 800 LSB/mg = 800 / 9.8 LSB / m/s^2
                gX, gY, gZ = np.deg2rad(float(gX) / 10), np.deg2rad(float(gY) / 10), np.deg2rad(float(gZ)) / 10  # 10 LSB/ deg/s

                adis_msg = IMU()
                adis_msg.ts = sync_ts
                adis_msg.frame_id = FrameType.imu.value

                adis_msg.acc.x = aX  # deg/s
                adis_msg.acc.y = aY  # deg/s
                adis_msg.acc.z = aZ  # deg/s

                adis_msg.gyr.x = gX
                adis_msg.gyr.y = gY
                adis_msg.gyr.z = gZ

                return adis_msg
            case _:
                return None

    @staticmethod
    def handle_mag_pkg(msg: str, sync_ts: TimeStamp) -> tuple[Mag, State] | None:
         match msg.split(','):
            case 'MAG', msg_sn, msg_ts, flags, temperature, 'C', X, 'X', Y, 'Y', Z, 'Z':
                mag_msg = Mag()
                mag_msg.ts = sync_ts
                mag_msg.frame_id = FrameType.imu.value

                mX, mY, mZ = float(X) * 0.1, float(Y) * 0.1, float(Z) * 0.1
                mag_msg.mag.x = mX  # LSB: 0.1 μT
                mag_msg.mag.y = mY
                mag_msg.mag.z = mZ

                state_msg = State()
                state_msg.state = flags

                return mag_msg, state_msg
            case _:
                return None

    @staticmethod
    def handle_lps22_pkg(msg: str, sync_ts: TimeStamp) -> tuple[Baro, State] | None:
        match msg.split(','):
            case 'LPS22', msg_sn, msg_ts, flags, pres, 'hPa', temp, 'C':
                pres, temp = float(pres), float(temp)
                p0 = 1013.25  # hPa
                alt = ((p0 / np.array(pres)) ** (1 / 5.257) - 1) * (np.array(temp) + 273.15) / 0.0065

                lps_msg = Baro()
                lps_msg.ts = sync_ts
                lps_msg.frame_id = FrameType.imu.value

                lps_msg.pres = pres    # hPa
                lps_msg.temp = temp    # C
                lps_msg.alt = alt

                state_msg = State()
                state_msg.state = flags

                return lps_msg, state_msg
            case _:
                return None

    @staticmethod
    def handle_lps28_pkg(msg: str, sync_ts: TimeStamp) -> tuple[Baro, State] | None:
        match msg.split(','):
            case 'LPS28', msg_sn, msg_ts, flags, pres, 'hPa', temp, 'C':
                pres, temp = float(pres), float(temp)
                p0 = 1013.25  # hPa
                alt = ((p0 / np.array(pres)) ** (1 / 5.257) - 1) * (np.array(temp) + 273.15) / 0.0065

                lps_msg = Baro()
                lps_msg.ts = sync_ts
                lps_msg.frame_id = FrameType.imu.value

                lps_msg.pres = pres  # hPa
                lps_msg.temp = temp  # C
                lps_msg.alt = alt

                state_msg = State()
                state_msg.state = flags

                return lps_msg, state_msg
            case _:
                return None