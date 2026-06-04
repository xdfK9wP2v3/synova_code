import os
import pymap3d as pm
from tracker.msg_data_parser import *
from calibration.mag_calib_matlab import *
from datetime import datetime, timezone


class rtkMsgType(Enum):
    GNGGA = 'gngga'
    GNGST = 'gngst'
    GNRMC = 'gnrmc'
    GNVTG = 'gnvtg'
    GNHDT = 'gnhdt'


class GPS_State(Enum):
    Invalid = 0
    Single = 1
    Differential = 2
    PPS = 3
    RTK_Int = 4
    RTK_Float = 5
    IMU = 6
    User_Input = 7
    Simulation = 8


class RTKData(MsgData):
    def __init__(self, filepath: str, sensor_name: str = 'RTK'):
        super().__init__()
        self.filepath = filepath
        self.name = sensor_name
        self.data = self.file_parser()

    def line_parser(self, line: str, data_re: re.Pattern, msg_filter: list = None):
        nmea_f = data_re.search(line.strip())
        if nmea_f is None:
            return None

        if msg_filter is not None:
            if nmea_f.group(1).split(',')[0] not in msg_filter:
                return None

        if MsgData.checksum_xor(nmea_f.group(1)) != nmea_f.group(2):
            print(f"checksum failed: {line}")
            return None

        match nmea_f.group(1).split(','):
            case 'GNGGA', utc_time, lat, lat_dir, lon, lon_dir, state, satellite_num, hdop_acc, altitude, 'M', undulation, 'M', rtcm_age, bs_id:
                state = GPS_State(int(state))
                if state == GPS_State.Invalid:
                    return None

                utc_time = (int(utc_time[0:2]), int(utc_time[2:4]), float(utc_time[4:]))

                lat = {'S': -1, 'N': 1}[lat_dir] * (float(lat[:2]) + float(lat[2:]) / 60.0)
                lon = {'W': -1, 'E': 1}[lon_dir] * (float(lon[:3]) + float(lon[3:]) / 60.0)

                satellite_num = int(satellite_num)
                hdop_acc = float(hdop_acc)
                rtcm_age = float(rtcm_age) if rtcm_age else None

                altitude = float(altitude)
                undulation = float(undulation)
                height = altitude + undulation

                res = {
                    'type': rtkMsgType.GNGGA,
                    'utc_time': utc_time,
                    'geodetic': (lat, lon, height),
                    'latitude': lat,
                    'longitude': lon,
                    'state': state,
                    'satellite_num': satellite_num,
                    'hdop_acc': hdop_acc,
                    'altitude': altitude,
                    'undulation': undulation,
                    'height': height,
                    'rtcm_age': rtcm_age,
                    'bs_id': bs_id,
                }
                return namedtuple('GNGGA', res.keys())(**res)

            case 'GNGST', utc_time, rms, smjr_std, smnr_std, orient, lat_std, lon_std, alt_std:
                # $GNGST,205246.00,1.19,0.02,0.01,-2.4501,0.02,0.01,0.03*5B
                res = {
                    'type': rtkMsgType.GNGST,
                    'utc_time': (int(utc_time[0:2]), int(utc_time[2:4]), float(utc_time[4:])),
                    'rms': float(rms),
                    'std': (float(smjr_std), float(smnr_std), float(orient)),
                    'lat_std': float(lat_std),
                    'lon_std': float(lon_std),
                    'alt_std': float(alt_std),
                }
                return namedtuple('GNGST', res.keys())(**res)

            case 'GNRMC', utc_time, pos_status, lat, lat_dir, lon, lon_dir, speed_kn, track_true, date, mag_var, mag_var_dir, mode_ind, mode_status:
                utc_time = (int(utc_time[0:2]), int(utc_time[2:4]), float(utc_time[4:]))

                lat = {'S': -1, 'N': 1}[lat_dir] * (float(lat[:2]) + float(lat[2:]) / 60.0)
                lon = {'W': -1, 'E': 1}[lon_dir] * (float(lon[:3]) + float(lon[3:]) / 60.0)

                speed_kn = float(speed_kn)

                track_true = float(track_true)

                day, mon, year = (int(date[0:2]), int(date[2:4]), float(date[4:]))

                mag_var = {'W': 1, 'E': -1}[mag_var_dir] * float(mag_var)

                res = {
                    'type': rtkMsgType.GNRMC,
                    'utc_time': utc_time,
                    'pos_status': pos_status,
                    'geodetic': (lat, lon),
                    'latitude': lat,
                    'longitude': lon,
                    'speed_kn': speed_kn,
                    'track_true': track_true,
                    'utc_date': (year, mon, day),
                    'mag_var': mag_var,
                    'mode_ind': mode_ind,
                    'mode_status': mode_status,
                }

                return namedtuple('GNRMC', res.keys())(**res)

            case 'GNVTG', track_true, 'T', track_mag, 'M', speed_kn, 'N', speed_km, 'K', mode_ind:
                track_true = float(track_true)
                track_mag = float(track_mag)

                speed_kn = float(speed_kn)
                speed_km = float(speed_km)

                mode_ind = re.search(r"[A-Za-z]", mode_ind)[0]

                res = {
                    'type': rtkMsgType.GNVTG,
                    'track_true': track_true,
                    'track_mag': track_mag,
                    'speed_kn': speed_kn,
                    'speed_km': speed_km,
                    'mode_ind': mode_ind
                }

                return namedtuple('GNVTG', res.keys())(**res)

            case 'GNHDT', heading, 'T':
                heading = float(heading)

                res = {
                    'type': rtkMsgType.GNHDT,
                    'heading': heading
                }

                return namedtuple('GNHDT', res.keys())(**res)

            case _:
                return None

    @staticmethod
    def convert_pos_to_enu(data: list[namedtuple], reference: tuple):
        res = []
        for line in data:
            if hasattr(line, 'geodetic') and hasattr(line, 'utc_time'):
                pos_tuple = {'utc_time': line.utc_time,
                             'pos_enu': pm.geodetic2enu(*line.geodetic, *reference)}
                res.append(namedtuple('pos_enu'.upper(), pos_tuple.keys())(**pos_tuple))
        return res

    @staticmethod
    def convert_pos_cov_to_enu(data: list[namedtuple]) -> list:
        res = []
        for line in data:
            if hasattr(line, 'std'):
                theta = np.deg2rad(line.std[-1])
                R = np.array([[np.sin(theta), np.cos(theta), 0],
                              [np.cos(theta), -np.sin(theta), 0],
                              [0, 0, 1]])
                geodetic_std = np.diag([line.std[0], line.std[1], line.alt_std]) ** 2
                pos_cov_tuple = {'utc_time': line.utc_time,
                                 'pos_cov_enu': R @ geodetic_std @ R.T
                                 }
                res.append(namedtuple('pos_cov_enu'.upper(), pos_cov_tuple.keys())(**pos_cov_tuple))

        return res

    @staticmethod
    def get_enu_data(bs_data: list, mob_data: list, sync_utc_ts_start: int) -> dict:
        output = dict()
        for t in rtkMsgType:
            data = list(filter(lambda s: s.type == t, bs_data))
            if len(data) > 0:
                output[f'bs_{t.value}'] = data

            data = list(filter(lambda s: s.type == t, mob_data))
            if len(data) > 0:
                output[f'mob_{t.value}'] = data

        # align the timestamp of mobile station's GNGGA & GNGST messages
        start_time = max(output['mob_gngga'][0].utc_time, output['mob_gngst'][0].utc_time)
        end_time = min(output['mob_gngga'][-1].utc_time, output['mob_gngst'][-1].utc_time)
        output['mob_gngga'] = [item for item in output['mob_gngga'] if start_time <= item.utc_time <= end_time]
        output['mob_gngst'] = [item for item in output['mob_gngst'] if start_time <= item.utc_time <= end_time]

        output['bs_pos_enu'] = RTKData.convert_pos_to_enu(output['bs_gngga'], output['bs_gngga'][0].geodetic)
        output['mob_pos_enu'] = RTKData.convert_pos_to_enu(output['mob_gngga'], output['bs_gngga'][0].geodetic)
        output['mob_pos_cov_enu'] = RTKData.convert_pos_cov_to_enu(output['mob_gngst'])

        # align the UTC timestamps with the sync start UTC time
        sync_start = datetime.fromtimestamp(sync_utc_ts_start, tz=timezone.utc)
        sync_start_hms = (sync_start.hour, sync_start.minute, sync_start.second)
        output['ts'] = []

        for gngga_utc in output['mob_gngga']:
            assert gngga_utc.utc_time > sync_start_hms

            ts = ((gngga_utc.utc_time[0] - sync_start_hms[0]) * 3600 + (gngga_utc.utc_time[1] - sync_start_hms[1]) * 60 + (gngga_utc.utc_time[2] - sync_start_hms[2])) * 1e9
            output['ts'].append(ts)

        return output



if __name__ == '__main__':
    data_dir = ''
    data_date = ''
    data_seq: int = 2

    mob_filepath = os.path.join(data_dir, data_date, f'{data_seq}_RTK_NMEA.log')
    bs_filepath = os.path.join(data_dir, data_date, 'bs.log')

    rtk_mob = RTKData(mob_filepath)
    rtk_bs = RTKData(bs_filepath)

    rtk_data = RTKData.get_enu_data(bs_data=rtk_bs.data, mob_data=rtk_mob.data, sync_utc_ts_start=1732176000)
    pass