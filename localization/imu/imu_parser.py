from tracker.sync_parser import *
import matplotlib.pyplot as plt
from tracker.parameters import *


class IMUData(MsgData):
    def __init__(self, filepath: str, sensor_name: str | None = 'IMU'):
        super().__init__()
        self.filepath = filepath
        self.name = sensor_name
        self.host_ts_start: int | None = None
        self.sync_cnt: int = 0
        self.msg_cnt: int = 0
        self.sync_idx: list[int] = []

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
            # $ADIS,5294,59527,279/10,C,0,gX,-1,gY,1,gZ,-36,aX,7,aY,800,aZ*44
            case 'ADIS', msg_sn, cntr, temperature, 'C', gX, 'gX', gY, 'gY', gZ, 'gZ', aX, 'aX', aY, 'aY', aZ, 'aZ':
                self.msg_cnt += 1

                res = {
                    'type': IMUMsgType.ADIS,
                    'sync_idx': np.int64(self.sync_cnt),
                    'msg_sn': int(msg_sn),
                    'cntr': int(cntr),
                    'temperature': float(temperature.split('/')[0]) / float(temperature.split('/')[1]),
                    'gyr': (np.deg2rad(float(gX) / 10), np.deg2rad(float(gY) / 10), np.deg2rad(float(gZ)) / 10),     # 10 LSB/ deg/s
                    'acc': (float(aX) / 800 * Params.g_n[2], float(aY) / 800 * Params.g_n[2], float(aZ) / 800 * Params.g_n[2]),     # 800 LSB/mg = 800 / 9.8 LSB / m/s^2
                }
                return namedtuple('ADIS', res.keys())(**res)

            # $LPS28,1236,03,1000.10,hPa,28.93,C*5A
            case 'LPS28', msg_sn, flags, P, 'hPa', T, 'C':
                self.msg_cnt += 1

                res = {
                    'type': IMUMsgType.LPS28,
                    'sync_idx': np.int64(self.sync_cnt),
                    'msg_sn': int(msg_sn),
                    'flags': flags,
                    'P': float(P),
                    'T': float(T),
                    'alt': float(IMUData.get_altitude(float(T), float(P))),
                    'ts': np.int64(0),
                }
                return namedtuple('LPS28', res.keys())(**res)

            # $LPS22,1233,03,1001.37,hPa,29.52,C*5D
            case 'LPS22', msg_sn, flags, P, 'hPa', T, 'C':
                self.msg_cnt += 1

                res = {
                    'type': IMUMsgType.LPS22,
                    'sync_idx': np.int64(self.sync_cnt),
                    'msg_sn': int(msg_sn),
                    'flags': flags,
                    'P': float(P),
                    'T': float(T),
                    'alt': float(IMUData.get_altitude(float(T), float(P))),
                    'ts': np.int64(0),
                }
                return namedtuple('LPS22', res.keys())(**res)

            case 'HOST', host_time, host_day, host_month, host_year, data_split:
                if self.host_ts_start is not None:
                    raise ValueError('More than one host time stamp message exist!')

                self.host_ts_start = (int(host_time[:2]) + 8) * 3600 + int(host_time[2:4]) * 60 + int(host_time[4:6]) + float(host_time.split('.')[-1]) * 1e-6

            case 'SYNC', sync_sn:
                self.sync_cnt += 1
                self.sync_idx.append(self.msg_cnt)
                return None

            case _:
                return None

    @staticmethod
    def get_altitude(tem: float | list, pre: float | list) -> float | list:
        p0 = 1013.25   # hPa
        alt = ((p0 / np.array(pre)) ** (1 / 5.257) - 1) * (np.array(tem) + 273.15) / 0.0065

        if isinstance(tem, list):
            return list(alt)
        else:
            return float(alt)

    def plot_baro_data(self):
        pre = [data.P for data in self.data]
        tem = [data.T for data in self.data]
        alt = IMUData.get_altitude(tem, pre)

        # plot pressure and altitude
        plt.plot(list(range(len(pre))), pre, 'C0', label="pressure")
        plt.legend(loc="upper left")
        plt.twinx()
        plt.plot(list(range(len(pre))), alt, 'C2', label="altitude")
        plt.legend(loc="upper right")
        plt.xlabel("index")
        plt.title('pressure & altitude')
        plt.show()

        # plot temperature and altitude
        plt.plot(list(range(len(pre))), tem, 'C1', label="temperature")
        plt.ylabel("temperature")
        plt.legend(loc="upper left")
        plt.twinx()
        plt.plot(list(range(len(pre))), alt, 'C2', label="altitude")
        plt.ylabel("altitude")
        plt.legend(loc="upper right")
        plt.xlabel("index")
        plt.title('temperature & altitude')
        plt.show()

    def plot_imu_data(self):
        gyr = np.array([data.gyr for data in self.data])
        plt.plot(gyr[:, 0], label='omega_x')
        plt.plot(gyr[:, 1], label='omega_y')
        plt.plot(gyr[:, 2], label='omega_z')
        plt.title('Angular velocity (rad/s)')
        plt.legend()
        plt.show()

        acc = np.array([data.acc for data in self.data])
        plt.plot(acc[:, 0], label='acc_x')
        plt.plot(acc[:, 1], label='acc_y')
        plt.plot(acc[:, 2], label='acc_z')
        plt.title('Acceleration (m/s^2)')
        plt.legend()
        plt.show()

        pass


if __name__ == '__main__':
    data_dir = ''
    data_date = ''
    data_seq: int = 1

    adis = IMUData(os.path.join(data_dir, data_date, f'{data_seq}_ADIS.log'))
    lps22 = IMUData(os.path.join(data_dir, data_date, f'{data_seq}_LPS22.log'))
    lps28 = IMUData(os.path.join(data_dir, data_date, f'{data_seq}_LPS28.log'))

    # adis_data = adis.data
    lps22_data = lps22.data
    lps28_data = lps28.data

    # adis.check_packet_loss(msg_sn_cycle=10_000, plot_diff=True)
    # lps22.check_packet_loss(msg_sn_cycle=10_000, plot_diff=True)
    # lps28.check_packet_loss(msg_sn_cycle=10_000, plot_diff=True)

    # adis.plot_imu_data()
    lps22.plot_baro_data()
    lps28.plot_baro_data()
    pass