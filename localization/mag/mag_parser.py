from tracker.msg_data_parser import *
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from calibration.mag_calib_matlab import *


class MAGData(MsgData):
    def __init__(self, filepath: str, sensor_name: str = 'MAG'):
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
            # $MAG,0426,0F,15,C,-1990,X,1981,Y,2687,Z*07
            case 'MAG', msg_sn, ts, flags, temperature, 'C', X, 'X', Y, 'Y', Z, 'Z':
                self.msg_cnt += 1

                res = {
                    'type': IMUMsgType.MAG,
                    'sync_idx': np.int64(self.sync_cnt),
                    'msg_sn': int(msg_sn),
                    'flags': flags,
                    'temperature': float(temperature),
                    'mag': (float(X) * 0.1, float(Y) * 0.1, float(Z) * 0.1),  # LSB: 0.1 μT
                }

                return namedtuple('MAG', res.keys())(**res)

            # $HOST,065519.489764,03,12,2024,1*39
            case 'HOST', host_time, host_day, host_month, host_year, data_split:
                if self.host_ts_start is not None:
                    raise ValueError('More than one host time stamp message exist!')

                self.host_ts_start = (int(host_time[:2]) + 8) * 3600 + int(host_time[2:4]) * 60 + int(host_time[4:6]) + float(host_time.split('.')[-1]) * 1e-6

                # res = {
                #     'type': IMUMsgType.HOST,
                #     'host_hour': int(host_time[:2]) + 8,
                #     'host_min': int(host_time[2:4]),
                #     'host_sec': int(host_time[4:6]),
                #     'host_us': int(host_time.split('.')[-1]),
                # }
                # return namedtuple('HOST', res.keys())(**res)

            case 'SYNC', sync_sn:
                self.sync_idx.append(self.msg_cnt)


    def get_calibrated_data(self):
        mag_val = np.array([np.array(data.mag) for data in self.data])
        A, b, expMFS = magcal(mag_val)
        mag_val_cal = np.einsum('ij,kj->ik', mag_val - b, A)

        return mag_val_cal

    @staticmethod
    def plot_mag_data(mag_val: np.ndarray) -> None:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(mag_val[:, 0], mag_val[:, 1], mag_val[:, 2], alpha=0.1)
        ax.set_xlabel('mag_X')
        ax.set_ylabel('mag_Y')
        ax.set_zlabel('mag_Z')
        plt.title('Mag raw data')
        plt.show()

    @staticmethod
    def plot_calibrated_mag_fitting(data_cal: np.ndarray, sphere_r: float) -> None:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        mask = np.linalg.norm(data_cal, axis=-1) > sphere_r
        ax.scatter(data_cal[mask, 0], data_cal[mask, 1], data_cal[mask, 2], 'C1')
        ax.scatter(data_cal[~mask, 0], data_cal[~mask, 1], data_cal[~mask, 2], 'C3')

        h = np.arange(-sphere_r, sphere_r, 0.05)
        r = np.sqrt(sphere_r ** 2 - h ** 2)
        theta = np.arange(0, 2 * np.pi, 0.05)
        ax.plot_surface(np.outer(np.cos(theta), r), np.outer(np.sin(theta), r), np.outer(np.ones(len(theta)), h), color='g', alpha=0.1)

        ax.set_box_aspect(aspect=(1, 1, 1))
        ax.set_xlabel('mag_X')
        ax.set_ylabel('mag_Y')
        ax.set_zlabel('mag_Z')
        plt.title('Calibrated mag data')
        plt.show()


if __name__ == '__main__':
    mag_filepath = ''

    mag = MAGData(mag_filepath)
    mag_val = np.array([np.array(data.mag) for data in mag.data if hasattr(data, 'mag')])

    mag.check_packet_loss(msg_sn_cycle=10_000, plot_diff=False)

    MAGData.plot_mag_data(mag_val)

    A, b, expMFS = magcal(mag_val)  # Calib data: A: 3x3 matrix, b: 3x1 vector, expMFS: expected magnetic field strength
    print(f"expMFS:\n{expMFS}")
    print(f'A:\n{A}\nb:\n{b}')

    np.savez('mag_ego_calib.npz', A=A, b=b, expMFS=expMFS)

    mag_val_cal = np.einsum('ij,kj->ik', mag_val - b, A)
    MAGData.plot_calibrated_mag_fitting(mag_val_cal, expMFS)

    # plot_mag_data(mag_val_cal)

    pass