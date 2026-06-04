from collections import namedtuple
import re
from enum import Enum
import numpy as np
import matplotlib.pyplot as plt
from alive_progress import alive_bar


class IMUMsgType(Enum):
    ADIS = 'adis'
    MAG = 'mag'
    LPS28 = 'lps28'
    LPS22 = 'lps22'
    SYNC = 'sync'
    HOST = 'host'


# Parent class for message data, including imu, magnetometer, barometer, sync.
class MsgData:
    def __init__(self):
        self.filepath = None
        self.data = None
        self.name: str | None = None

    def file_parser(self, msg_filter: list = None) -> list[namedtuple]:
        data_re = re.compile(r'\$([a-zA-Z0-9,.\-:/\+]*)\*([a-zA-Z0-9]{2})')

        with open(self.filepath, 'r') as f:
            lines = f.readlines()

        results = []
        with alive_bar(len(lines), title=f"Parsing {self.name} file", length=60, max_cols=150, force_tty=True) as bar:
            for line in lines:
                res = self.line_parser(line, data_re, msg_filter)
                if res:
                    results.append(res)
                bar()
        return results

    def line_parser(self, line: str, data_re: re.Pattern, msg_filter: list = None):
        raise NotImplementedError("Line parser is not defined!")

    @staticmethod
    def checksum_xor(msg: str) -> str:
        checksum = 0
        msg_bytes = msg.encode()
        for b in msg_bytes:
            checksum = (checksum ^ b) & 0xff

        return f"{checksum:02x}".upper()

    def check_packet_loss(self, msg_sn_cycle: int, plot_diff: bool = False):
        msg_sn = np.array([data.msg_sn for data in self.data])
        msg_sn_diff = np.mod(np.diff(msg_sn), msg_sn_cycle)

        if np.any(msg_sn_diff != 1):
            print('Packet loss exists!')
        else:
            print('No packet loss detected.')

        if plot_diff:
            plt.plot(msg_sn_diff)
            plt.show()
