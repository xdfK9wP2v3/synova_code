import pathlib
import numpy as np
from struct import Struct
from collections import namedtuple
from alive_progress import alive_bar


class LVXReader:
    lvx_file_header = Struct('=16s 4B I I B')
    livox_device_header = Struct('=16s 16s B B B 6f')
    frame_header = Struct('=3Q')
    packet_header = Struct('=5B I B B Q')
    packet_struct = [
        (100, Struct('=3i B')),  # LvxRawPoint (type-0)
        (100, Struct('=i 2H B')),  # LvxSpherePoint (type-1)
        (96, Struct('=3i 2B')),  # LvxExtendedRawPoint (type-2)
        (96, Struct('=i 2H 2B')),  # LvxExtendedSpherePoint (type-3)
        (48, Struct('=3i 2B 3i 2B')),  # LvxDualExtendedRawPoint (type-4)
        (48, Struct('=3i 2B i 2B')),  # LvxDualExtendedSpherePoint (type-5)
        (1, Struct('=6f')),  # LvxImuPoint (type-6)
    ]
    packet_return = [
        None,  # (type-0)
        None,  # (type-1)
        namedtuple('Type_2', ['header', 'points', 'reflectivity', 'tag']),   # (type-2)
        None,  # (type-3)
        None,  # (type-4)
        None,  # (type-5)
        None,  # (type-6)
    ]

    utc_struct = Struct('4B I')

    PacketHeader = namedtuple('PacketHeader', ['device_index', 'version', 'port_id', 'lidar_index', 'reserved', 'error_code', 'timestamp_type', 'data_type', 'timestamp_raw', 'timestamp'])

    def __init__(self, filename: str | pathlib.Path):
        self.filename = filename

    def __enter__(self):
        self.fp = open(self.filename, 'rb')

        self.fp.seek(0, 2)
        end = self.fp.tell()
        self.fp.seek(0)

        *_, self.frame_duration, device_count = self.lvx_file_header.unpack(self.fp.read(self.lvx_file_header.size))

        for _ in range(device_count):
            self.fp.read(self.livox_device_header.size)

        self.packets: list[list[tuple[int, int, int, int]]] = [list() for _ in range(7)]
        timestamp = None
        with alive_bar(title="indexing packages from .lvx file", manual=True, length=60, max_cols=150, force_tty=True) as bar:
            while self.fp.tell() < end:
                current_offset, next_offset, frame_index = self.frame_header.unpack(self.fp.read(self.frame_header.size))
                assert current_offset == self.fp.tell() - 3 * 8

                while self.fp.tell() < next_offset:
                    packet_pos = self.fp.tell()

                    *_, timestamp_type, data_type, timestamp_raw = self.packet_header.unpack(self.fp.read(self.packet_header.size))

                    match timestamp_type:
                        case 0:
                            timestamp = timestamp_raw
                        case 3:
                            timestamp_raw = self.utc_struct.unpack(timestamp_raw.to_bytes(8, 'little'))
                            year, month, day, hour, us = timestamp_raw
                            assert len(self.packets[data_type]) == 0 or timestamp_raw[:2] == self.packets[data_type][-1][2][:2]
                            timestamp = ((day * 24 + hour) * 60 * 60 * 1e6 + us) * 1e3
                        case 4:
                            if len(self.packets[data_type]) == 0:  # pps is not available
                                timestamp = timestamp_raw
                            else:
                                *_, prev_timestamp_raw, prev_timestamp = self.packets[data_type][-1]
                                if timestamp_raw < prev_timestamp_raw:
                                    timestamp = timestamp_raw + (prev_timestamp // 1e9 + 1) * 1e9
                                else:
                                    timestamp = timestamp_raw + (prev_timestamp // 1e9) * 1e9

                    count, struct = self.packet_struct[data_type]
                    self.fp.seek(count * struct.size, 1)
                    self.packets[data_type].append((packet_pos, timestamp_type, timestamp_raw, timestamp))
                    bar(self.fp.tell() / end)
            bar(1)

            self.timestamp_end: list[float] = [self.packets[i][-1][3] if len(self.packets[i]) > 0 else 0 for i in range(7)]  # i: type i, -1: end of data, 3: timestamp
        return self

    def get_packet(self, index: int, data_type: int = 2):
        packet_pos, timestamp_type, timestamp_raw, timestamp = self.packets[data_type][index]

        self.fp.seek(packet_pos, 0)
        header = self.PacketHeader(*self.packet_header.unpack(self.fp.read(self.packet_header.size)), timestamp)
        count, struct = self.packet_struct[data_type]
        data = [struct.unpack(self.fp.read(struct.size)) for _ in range(count)]

        match data_type:
            case 2:
                x, y, z, reflectivity, tag = list(zip(*data))
                # noinspection PyArgumentList
                return self.packet_return[data_type](header, np.array((x, y, z)).T.astype(np.float64), np.array(reflectivity).astype(np.float64), np.array(tag).astype(np.uint8))

    @staticmethod
    def tag_filter(tags: np.ndarray, min_return: int = 1, max_return: int = 4, min_intensity_confidence: int = 1, min_spatial_confidence: int = 1) -> np.ndarray:
        returns = np.bitwise_and(np.right_shift(tags, 4), 3) + 1  # echo times [1, 4] (self-reflection without a target and echo within 1.5 meters are both number 1)
        intensity_confidence = np.mod(np.minimum(np.bitwise_and(np.right_shift(tags, 2), 3), 2).astype(np.int8) - 1, 3)  # confidence by intensity [0, 2] (The bigger, the more reliable)
        spatial_confidence = np.mod(np.bitwise_and(tags, 3).astype(np.int8) - 1, 4)  # confidence by spatial [0, 3] (The bigger, the more reliable)
        return (min_return <= returns) & (returns <= max_return) & (min_intensity_confidence <= intensity_confidence) & (min_spatial_confidence <= spatial_confidence)

    def size(self, data_type: int = 2):
        return len(self.packets[data_type])

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.fp.close()
