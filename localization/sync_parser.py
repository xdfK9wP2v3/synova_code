import os
from tracker.msg_data_parser import *
import matplotlib.pyplot as plt
from calibration.mag_calib_matlab import *
from tracker.parameters import *
from typing import Literal


class syncMsgType(Enum):
    SECON = 'SECON'
    CAPIN = 'CAPIN'
    TROUT = 'TROUT'
    PULSE = 'PULSE'
    UTCIN = 'UTCIN'


class SYNCData(MsgData):
    def __init__(self, filepath: str, msg_filter: list = None, sensor_name: str = 'SYNC'):
        super().__init__()
        self.filepath: str = filepath
        self.name = sensor_name
        self.power_ts_start: int | None = None
        self.utc_ts_start: int | None = None
        self.host_ts_start: int | None = None

        raw_data = self.file_parser(msg_filter=msg_filter)  # load data log file
        output = dict()
        for t in syncMsgType:
            filtered_data = [d for d in raw_data if d.type == t.value]
            if len(filtered_data) > 0:
                output[f'{t.value}'] = filtered_data
        self.data = output

    def line_parser(self, line: str, data_re: re.Pattern, msg_filter: list = None):
        nmea_f = data_re.search(line.strip())
        if nmea_f is None:
            return None

        msg = nmea_f.group(1)
        chksum = nmea_f.group(2)

        if msg_filter is not None:
            msg_type = msg.split(',')[0]
            if msg_type not in (msg_filter + ['HOST', 'SECON']):
                return None

        if MsgData.checksum_xor(msg) != chksum:
            print(f"checksum failed: {line}")
            return None

        message = msg.split(',')
        msg_type = message[0]

        # Load host timestamp data
        # $HOST,065519.489764,03,12,2024,1*39
        if msg_type == 'HOST':
            return self._parse_host_msg(message)

        common_data = self._parse_common_data(message)

        if self.power_ts_start is None and self.utc_ts_start is None:
            self.power_ts_start = common_data['power_ts']
            self.utc_ts_start = common_data['utc_ts']
            assert self.power_ts_start >= 0

        parse_func_map = {
            'SECON': self._parse_secon_msg,
            'CAPIN': self._parse_capin_msg,
            'TROUT': self._parse_trout_msg,
            'PULSE': self._parse_pulse_msg,
            'UTCIN': self._parse_utcin_msg,
        }

        if msg_type in parse_func_map:
            return parse_func_map[msg_type](message, common_data)
        else:
            return None

    # $HOST, 065519.489764, 03, 12, 2024, 1 * 39
    def _parse_host_msg(self, message: list[str]):
        # $HOST,065519.489764,03,12,2024,1*39
        if self.host_ts_start is not None:
            raise ValueError('More than one host time stamp message exist!')

        host_time = message[1]

        # Convert host to second, add 8 hours
        self.host_ts_start = (int(host_time[:2]) + 8) * 3600 + int(host_time[2:4]) * 60 + int(host_time[4:6]) + float(host_time.split('.')[-1]) * 1e-6
        return None

    def _parse_common_data(self, message: list[str]) -> dict:
        common_data = {
            'type': message[0],
            'msg_sn': int(message[1]),
            'system_state': message[2],
            # 'utc_year': int(message[3].split('/')[0]),
            # 'utc_mon': int(message[3].split('/')[1]),
            # 'utc_day': int(message[3].split('/')[2]),
            # 'utc_hour': int(message[4].split(':')[0]),
            # 'utc_min': int(message[4].split(':')[1]),
            # 'utc_sec': int(message[4].split(':')[2]),
            'utc_ts': np.int64(message[5]),
            'power_ts': int(message[6]),
            'tick': int(message[7]),
            'subtick': int(message[8].split('/')[0]),
            'tick_fraction': int(message[8].split('/')[1]),
            # 'clk_from_prev_event': np.int64(message[9]),
        }
        return common_data

    def _compute_ts(self, power_ts: int, tick: int, subtick: int, tick_fraction: int) -> float:
        # compute ns-level timestamp from tick info
        ts = (power_ts - self.power_ts_start) * 1e9  # align with power_ts
        ts += float(tick) * 1e9 / Params.ticks_per_sec
        ts += float(subtick) * 1e9 / (tick_fraction * Params.ticks_per_sec)
        return ts

    def _parse_secon_msg(self, message: list[str], common_data: dict):
        # $SECON,0012,E0001FFF,2024/11/21,09:33:22,1732181602,520,0000,00000/20000,240002382,519*4A
        # $SECON,...,prev_power_ts=message[10]
        common_data.update({
            'prev_power_ts': int(message[10]),
        })
        return namedtuple('SECON', common_data.keys())(**common_data)

    def _parse_capin_msg(self, message: list[str], common_data: dict):
        # $CAPIN,0009,E0001FFF,2024/11/21,09:33:21,1732181601,519,11993,07228/20000,239869610,2,F*17
        # $CAPIN,...,in_channel_idx=message[10],edge_type=message[11]
        ts = self._compute_ts(common_data['power_ts'], common_data['tick'], common_data['subtick'], common_data['tick_fraction'])
        common_data.update({
            'ts': ts,
            'in_channel_idx': int(message[10]),
            'edge_type': message[11],
        })
        return namedtuple('CAPIN', common_data.keys())(**common_data)


    def _parse_trout_msg(self, message: list[str], common_data: dict):
        # $TROUT,0010,E0001FFF,2024/11/21,09:33:21,1732181601,519,11994,00000/20000,239882382,4,R,0*0F
        # $TROUT,...,out_channel_idx=message[10],edge_type=message[11],rf_compensation=message[12]
        ts = self._compute_ts(common_data['power_ts'], common_data['tick'], common_data['subtick'], common_data['tick_fraction'])
        common_data.update({
            'ts': ts,
            'out_channel_idx': int(message[10]),
            'edge_type': message[11],
            'rf_compensation': message[12],
        })
        return namedtuple('TROUT', common_data.keys())(**common_data)

    def _parse_pulse_msg(self, message: list[str], common_data: dict):
        # $PULSE,0016,E0001FFF,2024/11/21,09:33:22,1732181602,520,0000,00006/20001,000000006,240002382,240002382+074937/100001*52
        # $PULSE,...,curr_clks_per_pulse
        pulse_data = message[11].split('+')
        integer_part = pulse_data[0]
        fraction_part = pulse_data[-1].split('/')
        curr_clks_per_pulse_integer = int(integer_part)
        curr_clks_per_pulse_numer = int(fraction_part[0])
        curr_clks_per_pulse_denom = int(fraction_part[-1])

        common_data.update({
            'curr_clks_per_pulse': int(message[10]),
            'curr_clks_per_pulse_integer': curr_clks_per_pulse_integer,
            'curr_clks_per_pulse_numer': curr_clks_per_pulse_numer,
            'curr_clks_per_pulse_denom': curr_clks_per_pulse_denom,
        })
        return namedtuple('PULSE', common_data.keys())(**common_data)

    def _parse_utcin_msg(self, message: list[str], common_data: dict):
        # $UTCIN,0044,E0001FFF,2024/11/21,09:33:22,1732181602,520,0304,15054/20000,006095119,2024/11/21,09:33:22.000*55
        # $UTCIN,...,in_utc_year, in_utc_mon, ...
        utc_date = message[10].split('/')
        utc_time = message[11].split(':')
        sec_ms = utc_time[2].split('.')
        in_utc_sec = int(sec_ms[0])
        in_utc_ms = int(sec_ms[-1])

        common_data.update({
            'in_utc_year': int(utc_date[0]),
            'in_utc_mon': int(utc_date[1]),
            'in_utc_day': int(utc_date[2]),
            'in_utc_hour': int(utc_time[0]),
            'in_utc_min': int(utc_time[1]),
            'in_utc_sec': in_utc_sec,
            'in_utc_ms': in_utc_ms,
        })
        return namedtuple('UTCIN', common_data.keys())(**common_data)

    def get_event_ts(self, event_type: Literal["CAPIN", "TROUT"]) -> dict:
        output = dict()
        if self.data is None or len(self.data[event_type]) == 0:
            raise KeyError(f'No {event_type} data found!')

        for data in self.data[event_type]:
            assert data.power_ts >= self.power_ts_start

            if event_type == 'CAPIN':
                output_key = f'in_{data.in_channel_idx:d}_{data.edge_type}'
            else:  # TROUT
                output_key = f'out_{data.out_channel_idx:d}_{data.edge_type}'

            if output_key not in output.keys():
                output[output_key] = [data.ts,]
            else:
                output[output_key].append(data.ts)

        return output


if __name__ == '__main__':
    data_dir = ''
    data_date = ''
    data_seq: int = 1

    sync = SYNCData(os.path.join(data_dir, data_date, f'{data_seq}_SYNC.log'))

    capin_ts = sync.get_event_ts('CAPIN')
    trout_ts = sync.get_event_ts('TROUT')

    with open(os.path.join(data_dir, data_date, f'{data_seq}_SYNC.log'), "r") as file:
        lines = file.readlines()

    types = [line.split(",")[0][1:] for line in lines if line.strip()]
    unique_types = set(types)

    pass