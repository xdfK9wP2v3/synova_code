import abc
import fractions
from my_msgs.msg import TimeStamp

class DeviceTime(abc.ABC):
    @property
    @abc.abstractmethod
    def sec(self) -> int:
        pass

    @property
    @abc.abstractmethod
    def nanosec(self) -> int:
        pass

    @classmethod
    def diff(cls, a, b) -> int:
        return a.nanosec_int - b.nanosec_int

    @property
    def sec_float(self):
        return self.sec + self.nanosec * 1e-9

    @property
    def nanosec_int(self):
        return int(self.sec * 1e9 + self.nanosec)

    def __str__(self):
        return f"{self.sec_float:.7f}"


class SensorDeviceTime(DeviceTime):
    def __init__(self, sec: int, microsec: int):
        self._sec = sec
        self._microsec = microsec

    @property
    def sec(self):
        return self._sec

    @property
    def nanosec(self):
        return self._microsec * 1e3


class SyncDeviceTime(DeviceTime):
    def __init__(self):
        self._pwr_sec          : int | None = None
        self._utc_sec          : int | None = None
        self._tick             : int | None = None
        self._ticks_per_sec    : int | None = None
        self._subtick          : int | None = None
        self._subtick_per_tick : int | None = None
        self._clk              : int | None = None
        self._clk_per_sec      : int | None = None

        self._sec : int | None = None
        self._nanosec: int | None = None

    @property
    def sec(self) -> int:
        return self._sec

    @property
    def nanosec(self) -> int:
        return self._nanosec

    def load_from_ts_msg(self, ts_msg: TimeStamp):
        self._pwr_sec          = int(ts_msg.pwr_sec)
        self._utc_sec          = int(ts_msg.utc_sec)
        self._tick             = int(ts_msg.tick)
        self._ticks_per_sec    = int(ts_msg.tick_per_sec)
        self._subtick          = int(ts_msg.subtick)
        self._subtick_per_tick = int(ts_msg.subtick_per_tick)
        self._clk              = int(ts_msg.clk)
        self._clk_per_sec      = int(ts_msg.clk_per_sec)

        # TODO: Check
        self._nanosec = int(1e9 / float(self._ticks_per_sec) * (self._tick + float(self._subtick) / float(self._subtick_per_tick)))
        self._sec = self._pwr_sec + int(self._nanosec // 1e9)
        self._nanosec = int(self._nanosec % 1e9)

    def convert_to_ts_msg(self) -> TimeStamp:
        ts_msg = TimeStamp()

        ts_msg.pwr_sec = self._pwr_sec
        ts_msg.utc_sec = self._utc_sec
        ts_msg.tick = self._tick
        ts_msg.tick_per_sec = self._ticks_per_sec
        ts_msg.subtick = self._subtick
        ts_msg.subtick_per_tick = self._subtick_per_tick
        ts_msg.clk = self._clk
        ts_msg.clk_per_sec = self._clk_per_sec

        return ts_msg

    @staticmethod
    def ts_msg_to_ts_nanosec_int(ts_msg: TimeStamp) -> int:
        pwr_sec = int(ts_msg.pwr_sec)
        tick = int(ts_msg.tick)
        ticks_per_sec = int(ts_msg.tick_per_sec)
        subtick = int(ts_msg.subtick)
        subtick_per_tick = int(ts_msg.subtick_per_tick)

        nanosec = int(1e9 / float(ticks_per_sec) * (tick + float(subtick) / float(subtick_per_tick)))
        sec = pwr_sec * 1e9

        return int(sec + nanosec)