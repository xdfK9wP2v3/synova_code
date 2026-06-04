import contextlib
import logging
import numpy as np
from collections import deque
from typing import Any, Callable
import threading
import csv
import contextlib

from .device_time import DeviceTime, SyncDeviceTime, SensorDeviceTime

logging.basicConfig(level=logging.DEBUG)


class TimeAlign:
    def __init__(self, data_fps: int, logger):
        self.a_pkgs: deque[tuple[DeviceTime, Any]] = deque()
        self.b_pkgs: deque[tuple[DeviceTime, Any]] = deque()

        self.a_tms: deque[tuple[float, DeviceTime]] = deque()
        self.b_tms: deque[tuple[float, DeviceTime]] = deque()

        # Tb == (1 + f_o) * Ta + t_o
        # Ta == Tb / (1 + f_o) - t_o / (1 + f_o)
        self.f_o: float | None = None  # in nanosecond
        self.t_o: float | None = None

        self.wall_tol: float = 0.5  # Note! unit: sec

        self.data_time_gap: float = 1e9 / data_fps  # TODO: determined by sensors
        self.device_tol: float = self.data_time_gap / 16  # TODO: To be determined
        self.obs_cov: float = 1e5 ** 2
        self.Q = (np.diag([1e1, 1e3]) / data_fps) ** 2  # Process noise of freq_offset and time_offset
        self.P = np.diag([1e3, 1e8]) ** 2

        self.ts_diff: float = 0.

        self.logger = logger
        self.update_idx: int = 0

        self._mutex = threading.Lock()

    def _add_to_queue(self, t_major, pkg_major,
                      major_pkg_queue: deque[tuple[Any, Any]],
                      minor_pkg_queue: deque[tuple[Any, Any]],
                      matcher: Callable[[Any, Any], int],
                      debug_msg: Callable[[Any, Any], str]
                      ) -> None | tuple[Any, Any, Any, Any]:
        assert not (minor_pkg_queue and major_pkg_queue), f"Unexpected queue @ {debug_msg}"
        # assert not major_pkg_queue, f"Expect Major Queue to be empty"

        while minor_pkg_queue:
            t_minor, pkg_minor = minor_pkg_queue[0]

            matched = matcher(t_major, t_minor)
            if matched == 0:
                minor_pkg_queue.popleft()
                return t_major, t_minor, pkg_major, pkg_minor
            else:
                self.logger.error(f"Mismatch @ {debug_msg(t_major, t_minor)}")
                if matched > 0:
                    minor_pkg_queue.popleft()
                else:
                    return None

        major_pkg_queue.append((t_major, pkg_major))
        return None

    def _match_device_a2b(self, t_a: DeviceTime, t_b: DeviceTime) -> int:
        """
            ((1 + fo) * ta + to) - tb
        ==  to + (ta - tb) + fo * ta

             0 : if matched
            -1 : if Ta expect a Tb < actual Tb (A is older)
            +1 : if Ta expect a Tb > actual Tb (B is older)
        """
        time_diff = self.t_o + DeviceTime.diff(t_a, t_b) + self.f_o * t_a.sec_float # (Tb expected by Ta) - (actual Tb)
        return 0 if abs(time_diff) < self.device_tol else (-1 if time_diff < 0 else +1)

    def _match_device_b2a(self, t_b: DeviceTime, t_a: DeviceTime) -> int:
        """
             0 : if matched
            -1 : if Tb expect a Ta < actual Ta (B is older)
            +1 : if Tb expect a Ta > actual Ta (A is older)
        """
        return -self._match_device_a2b(t_a, t_b)

    def _match_wall(self, t_a: float, t_b: float) -> int:
        """
             0 : if matched
            -1 : if Ta expect a Tb < actual Tb (A is older)
            +1 : if Ta expect a Tb > actual Tb (B is older)
        """
        return 0 if abs(t_a - t_b) < self.wall_tol else (-1 if t_a < t_b else +1)

    def add_a_pkg(self, a_t: DeviceTime, a_pkg: Any) -> tuple[Any, Any] | None:
        with self._mutex:
            if self.t_o is None:
                return None

            paired = self._add_to_queue(a_t, a_pkg, self.a_pkgs, self.b_pkgs, self._match_device_a2b, debug_msg=lambda ta, tb: f"PKG A=>B A:<{ta}>,  B:{tb} , expect B:{((1e9 + self.f_o) * ta.sec_float + self.t_o) * 1e-9:.7f}, to = {self.t_o:.7f}, fo = {self.f_o:.7f}")
            if paired is not None:
                t_a, t_b, pkg_a, pkg_b = paired
                self._update(t_a, t_b)
                return pkg_a, pkg_b
            return None

    def add_b_pkg(self, b_t: DeviceTime, b_pkg: Any):
        with self._mutex:
            if self.t_o is None:
                return None, None

            paired = self._add_to_queue(b_t, b_pkg, self.b_pkgs, self.a_pkgs, self._match_device_b2a, debug_msg=lambda tb, ta: f"PKG B=>A A: {ta} , <B:{tb}>, expect A:{1e9 / (1e9 + self.f_o) * (tb.sec_float - self.t_o * 1e-9):.7f}, to = {self.t_o:.7f}, fo = {self.f_o:.7f}")
            if paired is not None:
                t_b, t_a, pkg_b, pkg_a = paired
                self._update(t_a, t_b)
                return pkg_a, pkg_b
            return None, None

    def add_a_tms(self, a_wall: float, a_t: DeviceTime) -> None:
        with self._mutex:
            paired = self._add_to_queue(a_wall, a_t, self.a_tms, self.b_tms, self._match_wall, debug_msg=lambda ta, tb: f"TMS A=>B A:{ta}, B:{tb}")
            if paired is not None:
                *_, a_device, b_device = paired
                self._init_or_check(a_device, b_device)

    def add_b_tms(self, b_wall: float, b_t: DeviceTime) -> None:
        with self._mutex:
            paired = self._add_to_queue(b_wall, b_t, self.b_tms, self.a_tms, self._match_wall, debug_msg=lambda tb, ta: f"TMS B=>A A:{ta}, B:{tb}")
            if paired is not None:
                *_, b_device, a_device = paired
                self._init_or_check(a_device, b_device)

    def _init_or_check(self, a_device, b_device):
        device_diff = DeviceTime.diff(b_device, a_device)

        if self.t_o is None:
            self.t_o = device_diff
            self.f_o = 0.
            logging.info(f"Init Kalman Filter")
            logging.info(f"{self.f_o=}, {self.t_o=}, {device_diff=}")
        else:
            if self._match_device_a2b(t_a=a_device, t_b=b_device) != 0:
                logging.error(f"{self.f_o=}, {self.t_o=}, {device_diff=}")
                raise RuntimeError("Current Kalman filter is out of track!")
            else:
                pass

    def _update(self, t_a: DeviceTime, t_b: DeviceTime):
        """
        t_b = (1 + f_o) * t_a + t_o
        state vector:        x = [f_o, t_o] where f_o ~ 3.33e2, t_o ~ 1e9
        state transition:    x_{k+1} = F @ x_{k},    F = np.eye(2)
        observation:         z = t_b - t_a = f_o * t_a.sec_float + t_o = H @ x
                             H = [t_a, 1].T
                             y = z - H @ x = t_b - t_a - H @ x
                             S = H @ P @ H.T + R
                             K = P @ H.T @ inv(S)
        """
        # measurement z = t_sensor - t_sync = f_o * t_sync + t_o  ==>  H = [t_sync, 1]^T
        self.P += self.Q  # Process noise

        Pf, Pc, Pt = self.P[0, 0], self.P[0, 1], self.P[1, 1]
        ta_sec_float = t_a.sec_float
        a = 2 * Pc + Pf * ta_sec_float
        b = Pt + self.obs_cov + Pc * ta_sec_float
        c = Pt + self.obs_cov + a * ta_sec_float
        te = DeviceTime.diff(t_b, t_a)

        self.f_o = ((b * self.f_o) + (a - Pc) * (te - self.t_o)) / c
        self.t_o = self.t_o - (Pt + Pc * ta_sec_float) * (self.f_o * ta_sec_float - te + self.t_o) / c

        self.P[0, 0] = (- Pc ** 2 + Pf * (Pt + self.obs_cov)) / c
        self.P[0, 1] = (- a * Pt + Pc * (b + Pt)) / c
        self.P[1, 0] = self.P[0, 1]
        self.P[1, 1] = (c * Pt - (b - self.obs_cov) ** 2) / c

        self.ts_diff = DeviceTime.diff(t_b, t_a)
