import matplotlib.pyplot as plt
from collections import namedtuple

from .parameters import *
from .base_tracker import *


class RTKTracker(Tracker):
    def __init__(self):
        super().__init__()
        print('Init RTKTracker')
        self.kinetic_var |= {'P', 'V'}
        self.sensor.append('RTK')

    def unscented_rtk_measurement(self, m: np.ndarray, P: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        # get sigma points for unscented
        xs, wm, wc = get_unscented_sigma_points(m, P)
        xs = MeanState(self.kinetic_var, xs)
        xs.normalize_q()

        # propagate sigma points through the rtk measurement model
        c_bn: np.ndarray = quaternion_to_matrix(xs['Q'])  # Transformation matrix: imu_body -> ned

        # xs: imu state in ned
        # ys: rtk position in ned
        ys: np.ndarray = xs['P'] + c_bn @ Extrinsics.imu2rtk_b

        return xs.value, ys, wm, wc

    def rtk_update(self, data: namedtuple, current_time: float, step_end: bool = True) -> None:
        xs, ys, wm, wc = self.unscented_rtk_measurement(self.state.m.value, self.state.P.value)
        mea: np.ndarray = data.pos
        error: np.ndarray = data.cov

        u = np.sum(wm[:, None] * ys, axis=0)
        S = np.sum(wc[:, None, None] * (ys - u)[:, :, None] * (ys - u)[:, None, :], axis=0) + error
        C = np.sum(wc[:, None, None] * (xs - self.state.m.value)[:, :, None] * (ys - u)[:, None, :], axis=0)

        K = C @ np.linalg.inv(S)
        self.state.m.value = self.state.m.value + K @ (mea - u)
        self.state.m.normalize_q()
        self.state.P.value = self.state.P.value - K @ S @ K.T

        if step_end:
            self.save_step(current_time, 'RTK')