import matplotlib.pyplot as plt

from tracker.tracker import *
from tracker.parameters import *
import numpy as np
from collections import namedtuple

class IMUTracker(Tracker):
    def __init__(self):
        super().__init__()
        print('Init IMUTracker')
        self.kinetic_var |= {'Q', 'O', 'B', 'P', 'V', 'A', 'J'}
        self.sensor.append('IMU')

    def unscented_imu_measurement(self, m: np.ndarray, P: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        # get sigma points for unscented
        xs, wm, wc = get_unscented_sigma_points(m, P)
        xs = MeanState(self.kinetic_var, xs)
        xs.normalize_q()

        c_bn: np.ndarray = quaternion_to_matrix(xs['Q'])  # Transformation matrix: imu_body -> ned
        c_nb = np.transpose(c_bn, (0, 2, 1))

        # propagate sigma points through the imu measurement model
        # xs: imu state in ned
        # ys: imu state in body coordinate
        ys: np.ndarray = np.zeros((xs.value.shape[0], 6))   # acc 3 + gyr 3
        ys[:, 0:3] = np.einsum('ijk,ik->ij', c_nb, (xs['A'] - Params.g_n))
        ys[:, 3:6] = np.einsum('ijk,ik->ij', c_nb, xs['O'])

        return xs.value, ys, wm, wc

    def imu_update(self, data: namedtuple, current_time: float, step_end: bool = True) -> None:
        xs, ys, wm, wc = self.unscented_imu_measurement(self.state.m.value, self.state.P.value)
        mea: np.ndarray = np.array([data.acc_X, data.acc_Y, data.acc_Z,
                                    data.gyr_X, data.gyr_Y, data.gyr_Z])
        error: np.ndarray = scipy.linalg.block_diag(Params.acc_cov, Params.gyr_cov)

        u = np.sum(wm[:, None] * ys, axis=0)
        S = np.sum(wc[:, None, None] * (ys - u)[:, :, None] * (ys - u)[:, None, :], axis=0) + error
        C = np.sum(wc[:, None, None] * (xs - self.state.m.value)[:, :, None] * (ys - u)[:, None, :], axis=0)

        K = C @ np.linalg.inv(S)
        self.state.m.value = self.state.m.value + K @ (mea - u)
        self.state.m.normalize_q()
        self.state.P.value = self.state.P.value - K @ S @ K.T

        if step_end:
            self.save_step(current_time, 'IMU')