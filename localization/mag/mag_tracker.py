from tracker.tracker import *
from tracker.parameters import *
import numpy as np
from collections import namedtuple


class MAGTracker(Tracker):
    def __init__(self):
        super().__init__()
        print('Init MAGTracker')
        self.kinetic_var |= {'Q', 'O'}
        self.sensor.append('MAG')

    def unscented_mag_measurement(self, m: np.ndarray, P: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        # get sigma points for unscented
        xs, wm, wc = get_unscented_sigma_points(m, P)
        xs = MeanState(self.kinetic_var, xs)
        xs.normalize_q()

        c_bn: np.ndarray = quaternion_to_matrix(xs['Q'])  # Transformation matrix: imu_body -> ned
        c_nb = np.transpose(c_bn, (0, 2, 1))

        # propagate sigma points through the mag measurement model
        # xs: imu_mag state in ned
        # ys: imu_mag state in body coordinate
        mag_north_vec = np.array([np.cos(np.deg2rad(Params.mag_var)), np.sin(np.deg2rad(Params.mag_var)), 0.])
        ys = np.einsum('ijk,k->ij', c_nb, mag_north_vec)

        return xs.value, ys, wm, wc

    def mag_update(self, data: namedtuple, current_time: float, step_end: bool = True) -> None:
        xs, ys, wm, wc = self.unscented_mag_measurement(self.state.m.value, self.state.P.value)
        mea: np.ndarray = np.array([data.mag_X, data.mag_Y, data.mag_Z])

        c_bn: np.ndarray = quaternion_to_matrix(self.state.m['Q'])
        c_nb = c_bn.T
        ed_b = c_nb @ np.array([0., 0., 1.])
        mea = mea - np.inner(ed_b, mea) * ed_b
        mea /= np.linalg.norm(mea)  # mag_b on the

        u = np.sum(wm[:, None] * ys, axis=0)
        S = np.sum(wc[:, None, None] * (ys - u)[:, :, None] * (ys - u)[:, None, :], axis=0) + Params.mag_cov
        C = np.sum(wc[:, None, None] * (xs - self.state.m.value)[:, :, None] * (ys - u)[:, None, :], axis=0)

        K = C @ np.linalg.inv(S)
        self.state.m.value = self.state.m.value + K @ (mea - u)
        self.state.m.normalize_q()
        self.state.P.value = self.state.P.value - K @ S @ K.T

        if step_end:
            self.save_step(current_time, 'MAG')