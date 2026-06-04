import time

import numpy as np
import scipy
import warnings
import bisect
import matplotlib.pyplot as plt

from .rotation import *
from .parameters import *
from .tracker_state import State, MeanState, CovState
from .distribution_transfer import get_unscented_sigma_points


class Tracker:
    def __init__(self) -> None:
        print('Init Tracker')
        self.kinetic_var: set = set()
        self.sensor: list = []
        self.state = None
        self.prediction_error = None

    def reset_state(self,
                    init_q: np.ndarray | None = None,
                    init_p: np.ndarray | None = None,
                    init_q_cov: np.ndarray | None = None,
                    init_p_cov: np.ndarray | None = None,
                    current_time: float | None = None, ) -> None:  # TODO: why ts is float?
        self.state = State(self.kinetic_var)
        self.kinetic_var = self.state.kinetic_var  # Sort kinetic variables

        # Prediction error increase rate per second (used to characterize integral error, etc.)
        self.prediction_error = 5e-3 * np.eye(self.state.size)

        # reset q, p, time
        self.state.m['Q'], self.state.P['Q'] = init_q, init_q_cov
        self.state.m['P'], self.state.P['P'] = init_p, init_p_cov
        self.state.time = current_time    # unit: ns

    def save_step(self, current_time: float, sensor_type: str):
        self.state.ts.append(current_time)
        self.state.ms.append(self.state.m)
        self.state.Ps.append(self.state.P)
        self.state.time = current_time
        self.state.input.append(sensor_type)

    def kinetics_ode(self,
            kinetics_state: np.ndarray, _: float,
            angular_acceleration: np.ndarray, acceleration: np.ndarray,
            angular_jerk: np.ndarray, snap: np.ndarray,
            rotation_damping: np.ndarray, translation_damping: np.ndarray
    ) -> np.ndarray:
        # un-package & normalize
        kinetics_state = MeanState(self.kinetic_var, kinetics_state.reshape(-1, self.state.size))

        # angular_acceleration = kinetics_state[:, self.state.b_slice] if self.state.b_slice is not None else angular_acceleration
        # acceleration = kinetics_state[:, self.state.a_slice] if self.state.b_slice is not None else acceleration
        angular_acceleration = kinetics_state['B'] if 'B' in self.kinetic_var else angular_acceleration
        acceleration = kinetics_state['A'] if 'A' in self.kinetic_var else acceleration

        d_state = []
        if 'Q' in self.kinetic_var:
            quaternion = kinetics_state['Q']
            quaternion /= np.linalg.norm(quaternion, axis=-1, keepdims=True)

            angular_velocity = kinetics_state['O']

            # Q: get d(quaternion) / dt
            augmented_angular_velocity = np.concatenate([np.zeros([len(kinetics_state.value), 1]), angular_velocity], axis=-1)
            d_quaternion = .5 * quaternion_raw_multiply(augmented_angular_velocity, quaternion)
            d_state.append(d_quaternion)

            # O: get d(angular_velocity) / dt
            d_angular_velocity = - rotation_damping * angular_velocity + angular_acceleration
            d_state.append(d_angular_velocity)

        if 'B' in self.kinetic_var:   # When IMU is available
            # B: get d(angular_acceleration) / dt
            d_angular_acceleration = angular_jerk
            d_state.append(d_angular_acceleration)

        if 'P' in self.kinetic_var:
            velocity = kinetics_state['V']

            # P: get d(position) / dt
            d_position = velocity
            d_state.append(d_position)

            # V: get d(velocity) / dt
            d_velocity = - translation_damping * velocity + acceleration
            d_state.append(d_velocity)

        if 'A' in self.kinetic_var:   # When IMU is available
            jerk = kinetics_state['J']

            # A: get d(acceleration) / dt
            d_acceleration = jerk
            d_state.append(d_acceleration)

            # J: get d(jerk) / dt
            d_jerk = snap
            d_state.append(d_jerk)

        # package
        d_rotation_state = np.concatenate(d_state, axis=-1)

        return d_rotation_state.flatten()

    def unscented_kinetics(self, m: np.ndarray, P: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        aug_var_full = ['B', 'A', 'AJ', 'S']
        if 'IMU' in self.sensor:
            aug_var = ['AJ', 'S']
            m_aug = np.zeros(6)
            P_aug = (Params.angular_jerk_cov, Params.snap_cov)
        elif 'CAM' in self.sensor or 'LIDAR' in self.sensor:
            aug_var = ['B', 'A']
            m_aug = np.zeros(6)
            P_aug = (Params.angular_acceleration_cov, Params.acceleration_cov)
        elif 'RTK' in self.sensor:
            aug_var = ['A']
            m_aug = np.zeros(3)
            P_aug = (Params.acceleration_cov,)
        else:
            raise KeyError('Unexpected sensor!')

        # augmented m & P
        augmented_m = np.concatenate([m, m_aug])
        augmented_P = scipy.linalg.block_diag(P, *P_aug)

        # get sigma points for unscented
        xs, wm, wc = get_unscented_sigma_points(augmented_m, augmented_P)
        xs = MeanState(self.kinetic_var, xs)
        xs.normalize_q()

        # un-package & normalize
        kinetics_state, aug_state = np.split(xs.value, [self.state.size,], axis=-1)
        kinetics_state = MeanState.normalize_Q(self.kinetic_var, kinetics_state)

        kinetic_int_args = [aug_state[..., aug_var.index(var)*3:aug_var.index(var)*3+3] if var in aug_var else None for idx, var in enumerate(aug_var_full)]
        # kinetics update
        # noinspection PyTypeChecker
        sol: np.ndarray = scipy.integrate.odeint(
            self.kinetics_ode,
            kinetics_state.flatten(),
            [0, dt], (*kinetic_int_args, Params.rotation_dumping, Params.translation_dumping)
        )
        ys = sol[-1, :].reshape(-1, self.state.size)
        ys = MeanState.normalize_Q(self.kinetic_var, ys)

        return xs['VALID'], ys, wm, wc

    def predict_step(self, m: MeanState, P: CovState, dt: float, error: np.ndarray) -> tuple[MeanState, CovState]:
        _, ys, wm, wc = self.unscented_kinetics(m.value, P.value, dt)

        m = np.sum(wm[:, None] * ys, axis=0)
        m = MeanState.normalize_Q(self.kinetic_var, m)
        P = np.sum(wc[:, None, None] * (ys - m)[:, :, None] * (ys - m)[:, None, :], axis=0) + error

        return MeanState(self.kinetic_var, m), CovState(self.kinetic_var, P)

    def predict(self, current_time: float, step_end: bool = False) -> None:
        dt = (current_time - self.state.time) * 1e-9
        assert dt > 0
        self.state.m, self.state.P = self.predict_step(self.state.m, self.state.P, dt, self.prediction_error * abs(dt))

        if step_end:
            self.save_step(current_time, 'prediction')

    def smooth_step(self, prev_m: MeanState, prev_P: CovState, next_m: MeanState, next_P: CovState, dt: float, error: np.ndarray) -> tuple[MeanState, CovState]:
        assert dt > 0
        xs, ys, wm, wc = self.unscented_kinetics(prev_m.value, prev_P.value, dt)

        m = np.sum(wm[:, None] * ys, axis=0)
        m = MeanState.normalize_Q(self.kinetic_var, m)
        P = np.sum(wc[:, None, None] * (ys - m)[:, :, None] * (ys - m)[:, None, :], axis=0) + error
        D = np.sum(wc[:, None, None] * (xs - prev_m.value)[:, :, None] * (ys - m)[:, None, :], axis=0)

        G = D @ np.linalg.inv(P)

        m = prev_m.value + G @ (next_m.value - m)
        m = MeanState.normalize_Q(self.kinetic_var, m)
        P = prev_P.value + G @ (next_P.value - P) @ G.T

        return MeanState(self.kinetic_var, m), CovState(self.kinetic_var, P)

    def smooth(self, start_idx: int = 1):
        T = len(self.state.ts)
        if len(self.state.sms) == 0:
            self.state.sms = [None for _ in range(T - start_idx + 1)]
            self.state.sPs = [None for _ in range(T - start_idx + 1)]
        else:  # when backward tracking is finished and forward tracking smoothing is being performed
            self.state.sms.extend([None for _ in range(T - start_idx)])
            self.state.sPs.extend([None for _ in range(T - start_idx)])

        self.state.sms[-1] = self.state.ms[-1]
        self.state.sPs[-1] = self.state.Ps[-1]

        # for idx in alive_it(list(reversed(range(1, T))), title="smoothing", length=60, max_cols=150, force_tty=True):
        for idx in list(reversed(range(1, T))):
            dt = (self.state.ts[idx] - self.state.ts[idx - 1]) * 1e-9
            if idx >= start_idx:
                self.state.sms[idx - 1], self.state.sPs[idx - 1] = self.smooth_step(self.state.ms[idx - 1], self.state.Ps[idx - 1],
                                                                                    self.state.sms[idx], self.state.sPs[idx], dt,
                                                                                    self.prediction_error * abs(dt))
            else:
                self.state.sms[idx - 1], self.state.sPs[idx - 1] = self.smooth_step(self.state.sms[idx - 1], self.state.sPs[idx - 1],
                                                                        self.state.sms[idx], self.state.sPs[idx], dt,
                                                                        self.prediction_error * abs(dt))

    @staticmethod
    def quaternion_to_str(quaternion: np.ndarray, decimal: int = 1) -> str:
        quaternion = standardize_quaternion(quaternion)
        q_norm = np.linalg.norm(quaternion[1:])
        angle = 2 * np.rad2deg(np.arctan2(q_norm, quaternion[0]))
        rotation_axis = quaternion[1:] / q_norm

        return f"[{angle:{3 + 1 + decimal}.{decimal}f}°, <{rotation_axis[0]:+{2 + 1 + decimal}.{decimal}f}, {rotation_axis[1]:+{2 + 1 + decimal}.{decimal}f}, {rotation_axis[2]:+{2 + 1 + decimal}.{decimal}f}>]"

    @staticmethod
    def vector_to_str(vec: np.ndarray, width: int = 9, decimal: int = 1, thousand_sign: bool = True) -> str:
        return '[' + ', '.join(
            f"{v:{width + 1 + decimal}{',.' if thousand_sign else '.'}{decimal}f}" for v in vec) + ']'

    def save(self, filename: str):
        ms = [m.value for m in self.state.ms]
        Ps = [P.value for P in self.state.Ps]
        sms = [sm.value for sm in self.state.sms]
        sPs = [sP.value for sP in self.state.sPs]
        np.savez(filename, kinetic_var=list(self.kinetic_var), ts=self.state.ts, ms=ms, Ps=Ps, sms=sms, sPs=sPs)

    def load(self, filename: str):
        data = np.load(filename, allow_pickle=True)
        self.kinetic_var = set([str(var) for var in data['kinetic_var']])
        self.reset_state()   # Reset self.state and sort self.kinetic_var

        start = time.time()
        self.state.ts = list(data['ts'])
        print(time.time() - start)
        # self.state.ms = [MeanState(self.kinetic_var, m) for m in list(data['ms'])]
        # print(time.time() - start)
        # self.state.Ps = [CovState(self.kinetic_var, P) for P in list(data['Ps'])]
        # print(time.time() - start)
        self.state.sms = [MeanState(self.kinetic_var, sm) for sm in list(data['sms'])]
        print(time.time() - start)
        self.state.sPs = [CovState(self.kinetic_var, sP) for sP in list(data['sPs'])]
        print(time.time() - start)

    def load_ts(self, filename: str):
        data = np.load(filename, allow_pickle=True)

        self.kinetic_var = set([str(var) for var in data['kinetic_var']])
        self.reset_state()  # Reset self.state and sort self.kinetic_var
        self.state.ts = list(data['ts'])
        self.state.ms = [None for i in range(len(self.state.ts))]
        self.state.Ps = [None for i in range(len(self.state.ts))]
        self.state.sms = [None for i in range(len(self.state.ts))]
        self.state.sPs = [None for i in range(len(self.state.ts))]

    def load_state_from_ts(self, filename: str, timestamp: float):
        data = np.load(filename, allow_pickle=True)

        if self.state.ts[0] <= timestamp <= self.state.ts[-1]:
            idx = bisect.bisect(self.state.ts, timestamp) - 1
        else:
            raise ValueError(f"Time {timestamp} is outside the range of [{self.state.ts[0]}, {self.state.ts[-1]}]")

        for i in [idx, idx + 1]:
            self.state.ms[i] = MeanState(self.kinetic_var, data['ms'][i, :])
            self.state.Ps[i] = MeanState(self.kinetic_var, data['Ps'][i, :])
            self.state.sms[i] = MeanState(self.kinetic_var, data['sms'][i, :])
            self.state.sPs[i] = MeanState(self.kinetic_var, data['sPs'][i, :])

    def __call__(self, time: float, load_from_idx: bool = False) -> tuple[MeanState, CovState]:
        if self.state.ts[0] <= time <= self.state.ts[-1]:
            idx = bisect.bisect(self.state.ts, time) - 1

            if self.state.ts[idx] == time:
                return self.state.sms[idx], self.state.sPs[idx]
            else:
                dt = (time - self.state.ts[idx]) * 1e-9
                m, P = self.predict_step(self.state.sms[idx], self.state.sPs[idx], dt, self.prediction_error * dt)
                dt = (self.state.ts[idx + 1] - time) * 1e-9
                m, P = self.smooth_step(m, P, self.state.sms[idx + 1], self.state.sPs[idx + 1], dt, self.prediction_error * dt)
                return m, P
        else:
            warnings.warn(f"Time {time} is outside the range of [{self.state.ts[0]}, {self.state.ts[-1]}], the results are predicted using kinetic integration. ")
            idx = 0 if time < self.state.ts[0] else -1
            dt = (time - self.state.ts[idx]) * 1e-9
            return self.predict_step(self.state.sms[idx], self.state.sPs[idx], dt, self.prediction_error * dt)

    def visualize_states(self, confidence_level: float = 0.05, smoothed: bool = True, pkgs: slice = slice(None)):
        name_dict = {'Q': 'Quaternion', 'O': 'Angular Velocity', 'B': 'Angular Acceleration',
                     'P': 'Position', 'V': 'Velocity', 'A': 'Acceleration', 'J': 'Jerk'}

        # Visualize tracker states
        ts = np.array(self.state.ts[pkgs]) * 1e-9
        ms = (self.state.sms if smoothed else self.state.ms)[pkgs]
        Ps = (self.state.sPs if smoothed else self.state.Ps)[pkgs]

        for var in self.kinetic_var:
            name = name_dict[var]
            plt.figure(figsize=(24, 4))

            ms_var = np.array([m[var] for m in ms])
            Ps_var = np.array([P[var] for P in Ps])
            std = np.sqrt(np.diagonal(Ps_var, 0, -2, -1))

            plt.plot(ts, ms_var, alpha=1.0)
            k = scipy.stats.norm.ppf(1 - confidence_level / 2)
            for d, u in zip((ms_var - k * std).T, (ms_var + k * std).T):
                plt.fill_between(ts, d, u, alpha=0.5)
            plt.title(name)
            plt.show()

        Ps_pos = np.array([P['P'] for P in Ps])
        std_pos = np.sqrt(np.diagonal(Ps_pos, 0, -2, -1))
        plt.figure(figsize=(24, 4))
        plt.plot(ts, std_pos, alpha=0.5)
        plt.title('Position std')
        plt.show()

    def visualize_q(self, ref_q: np.ndarray | None = None, ref_ts: np.ndarray | None = None, smoothed: bool = True, pkgs: slice = slice(None)) -> None:
        ts = np.array(self.state.ts[pkgs]) * 1e-9
        qs = np.array([m['Q'] for m in self.state.sms[pkgs]]) if smoothed else np.array([m['Q'] for m in self.state.ms[pkgs]])
        assert np.sum(abs(np.linalg.norm(qs, axis=-1) - 1), axis=None) < 1e-5

        plot_ref = False if ref_ts is None or ref_q is None else True
        if plot_ref:
            ref_ts *= 1e-9
            assert ref_ts[0] < ts[-1] and ref_ts[-1] > ts[0]  # Assure ts and ref_ts have overlap
            ref_start = np.min(np.where(ref_ts > ts[0]))
            ref_end = np.max(np.where(ref_ts < ts[-1]))
            ref_slice = slice(ref_start, ref_end + 1)
            ref_q, ref_ts = ref_q[ref_slice], ref_ts[ref_slice]  # Slice the overlapped part of ref_ts within ts

            ref_axes_norm = np.linalg.norm(ref_q[:, 1:], axis=-1, keepdims=True)
            ref_angle = 2 * np.rad2deg(np.arctan2(ref_axes_norm, ref_q[:, :1])) - 360
            ref_axes = ref_q[:, 1:] / ref_axes_norm

        # Compare quaternion
        plt.figure()
        plt.plot(ts, qs[:, 0], 'C0*', label='track-Q1')
        if plot_ref:
            plt.plot(ref_ts, ref_q[:, 0], 'C0-', alpha=0.5, label='ref-Q1')
        plt.legend(loc='upper left')
        plt.twinx()
        for idx, c in zip(range(3), ['C1', 'C2', 'C3']):
            plt.plot(ts, qs[:, idx + 1], f'{c}--', label=f'track-Q{idx + 2}')
            if plot_ref:
                plt.plot(ref_ts, ref_q[:, idx + 1], f'{c}-', alpha=0.5)
        plt.title('Quaternion comparison' if plot_ref else 'Quaternion')
        plt.legend(loc='upper right')
        plt.show()

        # Compare rotation angle and rotation axes
        q_axes_norm = np.linalg.norm(qs[:, 1:4], axis=-1, keepdims=True)
        q_angle = 2 * np.rad2deg(np.arctan2(q_axes_norm, qs[:, :1])) - 360
        q_axes = qs[:, 1:4] / q_axes_norm

        plt.figure()
        plt.title('Rotation angle & axes comparison' if plot_ref else 'Rotation angle & axes')
        if plot_ref:
            plt.plot(ref_ts, ref_angle, 'k--', label='ref_theta')
        plt.plot(ts, q_angle, 'C3', label='imu_theta', alpha=0.6)
        plt.legend(loc='upper left')
        plt.twinx()
        for idx, c, ax in zip(range(3), ['C0', 'C1', 'C2'], ['track-x', 'track-y', 'track-z']):
            if plot_ref:
                plt.plot(ref_ts, ref_axes[:, idx], f'{c}--')
            plt.plot(ts, q_axes[:, idx], f'{c}-', alpha=0.5, label=ax)
        plt.ylim(-.5, 1.5)
        plt.legend(loc='upper right')
        plt.show()

        # Compare the multiplication of q
        if plot_ref:
            interp_q = []
            for ts in list(ref_ts):
                m, _ = self(ts * 1e9)
                interp_q.append(m['Q'])
            q_prod = quaternion_raw_multiply(quaternion_invert(np.array(interp_q)), ref_q)
            prod_axes_norm = np.linalg.norm(q_prod[..., 1:], axis=-1, keepdims=True)
            prod_angle = 2 * np.rad2deg(np.arctan2(prod_axes_norm, q_prod[..., :1]))
            prod_axes = q_prod[..., 1:] / prod_axes_norm
            plt.title('Q multiplication')
            plt.plot(ref_ts, prod_angle, 'C3', label='track-theta')
            plt.legend(loc='upper left')
            plt.twinx()
            for idx, c, ax in zip(range(3), ['C0', 'C1', 'C2'], ['track-x', 'track-y', 'track-z']):
                plt.plot(ref_ts, prod_axes[:, idx], f'{c}--', alpha=0.5, label=ax)
            plt.legend(loc='upper right')
            plt.show()

    def visualize_p(self, ref_p: np.ndarray | None = None, ref_ts: np.ndarray | None = None, smoothed: bool = True, pkgs: slice = slice(None)) -> None:
        ts = np.array(self.state.ts[pkgs]) * 1e-9
        ps = np.array([m['P'] for m in self.state.sms[pkgs]]) if smoothed else np.array([m['P'] for m in self.state.ms[pkgs]])

        plot_ref = False if ref_ts is None or ref_p is None else True
        if plot_ref:
            ref_ts *= 1e-9
            assert ref_ts[0] < ts[-1] and ref_ts[-1] > ts[0]  # Assure ts and ref_ts have overlap
            ref_start = np.min(np.where(ref_ts > ts[0]))
            ref_end = np.max(np.where(ref_ts < ts[-1]))
            ref_slice = slice(ref_start, ref_end + 1)
            ref_p, ref_ts = ref_p[ref_slice], ref_ts[ref_slice]  # Slice the overlapped part of ref_ts within ts

        for idx, c, ax in zip(range(3), ['C0', 'C1', 'C2'], ['track-x', 'track-y', 'track-z']):
            plt.plot(ts, ps[:, idx], f'{c}-', label=ax, alpha=0.5)
            if plot_ref:
                plt.plot(ref_ts, ref_p[:, idx], f'{c}--')
        plt.title('Position comparison' if plot_ref else 'Position after tracker')
        plt.legend()
        plt.show()


if __name__ == '__main__':
    class TestTracker(Tracker):
        def __init__(self):
            super().__init__()
            self.kinetic_var |= {'Q', 'O', 'P', 'V'}

    tracker = TestTracker()
    tracker.reset_state(init_p=np.array([1, 2, 3]), init_p_cov=np.eye(3) * 5, init_q=np.ones(4) * 2, init_q_cov=np.eye(4) * 2, current_time=50)
    print(tracker.state.P[('QP', 'P')])
    pass
    tracker.predict(current_time=1.)
    tracker.camera_update()
    tracker.imu_update()
    tracker.rtk_update()
    tracker.lidar_update()
