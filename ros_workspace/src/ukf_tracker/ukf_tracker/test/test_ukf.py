import unittest
import numpy as np
from scipy.integrate import solve_ivp

from ukf_filter.UKF_Base import UKF_Base

import matplotlib
import matplotlib.pyplot as plt


class TestUKF_Base(unittest.TestCase):
    def test_translation(self):
        ukf = UKF_Base({'P', 'V'})
        ukf.states.m['P'] = np.array([0., 0., 0.])
        ukf.states.m['V'] = np.array([0., 1., 0.])
        ukf.states.P['P'] = np.eye(3) * 1
        ukf.states.P['V'] = np.eye(3) * 0.1
        ukf.predict_error['P'] = np.eye(3) * 1e-4
        ukf.predict_error['V'] = np.eye(3) * 0.01

        ground_truth_acc = lambda t: 0.5 * np.array([np.cos(0.1 * t), np.sin(0.15 * t), 2 * np.sin(0.2 * t + 2)])
        sol = solve_ivp(
            lambda t, x: np.concatenate([x[3:], ground_truth_acc(t) - 1e-1 * x[3:]]),
            (0, 100),
            np.array([3, -2, 0, 0, 0, 0]),
            rtol=1e-7, atol=1e-12
        )

        pt = 0
        obs_func = lambda x: x[..., :3]
        res = []
        obs = []
        for t, y in zip(sol.t, sol.y.T):
            dt = t - pt
            ukf.predict_step(dt)
            z = obs_func(y) + np.random.randn(3) * 1
            ukf._update_by_error(obs_func, z, np.eye(3) * 1.0)
            res.append(ukf.states.m[...].copy())
            obs.append(z)
            pt = t
        res = np.stack(res)
        obs = np.stack(obs)

        matplotlib.use("Agg")
        plt.figure(figsize=(24, 16))
        plt.plot(sol.t, sol.y[0])
        plt.plot(sol.t, res[:, 0])
        plt.twinx()
        plt.plot(sol.t, obs[:, 0], 'g+--')
        plt.savefig("output_x.png")

        plt.figure(figsize=(24, 16))
        plt.plot(sol.t, sol.y[1])
        plt.plot(sol.t, res[:, 1])
        plt.twinx()
        plt.plot(sol.t, obs[:, 1], 'g+--')
        plt.savefig("output_y.png")

        pass


if __name__ == '__main__':
    unittest.main()
