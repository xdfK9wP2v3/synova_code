import unittest
from scipy.integrate import solve_ivp

from ukf_filter.State import *
from ukf_filter.Rotation import *


class TestState(unittest.TestCase):
    def test_normalize(self):
        state = KineticState('QO')
        state._m[...] = -1
        state.m['Q'] = np.array([4, 3, 2, 1])
        KineticState.normalize_(state.m)

        self.assertAlmostEqual(np.linalg.norm(state._m[:4]), 1.0)
        self.assertTrue(np.all(state._m[4:] == -1))
        self.assertTrue(np.all(state._m[:4] > 0))

        state = KineticState('PVA')
        state._m[...] = -1
        KineticState.normalize_(state.m)
        self.assertTrue(np.all(state._m == -1))

    def test_ode_simply(self):
        state = KineticState('PVA')
        x = np.arange(9).astype(float)
        state._m[...] = x

        dm = state.kinetics_ode(0, state._m)
        self.assertTrue(np.all(state._m == x))
        self.assertTrue(np.all(dm[:6] == x[3:]))
        self.assertTrue(np.all(dm[6:] == 0))

    def test_ode_hard(self):
        state = KineticState('QOPVA')
        state.m['Q'] = np.array([1, 0, 0, 0])
        state.m['O'] = np.array([0, 0, 1])
        state.m['P'] = np.array([0, 0, 0])
        state.m['V'] = np.array([0, 0, 10])
        state.m['A'] = np.array([0, 0, -1])

        # noinspection PyTypeChecker
        sol: np.ndarray = solve_ivp(
            state.kinetics_ode,
            (0, 5),
            state._m.flatten(),
            method='RK23', rtol=1e-3, atol=1e-3
        ).y[:, -1]
        sol[:4] /= np.linalg.norm(sol[:4], axis=-1, keepdims=True)

        self.assertTrue(np.allclose(sol, np.array([np.cos(5 / 2), 0, 0, np.sin(5 / 2), 0, 0, 1, 0, 0, 37.5, 0, 0, 5, 0, 0, -1]), rtol=1e-2))


if __name__ == '__main__':
    unittest.main()
