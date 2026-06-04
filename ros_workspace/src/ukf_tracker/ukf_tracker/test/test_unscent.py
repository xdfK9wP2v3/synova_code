import unittest
from ukf_filter.Sampler import *


class TestUnscented(unittest.TestCase):
    def setUp(self):
        self.sampler = UnscentedSampler(6)

    def test_identity(self):
        for _ in range(8):
            m = np.random.rand(6)
            P = np.random.rand(10, 6)
            P = P.T @ P
            xs = self.sampler.sample(m, P)
            ys = xs
            u, S, C = self.sampler.transfer_all(m, xs, ys)

            self.assertTrue(np.allclose(m, u))
            self.assertTrue(np.allclose(S, P))
            self.assertTrue(np.allclose(C, P))

    def test_linear(self):
        for _ in range(8):
            m = np.random.rand(6)
            P = np.random.rand(10, 6)
            P = P.T @ P
            xs = self.sampler.sample(m, P)

            A = np.random.rand(6, 6)
            b = np.random.rand(6)
            ys = xs @ A + b

            u, S, C = self.sampler.transfer_all(m, xs, ys)

            self.assertTrue(np.allclose(u, m @ A + b))
            self.assertTrue(np.allclose(S, A.T @ P @ A))
            self.assertTrue(np.allclose(C, P @ A))


if __name__ == '__main__':
    unittest.main()
