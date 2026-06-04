import abc
import scipy
import numpy as np


class Sampler(abc.ABC):
    def __init__(self):
        super(Sampler, self).__init__()
        pass

    @abc.abstractmethod
    def sample(self, m: np.ndarray, P: np.ndarray) -> np.ndarray:
        """
        getting a sample for the non-linear transfer.
        X ~ N(m, P), getting the distribution of Y = f(X), by ys = f(xs) where xs = sample(m, P).
        get the distribution result using transfer by ys.
        Args:
            m: mean of input
            P: covariance of input

        Returns:
            The sample points
        """
        pass

    @abc.abstractmethod
    def transfer_result(self, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        get the transfer result:
        f(X) ~ N(m, P)
        Args:
            ys: the transferred sample points

        Returns:
            (m, P): the mean and covariance of the transfer result
        """
        pass

    @abc.abstractmethod
    def transfer_all(self, m: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        get the transfer result:
        [X, f(X)] ~ N([m, u], [[P, C], [C.T, S]])
        Args:
            m: the mean of input
            xs: the sample points
            ys: the transferred sample points

        Returns:
            (u, S, C): the mean and covariance of the transfer result
        """
        pass


class UnscentedSampler(Sampler):
    def __init__(self, dim: int, alpha: float = 0.1, beta: float = 2., kappa: float | None = None):
        super(UnscentedSampler, self).__init__()

        self.dim = dim
        if kappa is None:
            kappa = max(0, 3 - dim)
        assert 0 < alpha <= 1
        assert kappa >= 0
        lambd = alpha ** 2 * (dim + kappa) - dim

        self.gamma = np.sqrt(dim + lambd)
        self.mean_weight = np.array([lambd / (dim + lambd), ] + [.5 / (dim + lambd)] * (2 * dim))
        self.cov_weight = np.array([lambd / (dim + lambd) + 1 - alpha ** 2 + beta, ] + [.5 / (dim + lambd)] * (2 * dim))

    def sample(self, m: np.ndarray, P: np.ndarray) -> np.ndarray:
        assert m.shape == (self.dim,), f"m shape {m.shape} does not match sampler shape {(self.dim,)}"
        assert P.shape == (self.dim, self.dim), f"m shape {m.shape} does not match sampler shape {(self.dim, self.dim)}"
        sqrt_P = np.linalg.cholesky(P)

        return np.concatenate([
            m[None, :],
            m[None, :] + self.gamma * sqrt_P.T,
            m[None, :] - self.gamma * sqrt_P.T
        ], axis=0)

    def transfer_result(self, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        u = np.sum(self.mean_weight[:, None] * ys, axis=0)
        S = np.sum(self.cov_weight[:, None, None] * (ys - u)[:, :, None] * (ys - u)[:, None, :], axis=0)
        return u, S

    def transfer_all(self, m: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        u, S = self.transfer_result(ys)
        C = np.sum(self.cov_weight[:, None, None] * (xs - m)[:, :, None] * (ys - u)[:, None, :], axis=0)
        return u, S, C
