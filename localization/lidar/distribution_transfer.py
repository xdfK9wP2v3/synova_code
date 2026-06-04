import scipy
import numpy as np


def get_unscented_sigma_points(mean: np.ndarray, covariance: np.ndarray, alpha: float = 0.1, beta: float = 2., kappa: float | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = len(mean)
    assert mean.shape == (n,) and covariance.shape == (n, n)

    if kappa is None:
        kappa = max(0, 3 - n)
    assert 0 < alpha <= 1
    assert kappa >= 0

    lambd = alpha ** 2 * (n + kappa) - n
    try:
        sqrt_P = np.linalg.cholesky(covariance)
    except Exception:
        pass
    xs = np.concatenate(
        [
            mean[None, :],
            mean[None, :] + np.sqrt(n + lambd) * sqrt_P.T,
            mean[None, :] - np.sqrt(n + lambd) * sqrt_P.T
        ], axis=0
    )
    wm = np.array([lambd / (n + lambd), ] + [.5 / (n + lambd)] * (2 * n))
    wc = np.array([lambd / (n + lambd) + 1 - alpha ** 2 + beta, ] + [.5 / (n + lambd)] * (2 * n))

    # ys = f(xs)
    # m = np.sum(wm * ys, axis=0)
    # P = np.sum(wc * (ys - m)[:, :, None] * (ys - m)[:, None, :], axis=0)

    return xs, wm, wc


def get_monte_carlo_importance_sampling_points(mean: np.ndarray, covariance: np.ndarray, sample_num: int = 16 * 1024, scales: list[tuple[float, float]] | None = None) -> tuple[np.ndarray, np.ndarray]:
    if scales is None:
        scales = [(.25, .25), (.5, .25), (1., .25), (1.5, .25)]
    assert sum(ratio for _, ratio in scales) == 1

    n = len(mean)
    assert mean.shape == (n,) and covariance.shape == (n, n)

    xs = np.concatenate([scipy.stats.qmc.MultivariateNormalQMC(mean=mean, cov=scale ** 2 * covariance).random(int(ratio * sample_num)) for scale, ratio in scales], axis=0)
    ws = scipy.stats.multivariate_normal.pdf(xs, mean=mean, cov=covariance) / (sum(ratio * scipy.stats.multivariate_normal.pdf(xs, mean=mean, cov=scale ** 2 * covariance) for scale, ratio in scales))

    # noinspection PyTypeChecker
    return xs, ws
