from typing import Callable

import numpy as np

from .State import KineticState, MeanView, CovarianceView
from .Sampler import Sampler, UnscentedSampler


class UKF_Base:
    def __init__(self, kinetic_var: set[str]):
        self.states = KineticState(kinetic_var)
        self.sampler: Sampler = UnscentedSampler(len(self.states))

        self._predict_error = np.zeros((self.states.view.source_length,) * 2)

    def predict_step(self, dt: float):
        self.states.predict_step(self.sampler, dt)
        self.states.P[...] += self._predict_error * dt

    @property
    def predict_error(self):
        return CovarianceView(self._predict_error, self.states.view)

    def _update_by_error(self, get_expect: Callable[[np.ndarray], np.ndarray], observed_value: np.ndarray, observe_error: np.ndarray):
        self.states.update_by_error(self.sampler, get_expect, observed_value, observe_error)
