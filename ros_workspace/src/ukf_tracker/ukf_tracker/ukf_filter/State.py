import functools
from typing import Callable

import numpy as np
from collections import namedtuple
from scipy.integrate import solve_ivp

from .Sampler import Sampler
from .Rotation import quaternion_raw_multiply


class KineticView:
    KineticConfig = namedtuple('SubStateConfig', ['len', 'doc'])

    @classmethod
    @property
    @functools.lru_cache()
    def kinetic_config(cls) -> dict[str, KineticConfig]:
        return {
            'Q': cls.KineticConfig(len=4, doc='posture in quaternions'),
            'O': cls.KineticConfig(len=3, doc='angular velocity in rad/s'),
            'B': cls.KineticConfig(len=3, doc='angular acceleration in rad/s^2'),
            'P': cls.KineticConfig(len=3, doc='position in m under NED frame'),
            'V': cls.KineticConfig(len=3, doc='velocity in m/s under NED frame'),
            'A': cls.KineticConfig(len=3, doc='acceleration in m/s^2 under NED frame'),
            'J': cls.KineticConfig(len=3, doc='jerk in m/s^3 under NED frame'),
        }

    @classmethod
    def format_seq(cls, seq) -> str:
        return ''.join(k for k in cls.kinetic_config.keys() if k in set(seq))

    def _check_kinetic_seq(self, kinetic_seq: str):
        assert all(kinetic in self.kinetic_config for kinetic in kinetic_seq), f"Unknown kinetic in {kinetic_seq}"
        assert len(set(kinetic_seq)) == len(kinetic_seq), f"Duplicate kinetics in {kinetic_seq}"

    @staticmethod
    def _check_new_kinetic_seq(new_kinetic_seq: str, old_kinetic_seq: str):
        assert all(kinetic in old_kinetic_seq for kinetic in new_kinetic_seq), f"{new_kinetic_seq} has kinetic not in {old_kinetic_seq}"

    def __init__(self, source_kinetic_seq: str, view_kinetic_seq: str | None = None):
        if view_kinetic_seq is None:
            view_kinetic_seq = source_kinetic_seq

        self._check_kinetic_seq(source_kinetic_seq)
        self._check_kinetic_seq(view_kinetic_seq)
        self._check_new_kinetic_seq(view_kinetic_seq, source_kinetic_seq)

        self._source_kinetic_seq = source_kinetic_seq
        self._view_kinetic_seq = view_kinetic_seq

    @property
    @functools.lru_cache()
    def source_length(self) -> int:
        return sum(self.kinetic_config[kinetic].len for kinetic in self._source_kinetic_seq)

    @property
    @functools.lru_cache()
    def view_length(self) -> int:
        return sum(self.kinetic_config[kinetic].len for kinetic in self._view_kinetic_seq)

    def __len__(self) -> int:
        return self.view_length

    def _get_sub_seq(self, kinetic_idx: int, sub_seq: str) -> tuple[int, int]:
        sub_seq_length = sum(self.kinetic_config[kinetic].len for kinetic in sub_seq)
        source_sub_begin = sum(self.kinetic_config[kinetic].len for kinetic in self._source_kinetic_seq[:kinetic_idx])
        return source_sub_begin, sub_seq_length

    @property
    @functools.lru_cache()
    def slice_mapping(self) -> list[tuple[slice, slice]]:
        """
        Returns: the list of slice mapping from the SOURCE to the VIEW
        """
        mapping = []
        view_seq_len = 0
        kinetic_seq = self._view_kinetic_seq
        while len(kinetic_seq) > 0:
            for sub_kinetic_cnt in reversed(range(len(kinetic_seq))):
                if (kinetic_idx := self._source_kinetic_seq.find(sub_seq := kinetic_seq[:(sub_kinetic_cnt + 1)])) >= 0:
                    source_sub_begin, sub_seq_length = self._get_sub_seq(kinetic_idx, sub_seq)
                    source_sub_slice = slice(source_sub_begin, source_sub_begin + sub_seq_length)

                    current_sub_slice = slice(view_seq_len, view_seq_len + sub_seq_length)
                    view_seq_len += sub_seq_length

                    mapping.append((source_sub_slice, current_sub_slice))
                    kinetic_seq = kinetic_seq[(sub_kinetic_cnt + 1):]
                    break
        return mapping

    @property
    @functools.lru_cache()
    def inverse_slice_mapping(self) -> list[tuple[slice, slice]]:
        """
        Returns: the list of slice mapping from the VIEW to the SOURCE
        """
        return [(c, s) for s, c in self.slice_mapping]

    @functools.lru_cache()
    def __getitem__(self, new_kinetic_seq: str) -> "KineticView":
        self._check_kinetic_seq(new_kinetic_seq)
        self._check_new_kinetic_seq(new_kinetic_seq, self._view_kinetic_seq)
        return KineticView(self._source_kinetic_seq, new_kinetic_seq)

    @functools.lru_cache()
    def __call__(self, sub_seq: str) -> slice:
        self._check_kinetic_seq(sub_seq)
        self._check_new_kinetic_seq(sub_seq, self._view_kinetic_seq)

        kinetic_idx = self._source_kinetic_seq.find(sub_seq)
        assert kinetic_idx >= 0, f"Did not find kinetic in {sub_seq} in {self.view_length}"

        source_sub_begin, sub_seq_length = self._get_sub_seq(kinetic_idx, sub_seq)
        return slice(source_sub_begin, source_sub_begin + sub_seq_length)

    def __contains__(self, item) -> bool:
        return item in self._view_kinetic_seq

    def __iter__(self):
        for k in self._view_kinetic_seq:
            yield k

    def __repr__(self) -> str:
        return f"View['{self._view_kinetic_seq}' of '{self._source_kinetic_seq}']"


class ArrayView:
    def __init__(self, arr: np.array, view: KineticView, view_dims: int) -> None:
        assert arr.shape[-view_dims:] == (view.source_length,) * view_dims, f"View shape mismatch: {arr.shape}"
        self._arr = arr
        self._view = view
        self._view_dims = view_dims

    def __call__(self, new_kinetic_seq: slice | str | type(Ellipsis) | None = None) -> "ArrayView":
        if new_kinetic_seq is None or new_kinetic_seq is Ellipsis:
            return self
        if isinstance(new_kinetic_seq, slice):
            assert new_kinetic_seq == slice(None, None, None)
            return self
        if isinstance(new_kinetic_seq, str):
            return ArrayView(self._arr, self._view[new_kinetic_seq], self._view_dims)
        raise TypeError(f"Unexpected type {type(new_kinetic_seq)}")

    def __getitem__(self, idx: tuple[slice | str | type(Ellipsis), ...] | slice | str | type(Ellipsis)) -> np.ndarray:
        if isinstance(idx, tuple):
            if idx[-1] is Ellipsis:
                return self.value[*idx[:-1], ..., :]
            else:
                return self(idx[-1]).value[*idx[:-1], :]
        else:
            return self(idx).value

    def __setitem__(self, new_kinetic_seq: str, value: np.ndarray):
        self(new_kinetic_seq).value = value

    def __contains__(self, item) -> bool:
        return item in self._view

    def __len__(self) -> int:
        return len(self._view)

    def get_ref_(self, sub_seq) -> np.ndarray:
        return self._arr[..., self._view(sub_seq)]

    @staticmethod
    def _mapping_setter_(dst: np.ndarray, src: np.ndarray, dim: int, mapping: list[tuple[slice, slice]]):
        if dim == 0:
            dst[...] = src[...]
        else:
            tail = (slice(None, None, None),) * (dim - 1)
            for d, s in mapping:
                ArrayView._mapping_setter_(dst[..., d, *tail], src[..., s, *tail], dim - 1, mapping)

    @property
    def value(self) -> np.ndarray:
        res = np.empty_like(self._arr, shape=self._arr.shape[:-self._view_dims] + (self._view.view_length,) * self._view_dims)
        ArrayView._mapping_setter_(res, self._arr, self._view_dims, self._view.inverse_slice_mapping)
        return res

    @value.setter
    def value(self, value: np.ndarray):
        assert value.shape[-self._view_dims:] == (self._view.view_length,) * self._view_dims, f"Miss match shape in the view, expect view to be {(self._view.view_length,) * self._view_dims}, got {value.shape[-self._view_dims:]}"
        assert value.shape[:-self._view_dims] == self._arr.shape[:-self._view_dims], f"Miss match shape in the view, expect view to be {self._arr.shape[:-self._view_dims]}, got {value.shape[:-self._view_dims]}"
        ArrayView._mapping_setter_(self._arr, value, self._view_dims, self._view.slice_mapping)

    def __repr__(self) -> str:
        return f"{self._view} in {self._arr}"


class MeanView(ArrayView):
    def __init__(self, m: np.ndarray, view: KineticView) -> None:
        super().__init__(m, view, 1)


class CovarianceView(ArrayView):
    def __init__(self, P: np.ndarray, view: KineticView) -> None:
        super().__init__(P, view, 2)


class KineticState:
    def __init__(self, kinetic_seq: str | set | list):
        self._view = KineticView(KineticView.format_seq(kinetic_seq))

        self._m = np.zeros(self._view.source_length)
        self._P = np.eye(self._view.source_length)

    def __len__(self):
        return self._view.source_length

    def predict_step(self, sampler: Sampler, dt: float):
        xs = sampler.sample(self._m, self._P)

        # noinspection PyTypeChecker
        ys: np.ndarray = solve_ivp(
            self.kinetics_ode,
            (0, dt),
            xs.flatten(),
            method='RK23', rtol=1e-3, atol=1e-3
        ).y[:, -1].reshape(-1, self._view.source_length)
        KineticState.normalize_(self.m)

        self._m, self._P = sampler.transfer_result(ys)

    def update_by_error(self, sampler: Sampler, get_expect: Callable[[np.ndarray], np.ndarray], observed_value: np.ndarray, observe_error: np.ndarray):
        xs = sampler.sample(self._m, self._P)
        ys = get_expect(xs)

        u, S, C = sampler.transfer_all(self._m, xs, ys)
        S = S + observe_error

        K = C @ np.linalg.inv(S)
        self._m = self._m + K @ (observed_value - u)
        self._P = self._P - K @ S @ K.T
        KineticState.normalize_(self.m)

    # @property
    # def raw_m(self) -> np.ndarray:
    #     return self._m
    #
    # @raw_m.setter
    # def raw_m(self, m: np.ndarray):
    #     self._m = m
    #
    # @property
    # def raw_P(self) -> np.ndarray:
    #     return self._P
    #
    # @raw_P.setter
    # def raw_P(self, P: np.ndarray):
    #     self._P = P

    @property
    def view(self) -> KineticView:
        return self._view

    @staticmethod
    def normalize_(m: MeanView):
        if 'Q' in m:
            m['Q'] /= np.linalg.norm(m['Q'], axis=-1, keepdims=True)

    @property
    def m(self) -> MeanView:
        return MeanView(self._m, self._view)

    @property
    def P(self) -> CovarianceView:
        return CovarianceView(self._P, self._view)

    def kinetics_ode(self, _: float, kinetics: np.ndarray) -> np.ndarray:
        # un-package & normalize
        kinetics = kinetics.reshape(-1, self._view.source_length).copy()
        d_kinetics = MeanView(np.zeros_like(kinetics), self._view)
        kinetics = MeanView(kinetics, self._view)
        self.normalize_(kinetics)

        if 'Q' in kinetics and 'O' in kinetics:
            angular_velocity = kinetics.get_ref_('O')
            augmented_angular_velocity = np.concatenate([np.zeros([len(angular_velocity), 1]), angular_velocity], axis=-1)

            d_kinetics['Q'] = .5 * quaternion_raw_multiply(augmented_angular_velocity, kinetics['Q'])
        if 'O' in kinetics and 'B' in kinetics:
            d_kinetics['O'] = kinetics['B']

        if 'P' in kinetics and 'V' in kinetics:
            d_kinetics['P'] = kinetics['V']
        if 'V' in kinetics and 'A' in kinetics:
            d_kinetics['V'] = kinetics['A']
        if 'A' in kinetics and 'J' in kinetics:
            d_kinetics['A'] = kinetics['J']

        return d_kinetics.value.flatten()
