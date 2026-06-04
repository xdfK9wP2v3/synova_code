import numpy as np
import scipy


class StateVariable:
    def __init__(self, kinetic_var: set[str]):
        var_idx = {'Q': 0, 'O': 1, 'B': 2, 'P': 3, 'V': 4, 'A': 5, 'J': 6}
        var_len = {'Q': 4, 'O': 3, 'B': 3, 'P': 3, 'V': 3, 'A': 3, 'J': 3}
        var_cumsum: list[int] = np.cumsum([0, ] + [int(var_len[var]) for var in kinetic_var])
        self.kinetic_var = kinetic_var
        self.size: int = var_cumsum[-1]

        # Get kinetic variable slice and indices
        idx = 0
        for var in var_idx.keys():
            if var in kinetic_var:
                exec(f'self.{var.lower()}_slice = slice({var_cumsum[idx]}, {var_cumsum[idx + 1]})')
                exec(f'self.{var.lower()}_idx = list(range({self.size}))[self.{var.lower()}_slice]')
                idx += 1
            else:
                exec(f'self.{var.lower()}_slice = None')

        self.state_var = None

    @property
    def value(self):
        return self.state_var

    @value.setter
    def value(self, new_value):
        self.state_var = new_value

    def get_indices(self, item: str):
        # TO DO: check whether the item is legal

        if item == 'ALL':
            return slice(None)

        # Compatible with augmented states
        if item == 'VALID':
            return slice(self.size)

        if item[0] == '~':
            item = [var for var in self.kinetic_var if var not in item]

        ret = []
        for var in item:
            if var in self.kinetic_var:
                match var:
                    case 'Q':
                        ret.append(self.q_idx)
                    case 'O':
                        ret.append(self.o_idx)
                    case 'B':
                        ret.append(self.b_idx)
                    case 'P':
                        ret.append(self.p_idx)
                    case 'V':
                        ret.append(self.v_idx)
                    case 'A':
                        ret.append(self.a_idx)
                    case 'J':
                        ret.append(self.j_idx)
            else:
                raise ValueError(f'Kinetic variable {var} is not available!')
        return np.concatenate(ret, axis=0)


class MeanState(StateVariable):
    def __init__(self, kinetic_var: set[str], state_var: np.ndarray | None = None):
        super().__init__(kinetic_var)
        self.state_var = state_var if state_var is not None else np.zeros(self.size)

    def normalize_q(self):
        if 'Q' in self.kinetic_var:
            self.state_var[..., self.q_slice] /= np.linalg.norm(self.state_var[..., self.q_slice], axis=-1, keepdims=True)

    @staticmethod
    def normalize_Q(kinetic_var: set[str], m: np.ndarray) -> np.ndarray:
        if 'Q' in kinetic_var:
            q_slice = slice(0, 4)
            m[..., q_slice] /= np.linalg.norm(m[..., q_slice], axis=-1, keepdims=True)

        return m

    def __getitem__(self, key: str):
        return self.state_var[..., self.get_indices(key)]

    def __setitem__(self, key: str, value: np.ndarray | None):
        if value is not None:
            self.state_var[..., self.get_indices(key)] = value


class CovState(StateVariable):
    def __init__(self, kinetic_var: set[str], state_var: np.ndarray | None = None):
        super().__init__(kinetic_var)
        var_cov = {'Q': np.eye(4) * 1e-2 ** 2, 'O': np.eye(3) * 1e-1 ** 2, 'B': np.eye(3) * 1e-2 ** 2,
                   'P': np.eye(3) * 1e-1 ** 2, 'V': np.eye(3) * 1e-1 ** 2, 'A': np.eye(3) * 1e-2 ** 2,
                   'J': np.eye(3) * 1e-3 ** 2}  # TODO: Need fine-tuning
        self.state_var = state_var if state_var is not None else scipy.linalg.block_diag(*[var_cov[var] for var in self.kinetic_var])

    def __getitem__(self, key: str | tuple[str, str]):
        if type(key) is str:
            return self.state_var[..., self.get_indices(key), :][..., :, self.get_indices(key)]
        elif type(key) is tuple:
            assert len(key) == 2
            return self.state_var[..., self.get_indices(key[0]), :][..., :, self.get_indices(key[1])]
        else:
            raise TypeError('Unsupported key type!')

    def __setitem__(self, key: str | tuple[str, str], value: np.ndarray | None):
        if value is not None:
            if type(key) is str:
                cov_slice = self.state_var[..., self.get_indices(key), :]
                cov_slice[..., :, self.get_indices(key)] = value
                self.state_var[..., self.get_indices(key), :] = cov_slice
            elif type(key) is tuple:
                assert len(key) == 2
                cov_slice = self.state_var[..., self.get_indices(key[0]), :]
                cov_slice[..., :, self.get_indices(key[1])] = value
                self.state_var[..., self.get_indices(key[0]), :] = cov_slice
            else:
                raise TypeError('Unsupported key type!')


class State:
    def __init__(self, kinetic_var: set[str]):
        var_idx = {'Q': 0, 'O': 1, 'B': 2, 'P': 3, 'V': 4, 'A': 5, 'J': 6}
        self.kinetic_var = kinetic_var
        self.kinetic_var = sorted(self.kinetic_var, key=lambda x: var_idx[x])

        # Initialize m, P and time
        self.m = MeanState(self.kinetic_var)
        self.P = CovState(self.kinetic_var)
        self.time: float = 0      # TODO: int or float?
        self.size: int = self.m.size

        # Initialize state list
        self.ts = []
        self.ms = []
        self.Ps = []
        self.sms = []
        self.sPs = []
        self.input = []