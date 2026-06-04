import scipy
import numpy as np

from tracker.lidar.rotation import quaternion_to_matrix
from tracker.lidar.distribution_transfer import get_unscented_sigma_points

TolType = float | tuple[float, float] | tuple[float, float, float, float] | tuple[tuple[float, float, float, float], tuple[float, float, float, float], tuple[float, float, float, float]]


class LidarTarget:
    @staticmethod
    def new_vec3(v_idx, a_idx, b_idx, v_val, a_val, b_val):
        assert 0 <= v_idx < 3 and 0 <= a_idx < 3 and 0 <= b_idx < 3
        assert v_idx != a_idx and a_idx != b_idx and b_idx != v_idx
        res = np.empty(3)
        res[v_idx] = v_val
        res[a_idx] = a_val
        res[b_idx] = b_val
        return res

    def __init__(self, side_length: float = 385., gap_length: float = 7., distance_std: float = 20, angle_std: float = np.deg2rad(0.05), quantization_std: float = 1.):
        self.side_length = side_length
        self.gap_length = gap_length
        self.distance_std = distance_std
        self.angle_std = angle_std
        self.quantization_std = quantization_std

        self.plane_axis_idxs = tuple(tuple(np.roll(np.arange(3), -shift)) for shift in range(3))

        # (plane, keypoint: (center, diag1, diag2), pos)
        self.key_points = np.array(
            [
                [
                    self.new_vec3(*idxs, 0, self.gap_length, self.gap_length),  # center
                    self.new_vec3(*idxs, 0, self.gap_length + self.side_length, self.gap_length),
                    self.new_vec3(*idxs, 0, self.gap_length, self.gap_length + self.side_length)
                ] for idxs in self.plane_axis_idxs
            ]
        )

        self.colors = np.eye(3).tolist()

    # fast check if there are points near target
    def fast_check(self, points: np.ndarray, tol: float = 100) -> bool:
        x, y, z = np.split(points, 3, axis=-1)
        x, y, z = x.flatten(), y.flatten(), z.flatten()

        return np.sum((-tol < x) & (-tol < y) & (-tol < z) & (x + y + z < self.gap_length + self.side_length + tol)) > 0

    def standardize_tolerance(self, tolerance: TolType) -> TolType:
        if not isinstance(tolerance, tuple):
            tolerance = (tolerance, tolerance)
        if len(tolerance) == 2:
            tolerance = (tolerance[0], self.gap_length + self.side_length - tolerance[1], self.gap_length + tolerance[1], self.gap_length + tolerance[1])
        if len(tolerance) == 4:
            tolerance = (tolerance, tolerance, tolerance)
        return tolerance

    def get_target_point_bool(self, points: np.ndarray, tolerance: TolType) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        tolerance = self.standardize_tolerance(tolerance)

        # noinspection PyTypeChecker
        return tuple(
            (np.abs(points[..., v_idx]) <= v_lim) &
            (np.sum(points[..., [a_idx, b_idx]], axis=-1) <= diag_min) &
            (av_max + np.maximum(0, points[..., v_idx]) < points[..., a_idx]) &
            (bv_max + np.maximum(0, points[..., v_idx]) < points[..., b_idx])
            for (v_idx, a_idx, b_idx), (v_lim, diag_min, av_max, bv_max)
            in zip(self.plane_axis_idxs, tolerance)
        )

    def get_target_point_indexes(self, points: np.ndarray, tolerance: TolType) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        yz_bool, xz_bool, xy_bool = self.get_target_point_bool(points, tolerance)

        yz_idx = np.arange(len(points))[yz_bool]
        xz_idx = np.arange(len(points))[xz_bool]
        xy_idx = np.arange(len(points))[xy_bool]

        return yz_idx, xz_idx, xy_idx

    def get_target_points(self, points: np.ndarray, tolerance: TolType) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        yz_bool, xz_bool, xy_bool = self.get_target_point_bool(points, tolerance)

        yz_points = points[yz_bool]
        xz_points = points[xz_bool]
        xy_points = points[xy_bool]

        return yz_points, xz_points, xy_points

    @staticmethod
    def get_orthogonal_frame_by_vector(vec: np.ndarray) -> np.ndarray:
        norm_vec = vec / np.linalg.norm(vec)
        # noinspection PyUnreachableCode
        cross_1 = np.cross(norm_vec, np.eye(3)[np.argmin(norm_vec)])
        # noinspection PyUnreachableCode
        cross_2 = np.cross(norm_vec, cross_1)
        return np.stack([norm_vec, cross_1, cross_2])

    def get_tolerance(
            self, m: np.ndarray, P: np.ndarray,
            p_confidence_level: float = .2,
            v_confidence_level: float = .05,
            clamp: tuple[tuple[float, float], tuple[float, float]] = ((10, 100), (5, 50))
    ) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float], tuple[float, float, float, float]]:
        k_v = scipy.stats.norm.ppf(1 - v_confidence_level / 2)
        k_p = scipy.stats.norm.ppf(1 - p_confidence_level)
        (min_v, max_v), (min_p, max_p) = clamp

        mq, mp = np.split(m, [4, ], axis=-1)

        augmented_m = np.concatenate([m, np.zeros(3)])
        augmented_P = scipy.linalg.block_diag(P, np.diag(np.array([self.distance_std * 1e-3, self.angle_std, self.angle_std]) ** 2))

        xs, wm, wc = get_unscented_sigma_points(augmented_m, augmented_P)
        qs, ps, err = np.split(xs, [4, 7], axis=-1)
        qs = qs / np.linalg.norm(qs, axis=-1, keepdims=True)

        r = np.linalg.norm(mp)
        key_points_global = self.key_points[:, None, :, :] @ np.transpose(quaternion_to_matrix(qs), (0, 2, 1)) + 1e3 * ps[:, None, :]
        key_points_global = key_points_global + ((err * np.array([1, r, r]) * 1e3) @ self.get_orthogonal_frame_by_vector(mp))[:, None, :]
        ys = (key_points_global - mp * 1e3) @ quaternion_to_matrix(mq)

        def clamp(value, minimum, maximum):
            return max(minimum, min(value, maximum))

        def get_tol(pts: np.ndarray, v_idx: int, a_idx: int, b_idx: int) -> tuple[float, float, float, float]:
            # pts: (plane, sample, keypoint, pos)
            v_lim = np.max(k_v * np.sqrt(np.sum(wc[:, None] * pts[v_idx, :, :, v_idx] ** 2, axis=0)))

            diag = np.sum(pts[v_idx, :, 1:, :][..., [a_idx, b_idx]], axis=-1)
            diag_avg = np.sum(wm[:, None] * diag, axis=0)
            diag_min = np.min(diag_avg - k_p * np.sqrt(np.sum(wc[:, None] * (diag - diag_avg) ** 2, axis=0)))

            av = pts[a_idx, :, :, a_idx] - np.maximum(0, pts[a_idx, :, :, v_idx])
            av_avg = np.sum(wm[:, None] * av, axis=0)
            av_max = np.max(av_avg + k_p * np.sqrt(np.sum(wc[:, None] * (av - av_avg) ** 2, axis=0)))

            bv = pts[b_idx, :, :, b_idx] - np.maximum(0, pts[b_idx, :, :, v_idx])
            bv_avg = np.sum(wm[:, None] * bv, axis=0)
            bv_max = np.max(bv_avg + k_p * np.sqrt(np.sum(wc[:, None] * (bv - bv_avg) ** 2, axis=0)))

            return (
                clamp(v_lim, min_v, max_v),
                clamp(diag_min, self.gap_length + self.side_length - max_p, self.gap_length + self.side_length - min_p),
                clamp(av_max, self.gap_length + min_p, self.gap_length + max_p),
                clamp(bv_max, self.gap_length + min_p, self.gap_length + max_p)
            )

        # yz-plane (abs(x) < 𝜏, y + z < 𝜏, x < y - 𝜏, x < z - 𝜏)
        # xz-plane (abs(y) < 𝜏, x + z < 𝜏, y < z - 𝜏, y < x - 𝜏)
        # xy-plane (abs(z) < 𝜏, x + y < 𝜏, z < x - 𝜏, z < y - 𝜏)
        # noinspection PyTypeChecker
        tolerance: TolType = tuple(get_tol(ys, *idxs) for idxs in self.plane_axis_idxs)

        return tolerance

    def get_tolerance_keypoint(self, tolerance: TolType) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        vertices = np.array([
            [
                self.new_vec3(*idxs, -v_lim, av_max, bv_max),
                self.new_vec3(*idxs, -v_lim, av_max, diag_min - av_max),
                self.new_vec3(*idxs, -v_lim, diag_min - bv_max, bv_max),
                self.new_vec3(*idxs, 0, av_max, bv_max),
                self.new_vec3(*idxs, 0, av_max, diag_min - av_max),
                self.new_vec3(*idxs, 0, diag_min - bv_max, bv_max),
                self.new_vec3(*idxs, +v_lim, v_lim + av_max, v_lim + bv_max),
                self.new_vec3(*idxs, +v_lim, v_lim + av_max, diag_min - av_max - v_lim),
                self.new_vec3(*idxs, +v_lim, diag_min - bv_max - v_lim, v_lim + bv_max),
            ]
            for idxs, (v_lim, diag_min, av_max, bv_max) in zip(self.plane_axis_idxs, self.standardize_tolerance(tolerance))
        ])

        lines = np.array([
            [
                [0, 1],
                [1, 2],
                [2, 0],
                [3, 4],
                [4, 5],
                [5, 3],
                [6, 7],
                [7, 8],
                [8, 6],
                [0, 3],
                [1, 4],
                [2, 5],
                [3, 6],
                [4, 7],
                [5, 8],
            ]
            for _ in self.plane_axis_idxs
        ]) + np.arange(3)[:, None, None] * vertices.shape[1]

        colors = np.array(self.colors)[:, None, :] * np.ones(lines.shape[1])[:, None]

        return vertices.reshape(-1, 3), lines.reshape(-1, 2), colors.reshape(-1, 3)

    def get_sigmas(self, view_points: np.ndarray) -> np.ndarray:
        r = np.linalg.norm(view_points)
        alpha = (view_points / r) ** 2
        beta = 1 - alpha

        return np.sqrt(alpha * self.distance_std ** 2 + beta * (r * self.angle_std * 1e3) ** 2 + self.quantization_std ** 2)
