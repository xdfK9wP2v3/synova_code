import open3d as o3d
from tracker.lidar.lidar_target import *


def get_target_distribution(m: np.ndarray, P: np.ndarray, lidar_target: LidarTarget, sample_num: int = 256, base_color: float = .8, plane_color: float = .15) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    avg_q, _, avg_p, _ = np.split(m, [4, 7, 10], axis=-1)
    q_p_idx = [0, 1, 2, 3, 7, 8, 9]

    augmented_m = np.concatenate([m[q_p_idx], np.zeros(3)])
    augmented_P = scipy.linalg.block_diag(P[q_p_idx, :][:, q_p_idx], np.diag(np.array([lidar_target.distance_std * 1e-3, lidar_target.angle_std, lidar_target.angle_std]) ** 2))

    xs = scipy.stats.qmc.MultivariateNormalQMC(augmented_m, augmented_P).random(sample_num)
    qs, ps, err = np.split(xs, [4, 7], axis=-1)
    qs = qs / np.linalg.norm(qs, axis=-1, keepdims=True)

    r = np.linalg.norm(avg_p)
    key_points_global = lidar_target.key_points[:, None, :, :] @ np.transpose(quaternion_to_matrix(qs), (0, 2, 1)) + 1e3 * ps[:, None, :]
    key_points_global = key_points_global + ((err * np.array([1, r, r]) * 1e3) @ lidar_target.get_orthogonal_frame_by_vector(avg_p))[:, None, :]
    key_lines_global = np.array([[[[0, 1], [1, 2], [2, 0], ], ] * key_points_global.shape[1], ] * key_points_global.shape[0])
    key_lines_global = key_lines_global + np.arange(key_points_global.shape[1])[:, None, None] * key_points_global.shape[2]
    key_lines_global = key_lines_global + np.arange(key_points_global.shape[0])[:, None, None, None] * key_points_global.shape[1] * key_points_global.shape[2]

    return key_points_global.reshape(-1, 3), key_lines_global.reshape(-1, 2), ((base_color + plane_color * np.array(lidar_target.colors))[:, None, :] * np.ones(key_points_global.shape[1] * key_points_global.shape[2])[:, None]).reshape(-1, 3)


def plot_possible_planes(m: np.ndarray, P: np.ndarray, observed: np.ndarray, lidar_target: LidarTarget, tol: TolType):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    avg_q, avg_o, avg_p, avg_v = np.split(m, [4, 7, 10], axis=-1)
    R = quaternion_to_matrix(avg_q)

    pts = observed[(0 < np.linalg.norm(observed, axis=-1)) & (np.linalg.norm(observed, axis=-1) < 10 * 1e3)]
    pts_color = np.ones([len(pts), 3]) * .85
    for idx, color in zip(lidar_target.get_target_point_bool((pts - avg_p * 1e3) @ R, tol), lidar_target.colors):
        pts_color[idx] = color
    point_clouds = o3d.geometry.PointCloud()
    point_clouds.points = o3d.utility.Vector3dVector(pts)
    point_clouds.colors = o3d.utility.Vector3dVector(pts_color)
    vis.add_geometry(point_clouds)

    vs, ls, cs = lidar_target.get_tolerance_keypoint(tol)
    vs = vs @ quaternion_to_matrix(avg_q).T + avg_p * 1e3
    decision_boundary = o3d.geometry.LineSet()
    decision_boundary.points = o3d.utility.Vector3dVector(vs)
    decision_boundary.lines = o3d.utility.Vector2iVector(ls)
    decision_boundary.colors = o3d.utility.Vector3dVector(cs)
    vis.add_geometry(decision_boundary)

    coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame(size=350)
    coordinate.translate(avg_p * 1e3)
    coordinate.rotate(R)
    vis.add_geometry(coordinate)

    vs, ls, cs = get_target_distribution(m, P, lidar_target)
    possible_planes = o3d.geometry.LineSet()
    possible_planes.points = o3d.utility.Vector3dVector(vs)
    possible_planes.lines = o3d.utility.Vector2iVector(ls)
    possible_planes.colors = o3d.utility.Vector3dVector(cs)
    vis.add_geometry(possible_planes)

    vis.run()
    vis.destroy_window()
