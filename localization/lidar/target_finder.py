import cv2
import math
import itertools
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from alive_progress import alive_bar, alive_it

from tracker.lidar.rotation import *
from tracker.lidar.lidar_target import LidarTarget


def orthogonalize(v1: np.ndarray, v2: np.ndarray, v3: np.ndarray) -> np.ndarray:
    return quaternion_to_matrix(matrix_to_quaternion(np.stack([v1, v2, v3], axis=-1)))


def imshow(arr, vmin: float | None = None, vmax: float | None = None, cmap: str = 'viridis'):
    plt.figure(figsize=(12, 4))
    plt.imshow(arr, origin='lower', vmin=vmin, vmax=vmax, cmap=cmap)
    plt.gca().invert_xaxis()
    plt.show()


def find_planes(
        points: np.ndarray,
        focus: float = 1500,
        h_fov: float = 81.7,
        v_fov: float = 25.1,
        max_depth: float = 30_000,
        depth_image_filling_kernel_size: int = 15,
        grad_kernel_size: int = 5,
        mask_filter_kernel_size: int = 15,
        floodfill_tol: float = .85,
        min_plane: float = 70 ** 2,
        plot: bool = False,
        eps: float = 1e-12
) -> list[tuple[int, np.ndarray, np.ndarray, float]]:
    depth_image_filling_kernel_size = (depth_image_filling_kernel_size, depth_image_filling_kernel_size)
    floodfill_tol = np.full(3, floodfill_tol)
    img_range = (slice(1, -1), slice(1, -1))
    masks_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mask_filter_kernel_size, mask_filter_kernel_size))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    width, height = int(np.ceil(2 * focus * np.tan(np.deg2rad(h_fov / 2)))), int(np.ceil(2 * focus * np.tan(np.deg2rad(v_fov / 2))))
    uv = np.transpose(np.mgrid[0:width, 0:height], (2, 1, 0)) - np.array([width / 2, height / 2]) + .5

    # build up open3d point cloud
    pcd = o3d.t.geometry.PointCloud()
    pcd.point.positions = o3d.core.Tensor(points[points[:, 0] > 0, :], o3d.core.float32)
    pcd.remove_statistical_outliers(nb_neighbors=30, std_ratio=2.0)

    # project point cloud into depth image
    #  <X, Y, Z> <-> <d, U, V> <-> <d, u / f * d, v / f * d>
    #  <Y / X * f, Z / X * f, X> <-> <U / d * f, V / d * f, d> <-> <u, v, d>
    intrinsic = o3d.core.Tensor([[focus, 0, width / 2.], [0, focus, height / 2.], [0, 0, 1]])
    extrinsics = o3d.core.Tensor([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
    depth_img = pcd.project_to_depth_image(width=width, height=height, intrinsics=intrinsic, extrinsics=extrinsics, depth_scale=1, depth_max=max_depth)
    depth_img = np.asarray(depth_img.to_legacy())

    # fill the pixel doesn't have points by average of surrounding pixels with points
    has_point = depth_img > 0
    depth_img[~has_point] = (cv2.GaussianBlur(depth_img, depth_image_filling_kernel_size, 0) / (cv2.GaussianBlur(has_point.astype(np.float32), depth_image_filling_kernel_size, 0) + eps))[~has_point]

    # get normal vector from the depth image
    dh_du, dh_dv = cv2.Sobel(depth_img, cv2.CV_64F, 1, 0, grad_kernel_size), cv2.Sobel(depth_img, cv2.CV_64F, 0, 1, grad_kernel_size)
    #     dh_du * u + dh_dv * v + C == -h  (using -h to make normal image look better)
    # ==> dh_du * (U / h * f) + dh_dv * (V / h * f) + h == C
    # ==> (dh_du, dh_dv, h / f) @ (U, V, h) == C
    norm_vec = np.stack([dh_du, dh_dv, depth_img / focus + eps], axis=-1)
    norm_vec /= np.linalg.norm(norm_vec, axis=-1, keepdims=True)

    if plot:
        imshow(norm_vec / 2 + .5)

    # get std of normal vector and depth
    norm_std = np.sqrt(np.sum(np.clip(cv2.GaussianBlur(norm_vec ** 2, depth_image_filling_kernel_size, 0) - cv2.GaussianBlur(norm_vec, depth_image_filling_kernel_size, 0) ** 2, a_min=0, a_max=None), axis=-1))
    depth_std = np.sqrt(np.clip(cv2.GaussianBlur(depth_img ** 2, depth_image_filling_kernel_size, 0) - cv2.GaussianBlur(depth_img, depth_image_filling_kernel_size, 0) ** 2, a_min=0, a_max=None)) / (depth_img + 1e-7)

    # get point position of the pixel in u-v
    UV = uv / focus * depth_img[..., None]

    # initial the masks
    planes = []
    new_tag = 2
    checked: np.ndarray = np.ones((height + 2, width + 2))  # 0: unknown, 1: barrier, >=2: tag of a plane
    checked[img_range] = ((depth_std > np.quantile(depth_std, .85)) | (depth_img <= 0)).astype(np.uint8)  # ignore areas with drastic depth or normal changes or lack of depth data
    source_masks_size = None
    if plot:
        imshow(checked)
    with alive_bar(title="searching possible planes", manual=True, length=60, max_cols=150, force_tty=True) as bar:
        while True:
            # filtering the checked (ignore some small areas)
            checked = cv2.morphologyEx(checked.astype(np.uint8), cv2.MORPH_CLOSE, masks_kernel)

            # get the possible position to start a floor fill
            source_masks = (checked[img_range] == 0) & has_point
            if not np.any(source_masks):
                bar(1.)
                break
            else:
                if source_masks_size is None:
                    source_masks_size = np.sum(source_masks)
                bar(1 - np.sum(source_masks) / source_masks_size)
            v, u = np.unravel_index(np.argmin(np.where(source_masks, norm_std, np.inf)), norm_std.shape)

            # find all points near (u, v) and have similar normal vector by flood fill
            cv2.floodFill(norm_vec.astype(np.float32), checked, (u, v), np.ones(3), floodfill_tol, floodfill_tol, flags=cv2.FLOODFILL_FIXED_RANGE | cv2.FLOODFILL_MASK_ONLY | (new_tag << 8) | 8)

            # get the masks of the found plane
            # noinspection PyTypeChecker
            plane_masks: np.ndarray = (checked == new_tag)
            plane_masks = cv2.morphologyEx(plane_masks.astype(np.uint8), cv2.MORPH_CLOSE, masks_kernel).astype(bool)

            if np.sum((depth_img[plane_masks[img_range]] / focus) ** 2) >= min_plane:
                # fit plane in the depth image (x == dx_dy * y + dx_dz * z + plane_r)
                dx_dy, dx_dz, plane_r = np.linalg.lstsq(np.concatenate([UV[plane_masks[img_range]], np.ones((np.sum(plane_masks[img_range]), 1))], axis=-1), depth_img[plane_masks[img_range]], rcond=None)[0]

                # get the model of plane as n @ v == r
                plane_norm = np.array([1, -dx_dy, -dx_dz])
                plane_norm, plane_r = plane_norm / np.linalg.norm(plane_norm), plane_r / np.linalg.norm(plane_norm)

                # get the center of the plane
                distance = np.zeros_like(plane_masks, dtype=np.uint8)
                distance[img_range] = plane_masks[img_range]
                distance = cv2.distanceTransform(distance, cv2.DIST_L2, 5)
                plane_center_index = np.unravel_index(np.argmax(distance[img_range]), depth_img.shape)
                plane_center = np.array([depth_img[plane_center_index], *UV[plane_center_index]])

                planes.append((np.sum(plane_masks), plane_center, plane_norm, plane_r))

                new_tag += 1  # change tag for next plane
            else:
                checked[plane_masks] = 1  # not a plane, setting the result as a barrier

    planes = sorted(planes, key=lambda p: (p[0], abs(p[2][0]), -p[3]), reverse=True)

    if plot:
        imshow(np.maximum(checked - 1, 0), cmap='jet')
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[(0 < np.linalg.norm(points, axis=-1)) & (np.linalg.norm(points, axis=-1) <= max_depth)])
        geometries = []
        for _, c, n, r in planes:
            idx = np.arange(len(pcd.points))[np.abs(np.sum(n * pcd.points, axis=-1) - r) < 25]
            p = pcd.select_by_index(idx)
            p.paint_uniform_color(np.random.rand(3))
            geometries.append(p)

            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=50.0)
            sphere.compute_vertex_normals()
            sphere.paint_uniform_color([1, 0, 0])
            sphere.translate(c)
            geometries.append(sphere)

            pcd = pcd.select_by_index(idx, invert=True)
        pcd.paint_uniform_color([0.8, 0.8, 0.8])
        o3d.visualization.draw_geometries([pcd, ] + geometries)

    return planes


def remove_outline(points: np.ndarray, radius: float = 50, min_num: int = 5) -> np.ndarray:
    return points[np.sum(np.linalg.norm(points[..., :, None, :] - points[..., None, :, :], axis=-1) < radius, axis=-1) > min_num]


def rough_estimate_lidar_target(points: np.ndarray, lidar_target: LidarTarget, max_depth: float = 30_000, plot: bool = False) -> tuple[None, None] | tuple[np.ndarray, np.ndarray]:
    planes = find_planes(points, max_depth=max_depth, plot=plot)

    out_rotation, out_translation = None, None
    out_plane_group = None
    max_area = 0
    # loop through all possible combinations of planes
    for plane_group in alive_it(itertools.combinations(planes, 3), title="finding lidar target", total=math.comb(len(planes), 3), length=60, max_cols=150, force_tty=True):
        plane_group = sorted(plane_group, key=lambda x: x[2][-1])  # sort the vector
        if np.linalg.det(np.stack(list(zip(*plane_group))[2])) < 0:  # ensure the det is +1 not -1
            plane_group = plane_group[0], plane_group[2], plane_group[1]
        # assert np.linalg.det(np.stack(plane_norms)) >= 0

        _, plane_centers, plane_norms, plane_rs = zip(*plane_group)

        # target planes are orthogonal, so det should be +1
        if np.linalg.det(np.stack(plane_norms)) < 0.9:
            continue

        # if the distance between the center points of the plane is too large, skip the group
        if max(np.linalg.norm(a - b) for a, b in itertools.combinations(plane_centers, 2)) > (lidar_target.side_length + lidar_target.gap_length) * np.sqrt(2):
            continue

        # build up the rotation & translation by plane group
        rotation = orthogonalize(*plane_norms)
        translation = np.linalg.solve(np.stack(plane_norms), np.array(plane_rs))

        # mapping the points into the target planes
        plane_point_groups = lidar_target.get_target_points((points - translation) @ rotation, 30)

        # if the points on any plane are too less, skip
        if min(len(point_group) for point_group in plane_point_groups) < 10:
            continue

        # finding the argmax[min(det(cov(pts)) in group) cross plane group]; larger det(cov(pts)) means more area, means more plane occupancy
        planes_pts = [remove_outline(point_group[:, [i for i in range(3) if i != idx]]) for idx, point_group in enumerate(plane_point_groups)]
        if (area := min(abs(np.linalg.det(np.cov(plane_pts.T))) if len(plane_pts) > 1 else 0 for plane_pts in planes_pts)) > max_area:
            out_rotation, out_translation = rotation, translation
            out_plane_group = plane_group
            max_area = area

    assert out_rotation is not None and out_translation is not None and out_plane_group is not None

    if plot:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[(0 < np.linalg.norm(points, axis=-1)) & (np.linalg.norm(points, axis=-1) <= max_depth)])
        pcd.paint_uniform_color([0.8, 0.8, 0.8])
        coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
        coordinate.translate(out_translation)
        coordinate.rotate(out_rotation)
        o3d.visualization.draw_geometries([pcd, coordinate])

    return out_rotation, out_translation


def refine_step(qs: np.ndarray, ps: np.ndarray, point_groups_list: list[tuple[np.ndarray, np.ndarray, np.ndarray]], epoch_size: int = 512, lr: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    assert qs.shape[-1] == 4 and ps.shape[-1] == 3

    qs, ps = torch.from_numpy(qs), torch.from_numpy(ps) * 1e-3

    qs.requires_grad_()
    ps.requires_grad_()

    optimizer = torch.optim.SGD([qs, ps], lr=lr)
    lr_scheduler = torch.optim.lr_scheduler.LinearLR(optimizer, start_factor=1, end_factor=1e-6, total_iters=epoch_size)

    # loss = []
    for _ in range(epoch_size):
        optimizer.zero_grad()

        m = quaternion_to_matrix(torch.nn.functional.normalize(qs, p=2, dim=-1))
        # noinspection PyTypeChecker
        err: torch.Tensor = 0.
        for idx, point_groups in enumerate(point_groups_list):
            err += sum(((torch.from_numpy(point_group * 1e-3) - ps[idx]) @ m[idx])[..., v_idx].square().mean() for v_idx, point_group in enumerate(point_groups)) / len(point_groups)
        err.backward()

        optimizer.step()
        lr_scheduler.step()
        # loss.append(err.item())
    # plt.plot(loss)
    # plt.show()
    return torch.nn.functional.normalize(qs, p=2, dim=-1).numpy(force=True), ps.numpy(force=True) * 1e3


def refine_lidar_target(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray, lidar_target: LidarTarget, max_depth: float = 10., plot: bool = False, do_bootstrap: bool = True) \
        -> tuple[None, None, None, None] | tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | tuple[np.ndarray, np.ndarray]:
    tols = [40, 25, 15, 10]
    lr = [1e-1, 1e-2, 5e-3, 1e-3]
    with alive_bar(len(tols), title="refine lidar target's position & posture", length=60, max_cols=150, force_tty=True) as bar:
        for tol, lr in zip(tols, lr):
            # get points under initial rotation and translation
            point_groups = lidar_target.get_target_points((points - translation) @ rotation, tol)

            # setup initial variable and prepare for gradient descent
            # the rotation and translation after the initial rotation and translation
            q, p = np.zeros(4), np.zeros(3)
            q[0] = 1

            q, p = refine_step(np.expand_dims(q, 0), np.expand_dims(p, 0), [point_groups], lr=lr)

            bar()

            # get combined rotation and translation
            q, p = q.squeeze(axis=0), p.squeeze(axis=0)
            m = quaternion_to_matrix(q)
            rotation, translation = rotation @ m, translation + p @ m @ rotation.T @ m.T

    if not do_bootstrap:
        return rotation, translation

    # Bootstrap
    n_resamples: int = 200
    n_pts: int = 200
    q_samples, p_samples = np.expand_dims(q, 0).repeat(n_resamples, 0), np.expand_dims(p, 0).repeat(n_resamples, 0)
    point_groups_set = []
    for _ in range(n_resamples):
        rng = np.random.default_rng()
        # noinspection PyTypeChecker
        point_groups_resample: tuple[np.ndarray, np.ndarray, np.ndarray] = tuple([rng.choice(pts, min(n_pts, pts.shape[0]), axis=0) for pts in point_groups])
        point_groups_set.append(point_groups_resample)

    print('bootstrap...')
    q_samples, p_samples = refine_step(q_samples, p_samples, point_groups_set, epoch_size=128, lr=1e-3)

    p_cov = np.sum((p_samples - p)[:, :, None] * (p_samples - p)[:, None, :], axis=0) / n_resamples
    q_cov = np.sum((q_samples - q)[:, :, None] * (q_samples - q)[:, None, :], axis=0) / n_resamples

    if plot:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[(0 < np.linalg.norm(points, axis=-1)) & (np.linalg.norm(points, axis=-1) <= max_depth)])
        pcd.paint_uniform_color([0.8, 0.8, 0.8])

        pcd_target = o3d.geometry.PointCloud()
        pcd_target.points = o3d.utility.Vector3dVector(np.concatenate(point_groups, axis=0))
        pcd_target.paint_uniform_color(([1.0, 0., 0.]))

        coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
        coordinate.translate(translation)
        coordinate.rotate(rotation)
        o3d.visualization.draw_geometries([pcd, pcd_target, coordinate])

    return rotation, translation, q_cov, p_cov


def plot_result(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray, lidar_target: LidarTarget, tolerance: float = 25, max_depth: float = 30_000):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    points = points[(0 < np.linalg.norm(points, axis=-1)) & (np.linalg.norm(points, axis=-1) <= max_depth)]
    idxs = lidar_target.get_target_point_bool((points - translation) @ rotation, tolerance)
    idxs = idxs + (~(idxs[0] | idxs[1] | idxs[2]),)
    colors = lidar_target.colors + [[.85, ] * 3, ]

    for idx, color in zip(idxs, colors):
        pts = points[idx]

        if len(pts) == 0:
            continue

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.paint_uniform_color(color)

        vis.add_geometry(pcd)

    coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame(size=lidar_target.side_length + lidar_target.gap_length)
    coordinate.translate(translation)
    coordinate.rotate(rotation)
    vis.add_geometry(coordinate)

    vs, ls, cs = lidar_target.get_tolerance_keypoint(tolerance)
    vs = vs @ rotation.T + translation
    decision_boundary = o3d.geometry.LineSet()
    decision_boundary.points = o3d.utility.Vector3dVector(vs)
    decision_boundary.lines = o3d.utility.Vector2iVector(ls)
    decision_boundary.colors = o3d.utility.Vector3dVector(cs)
    vis.add_geometry(decision_boundary)

    vis.get_view_control().set_up((0, 0, 1))
    vis.get_view_control().set_front((-1, 0, 0))
    vis.get_view_control().camera_local_translate(translation[0] - 1000, 0, 0)
    vis.get_view_control().set_lookat(translation)
    vis.get_view_control().set_zoom(0.1)

    vis.run()
    vis.destroy_window()