import time
import pathlib
import cv2
import scipy
import open3d as o3d
from tracker.tracker import *
from tracker.lidar.rotation import *
from tracker.lidar.lidar_target import LidarTarget
from tracker.lidar.distribution_transfer import get_monte_carlo_importance_sampling_points
from tracker.lidar.lvx_reader import LVXReader
from tracker.lidar.debug import get_target_distribution


class LiDARTracker(Tracker):
    def __init__(self):
        super().__init__()
        print('Init LiDARTracker')
        self.kinetic_var |= {'Q', 'O', 'P', 'V'}
        self.sensor.append('LIDAR')

        # Note! Convert mm -> m

    @staticmethod
    def evaluate_state(
            m_q: np.ndarray, m_p: np.ndarray,
            qs: np.ndarray, ps: np.ndarray,
            observed: np.ndarray, pts_bools: tuple[np.ndarray, np.ndarray, np.ndarray],
            lidar_target: LidarTarget
    ) -> np.ndarray:
        R = quaternion_to_matrix(qs)
        t = ps[:, None, :] * 1e3

        probs = np.concatenate(
            [
                scipy.stats.norm(loc=0, scale=sigma).pdf(((observed[pts_bool] - t) @ R)[..., plane_idx])
                for plane_idx, (pts_bool, sigma)
                in enumerate(zip(pts_bools, lidar_target.get_sigmas(-m_p @ quaternion_to_matrix(m_q))))
            ], axis=-1
        )

        return np.prod(probs, axis=-1)

    def monte_carlo_update(self,
            m: MeanState, P: CovState,
            m_q: np.ndarray, m_p: np.ndarray,
            observed: np.ndarray, pts_bools: tuple[np.ndarray, np.ndarray, np.ndarray],
            lidar_target: LidarTarget,
            sample_num: int = 2 * 1024
    ) -> tuple[MeanState, CovState]:
        # get sample points (posture & position only) for Bayesian inference
        xs, ws = get_monte_carlo_importance_sampling_points(m['QP'], P['QP'], sample_num=sample_num)
        xs = MeanState.normalize_Q(self.kinetic_var, xs)
        qs, ps = np.split(xs, [4, ], axis=-1)

        # @@ require further improvement and test
        # get the angular velocity & velocity distribution for a given posture & position
        K = P[('~QP', 'QP')] @ np.linalg.inv(P['QP'])
        mz_x = m['~QP'] + (xs - m['QP']) @ K.T
        Pz_x = P['~QP'] - K @ P[('QP', '~QP')]

        # calculate the posterior probability of the sample based on position and posture
        ws *= self.evaluate_state(m_q, m_p, qs, ps, observed, pts_bools, lidar_target)
        ws /= np.sum(ws)

        # the mean and covariance of posture & position
        mx = np.sum(ws[:, None] * xs, axis=0)
        mx[:4] /= np.linalg.norm(mx[:4])
        Px = np.sum(ws[:, None, None] * (xs - mx)[:, :, None] * (xs - mx)[:, None, :], axis=0)

        # the uncertainty introduced by the Monte Carlo method (empirical formula)
        monte_carlo_error = 5e-2 / np.sqrt(sample_num)
        Px = Px + monte_carlo_error ** 2 * np.eye(7)

        # the mean and covariance of angular velocity & velocity
        mz = np.sum(ws[:, None] * mz_x, axis=0)
        Pz = Pz_x + np.sum(ws[:, None, None] * (mz_x - mz)[:, :, None] * (mz_x - mz)[:, None, :], axis=0)

        # the covariance of posture & position v.s. angular velocity & velocity
        Pxz = np.sum(ws[:, None, None] * (mz_x - mz)[:, :, None] * (xs - mx)[:, None, :], axis=0)

        # package up
        m = np.block([mx[:4], mz[:3], mx[4:], mz[3:]])
        # noinspection PyTypeChecker
        P = np.block(
            [
                [Px[:4, :4], Pxz[:3, :4].T, Px[:4, 4:], Pxz[3:, :4].T, ],
                [Pxz[:3, :4], Pz[:3, :3], Pxz[:3, 4:], Pz[:3, 3:], ],
                [Px[4:, :4], Pxz[:3, 4:].T, Px[4:, 4:], Pxz[3:, 4:].T, ],
                [Pxz[3:, :4], Pz[3:, :3], Pxz[3:, 4:], Pz[3:, 3:], ]
            ]
        )

        return MeanState(self.kinetic_var, m), CovState(self.kinetic_var, P)

    def lidar_update(self, observed: np.ndarray, current_time: float, lidar_target: LidarTarget, step_end: bool = True) -> None:
        m_q = self.state.m['Q']
        m_p = self.state.m['P']
        points_local = (observed - m_p * 1e3) @ quaternion_to_matrix(m_q)

        # fast check if there are points near target
        if lidar_target.fast_check(points_local):
            tol = lidar_target.get_tolerance(self.state.m['QP'], self.state.P['QP'])
            pts_bools = lidar_target.get_target_point_bool(points_local, tol)

            if any(np.any(pts_bool) for pts_bool in pts_bools):
                # plot_possible_planes(self.m, self.P, observed, lidar_target, tol)
                self.state.m, self.state.P = self.monte_carlo_update(self.state.m, self.state.P, m_q, m_p, observed, pts_bools, lidar_target)

        if step_end:
            self.save_step(current_time, 'LIDAR')

    def visualize_lidar_results(self, filename: str, dt: int = 100 * 1e6, output_size: tuple[int, int] = (1920, 1080),
                                plot_range: tuple[int, int] = (0, 25), speed: int = 2) -> None:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        output_movie = cv2.VideoWriter(pathlib.Path(filename).with_suffix('.mp4').as_posix(), fourcc, int(1e9 / dt), output_size)
        lidar_target = LidarTarget()

        with LVXReader(filename) as data:
            start_time = data.get_packet(0).header.timestamp
            end_time = data.get_packet(data.size() - 1).header.timestamp

            print(f"total length: {(end_time - start_time) * 1e-6:8,.1f}ms")

            vis = o3d.visualization.Visualizer()
            vis.create_window()

            point_clouds: o3d.geometry.PointCloud | None = None
            decision_boundary: o3d.geometry.LineSet | None = None
            planes_distribution: o3d.geometry.LineSet | None = None

            t = self.state.ts[0]
            img_idx = 0
            wall_time = time.time()
            pts = np.empty((0, 3))
            ts = np.empty((0,))
            for pkg_idx in range(data.size()):
                header, xyz, *_ = data.get_packet(pkg_idx)
                timestamp = header.timestamp

                if timestamp < t:
                    continue

                if timestamp < t + dt:
                    pts = np.concatenate([pts, xyz], axis=0)
                    ts = np.concatenate([ts, np.ones(len(xyz)) * timestamp], axis=0)
                else:
                    m, P = self(t + dt / 2)
                    avg_q, avg_o, avg_p, avg_v = m['Q'], m['O'], m['P'], m['V']

                    print(
                        f"idx={pkg_idx:8,d}, time={(timestamp - start_time) * 1e-6:9,.2f}ms, "
                        f"q={LiDARTracker.quaternion_to_str(avg_q)}, ω={LiDARTracker.vector_to_str(avg_o, 6, 3)}, "
                        f"p={LiDARTracker.vector_to_str(avg_p * 1e3)}, v={LiDARTracker.vector_to_str(avg_v * 1e3)}"
                    )

                    tol = lidar_target.get_tolerance(m['QP'], P['QP'])
                    pts_filter = (plot_range[0] * 1e3 < np.linalg.norm(pts, axis=-1)) & (
                                np.linalg.norm(pts, axis=-1) < plot_range[1] * 1e3)
                    ts, pts = ts[pts_filter], pts[pts_filter]

                    pts_color = np.ones([len(pts), 3]) * (.25 + .5 * np.abs(ts - (t + dt / 2)) / (dt / 2))[:, None]
                    for idx, color in zip(
                            lidar_target.get_target_point_bool((pts - avg_p * 1e3) @ quaternion_to_matrix(avg_q), tol),
                            lidar_target.colors):
                        pts_color[idx] = color

                    new_pcd = point_clouds is None
                    if new_pcd:
                        point_clouds = o3d.geometry.PointCloud()
                    point_clouds.points = o3d.utility.Vector3dVector(pts)
                    point_clouds.colors = o3d.utility.Vector3dVector(pts_color)

                    if new_pcd:
                        vis.add_geometry(point_clouds)
                        vis.get_view_control().set_up((0, 0, 1))
                        vis.get_view_control().set_front((-2, 1, 0))
                        vis.get_view_control().set_zoom(0.1)
                        vis.get_render_option().line_width = 6
                        vis.get_render_option().point_size = 3
                    else:
                        vis.update_geometry(point_clouds)
                    vis.get_view_control().set_lookat(avg_p * 1e3)

                    vs, ls, cs = lidar_target.get_tolerance_keypoint(tol)
                    vs = vs @ quaternion_to_matrix(avg_q).T + avg_p * 1e3
                    new_db = decision_boundary is None
                    if new_db:
                        decision_boundary = o3d.geometry.LineSet()
                    decision_boundary.points = o3d.utility.Vector3dVector(vs)
                    if new_db:
                        decision_boundary.lines = o3d.utility.Vector2iVector(ls)
                        decision_boundary.colors = o3d.utility.Vector3dVector(cs)
                        vis.add_geometry(decision_boundary, reset_bounding_box=False)
                    else:
                        vis.update_geometry(decision_boundary)

                    vs, ls, cs = get_target_distribution(m.value, P.value, lidar_target)
                    new_pd = planes_distribution is None
                    if new_pd:
                        planes_distribution = o3d.geometry.LineSet()
                    planes_distribution.points = o3d.utility.Vector3dVector(vs)
                    if new_pd:
                        planes_distribution.lines = o3d.utility.Vector2iVector(ls)
                        planes_distribution.colors = o3d.utility.Vector3dVector(cs)
                        vis.add_geometry(planes_distribution, reset_bounding_box=False)
                    else:
                        vis.update_geometry(planes_distribution)

                    coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame(size=350)
                    coordinate.translate(avg_p * 1e3)
                    coordinate.rotate(quaternion_to_matrix(avg_q))
                    vis.add_geometry(coordinate, reset_bounding_box=False)

                    vis.update_renderer()
                    vis.poll_events()
                    while time.time() - wall_time < dt * 1e-9 / speed:
                        vis.poll_events()
                    wall_time = time.time()
                    output_movie.write(np.clip(255. * cv2.resize(
                        cv2.cvtColor(np.asarray(vis.capture_screen_float_buffer(False)), cv2.COLOR_RGB2BGR),
                        output_size), a_min=0, a_max=255).astype(np.uint8))

                    vis.remove_geometry(coordinate, reset_bounding_box=False)

                    t += dt
                    img_idx += 1

                    ts, pts = np.ones(len(xyz)) * timestamp, xyz

            output_movie.release()
            vis.destroy_window()
