import pathlib
import argparse
import numpy as np
import tkinter.filedialog
import matplotlib.pyplot as plt
from alive_progress import alive_it

from tracker.lidar.lvx_reader import LVXReader
from tracker.lidar.target_finder import rough_estimate_lidar_target, refine_lidar_target, plot_result
from tracker.lidar.lidar_tracker import LiDARTracker
from tracker.lidar.rotation import *
from tracker.lidar.lidar_target import LidarTarget


def run_tracking(tracker: LiDARTracker, data: LVXReader, start_pkg_idx: int = 0, backward_track: bool = False) -> LiDARTracker:
    if backward_track:
        title: str = 'Backward tracking'
        track_iter: tuple[int, int, int] = (start_pkg_idx - 1, -1, -1)
    else:
        title: str = 'Forward tracking'
        track_iter: tuple[int, int, int] = (start_pkg_idx + 1, data.size(), 1)

    smoother_start_idx = len(tracker.state.ts)
    track_len = track_iter[2] * (track_iter[1] - track_iter[0])
    for pkg_idx in alive_it(range(*track_iter), title=title, total=abs(track_len), length=60, max_cols=150, force_tty=True):
        header, xyz, _, tag = data.get_packet(pkg_idx)
        timestamp = data.timestamp_end[2] - header.timestamp if backward_track else header.timestamp

        tracker.predict(timestamp)
        tracker.lidar_update(xyz[data.tag_filter(tag)], timestamp, lidar_target)

        if np.sqrt(np.trace(tracker.state.P['P'], 0, -2, -1)) > .25:
            print(f'ERROR: loss of track in packet {pkg_idx}/{data.size()}')
            break

    if track_len > 0:
        tracker.smooth(start_idx=smoother_start_idx)

    if backward_track:
        tracker.state.ts = [data.timestamp_end[2] - ts for ts in tracker.state.ts]
        if track_len > 0:
            tracker.state.ts.reverse()
            tracker.state.ms.reverse()
            tracker.state.Ps.reverse()
            tracker.state.sms.reverse()
            tracker.state.sPs.reverse()
            tracker.state.m = tracker.state.sms[-1]
            tracker.state.P = tracker.state.sPs[-1]

    return tracker


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', type=str, help='Input file', default=None)
    parser.add_argument('-s', '--start_idx', type=int, default=0,
                        help='Start package index for tracking: 0 for forward tracking from the start; '
                             '-1 for backward tracking from the end; otherwise for bi-directional tracking from the given package')
    args = parser.parse_args()

    filename: str = tkinter.filedialog.askopenfilename(
        title="Select LVX point cloud File",
        filetypes=[("LVX Files", "*.lvx")],
        initialdir=pathlib.Path.cwd() / '../data/'
    ) if args.input is None else args.input

    print(f'Input file: {filename}')

    lidar_target = LidarTarget()
    with LVXReader(filename) as data:
        print(f"total length: {(data.get_packet(data.size() - 1).header.timestamp - data.get_packet(0).header.timestamp) * 1e-6:8,.1f}ms, "
              f"total num of packages: {data.size()}")

        accumulated_points = []
        start_time = None
        start_pkg_idx = args.start_idx if args.start_idx != -1 else data.size() - 1  # 63500

        # Adaptively adjust the accumulation window given the start package
        if start_pkg_idx - 1250 < 0:
            accu_range = (data.size(), )
        elif start_pkg_idx + 1250 > data.size():
            accu_range = (data.size() - 1, 0, -1)
        else:
            accu_range = (start_pkg_idx - 1250, data.size())

        for pkg_idx in range(*accu_range):
            header, xyz, *_ = data.get_packet(pkg_idx)

            if start_time is None:
                start_time = header.timestamp

            if abs(header.timestamp - start_time) > 1000e6:
                break

            accumulated_points.append(xyz)
        accumulated_points = np.concatenate(accumulated_points, axis=0)

        R, t = rough_estimate_lidar_target(accumulated_points, lidar_target, plot=False)
        R, t, q_cov, p_cov = refine_lidar_target(accumulated_points, R, t, lidar_target)
        # plot_result(accumulated_points, R, t, lidar_target)
        q = matrix_to_quaternion(R)

        print(
            f"initial: q={LiDARTracker.quaternion_to_str(q)}, p={LiDARTracker.vector_to_str(t)}, "
            f"q_std{LiDARTracker.vector_to_str(np.sqrt(np.diag(q_cov)), width=4, decimal=5)}, "
            f"p_std={LiDARTracker.vector_to_str(np.sqrt(np.diag(p_cov)), width=4, decimal=5)}"
        )

        q_cov[0, 0] = q_cov[1, 1]

        # tracker initialization
        header, xyz, _, tag = data.get_packet(start_pkg_idx)
        timestamp = data.timestamp_end[2] - header.timestamp
        tracker = LiDARTracker()
        tracker.reset_state(init_q=q, init_q_cov=q_cov, init_p=t*1e-3, init_p_cov=p_cov*1e-6, current_time=timestamp)
        tracker.lidar_update(xyz[data.tag_filter(tag)], timestamp, lidar_target)

        # backward tracking
        tracker = run_tracking(tracker, data, start_pkg_idx=start_pkg_idx, backward_track=True)

        # forward tracking
        tracker.state.time = tracker.state.ts[-1]
        tracker = run_tracking(tracker, data, start_pkg_idx=start_pkg_idx, backward_track=False)

        tracker.visualize_states()
        tracker.save(pathlib.Path(filename).with_suffix('.npz').as_posix())

    # (global - t) @ R == local
    # global == local @ R.T + t
    # local @ R.T @ R' + (t - t') @ R' <== R: True, R': Avg, t: True, t': Avg
