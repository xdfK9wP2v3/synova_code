import os
import cv2
import time
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
from datetime import datetime


def visualize_frame(lidar_data: dict):
    xyz = lidar_data[:, :3]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz)
    o3d.visualization.draw_geometries([pc])


def visualize_sequence(lidar_data_seq, frame_interval=0.1):
    frames = len(lidar_data_seq['data'])
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Livox Lidar", width=1280, height=720)
    render_option = vis.get_render_option()
    render_option.background_color = np.asarray([255, 255, 255])
    pcd = o3d.geometry.PointCloud()

    for i in range(frames):
        points = lidar_data_seq['data'][i]
        if points.shape[0] == 0:
            continue

        xyz = points[:, :3]

        vis.add_geometry(pcd)
        vis.get_view_control().set_up((0, 0, 1))
        vis.get_view_control().set_front((-1, 0, 0))
        vis.get_view_control().set_zoom(0.005)
        vis.get_view_control().set_lookat((-1, 0, 2))
        vis.get_render_option().line_width = 6
        vis.get_render_option().point_size = 3

        vis.update_geometry(pcd)
        distances = np.linalg.norm(points, axis=-1)
        d_min, d_max = distances.min(), distances.max()
        d_normalized = (distances - d_min) / (d_max - d_min)

        colormap = plt.get_cmap("turbo")
        colors = colormap(d_normalized)[:, :3]

        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        vis.update_renderer()
        vis.poll_events()
        time.sleep(frame_interval)

    vis.destroy_window()


def visualize_sequence_to_video(lidar_data_seq, output_path="lidar_sequence.mp4", fps=10):
    frames = len(lidar_data_seq['data'])

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Livox Lidar", width=1920, height=1080, visible=False)

    render_option = vis.get_render_option()
    render_option.background_color = np.asarray([1, 1, 1])
    render_option.line_width = 6
    render_option.point_size = 2

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    vis.get_view_control().set_up((0, 0, 1))
    vis.get_view_control().set_front((-1, 0, 0))
    vis.get_view_control().set_zoom(0.005)
    vis.get_view_control().set_lookat((-1, 0, 2))

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, (1920, 1080))

    try:
        for i in range(frames):
            points = lidar_data_seq['data'][i]
            if points.shape[0] == 0:
                continue

            xyz = points[:, :3]

            distances = np.linalg.norm(points, axis=-1)
            d_min, d_max = distances.min(), distances.max()
            d_normalized = (distances - d_min) / (d_max - d_min)

            colormap = plt.get_cmap("turbo")
            colors = colormap(d_normalized)[:, :3]

            pcd.points = o3d.utility.Vector3dVector(xyz)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()
            vis.reset_view_point(True)
            vis.get_view_control().set_up((0, 0, 1))
            vis.get_view_control().set_front((-1, 0, 0))
            vis.get_view_control().set_zoom(0.005)
            vis.get_view_control().set_lookat((-1, 0, 2))

            image = vis.capture_screen_float_buffer(do_render=True)
            image = np.asarray(image)
            image = (image * 255).astype(np.uint8)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            image = add_timestamp_overlay(image, lidar_data_seq, i, frames)

            video_writer.write(image)

    finally:
        video_writer.release()
        vis.destroy_window()
        print(f"Lidar video saved to {output_path}")


def add_timestamp_overlay(image, lidar_data_seq, frame_idx, total_frames):
    utc_sec = lidar_data_seq['utc_sec'][frame_idx]
    utc_nanosec = lidar_data_seq['utc_nanosec'][frame_idx]

    dt = datetime.utcfromtimestamp(utc_sec)
    timestamp_str = dt.strftime("%Y-%m-%d %H:%M:%S")

    millisec = utc_nanosec // 1000000
    timestamp_with_ms = f"{timestamp_str}.{millisec:03d}"

    points_count = lidar_data_seq['data'][frame_idx].shape[0]

    overlay = image.copy()

    cv2.rectangle(overlay, (10, 10), (650, 100), (0, 0, 0), -1)

    cv2.rectangle(overlay, (image.shape[1] - 250, 10), (image.shape[1] - 10, 80), (0, 0, 0), -1)

    alpha = 0.7
    image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

    cv2.putText(image, f'UTC Time: {timestamp_with_ms}',
                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    cv2.putText(image, f'UTC Sec: {utc_sec} | UTC Nanosec: {utc_nanosec}',
                (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    cv2.putText(image, f'Points: {points_count}',
                (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    cv2.putText(image, f'Frame: {frame_idx + 1}',
                (image.shape[1] - 240, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(image, f'Total: {total_frames}',
                (image.shape[1] - 240, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    return image


def format_utc_timestamp(utc_sec, utc_nanosec, format_type="readable"):
    dt = datetime.utcfromtimestamp(utc_sec)

    if format_type == "readable":
        base_time = dt.strftime("%Y-%m-%d %H:%M:%S")
        millisec = utc_nanosec // 1000000
        return f"{base_time}.{millisec:03d}"

    elif format_type == "iso":
        microsec = utc_nanosec // 1000
        return dt.strftime("%Y-%m-%dT%H:%M:%S") + f".{microsec:06d}Z"

    elif format_type == "compact":
        millisec = utc_nanosec // 1000000
        return dt.strftime("%H:%M:%S") + f".{millisec:03d}"

    else:
        return str(utc_sec)


def convert_lidar_to_pcd_files(lidar_data, output_folder: str):
    data = lidar_data['data']  # shape: (frames, points, 4) - [x, y, z, intensity]
    utc_sec = lidar_data['utc_sec']
    utc_nanosec = lidar_data['utc_nanosec']

    total_frames = len(data)
    print(f"Converting {total_frames} lidar frames to PCD files...")
    reference_ts_sec = 1754558980

    for frame_idx in range(total_frames):
        try:
            frame_points = data[frame_idx]  # shape: (24000, 4)

            if utc_sec[frame_idx] < reference_ts_sec:
                continue

            valid_mask = ~np.all(frame_points[:, :3] == 0, axis=1)
            valid_points = frame_points[valid_mask]

            if len(valid_points) == 0:
                print(f"Frame {frame_idx}: No valid points, skipping...")
                continue

            xyz = valid_points[:, :3]  # x, y, z
            intensity = valid_points[:, 3]  # intensity

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)

            if len(intensity) > 0:
                intensity_normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-8)
                colors = np.stack([intensity_normalized, intensity_normalized, intensity_normalized], axis=1)
                pcd.colors = o3d.utility.Vector3dVector(colors)

            timestamp_sec = utc_sec[frame_idx]
            timestamp_nanosec = utc_nanosec[frame_idx]

            full_timestamp = f"{int(timestamp_sec - reference_ts_sec):04d}_{int(timestamp_nanosec // 1e7):02d}"
            filename = f"{full_timestamp}.pcd"
            filepath = os.path.join(output_folder, filename)

            o3d.io.write_point_cloud(filepath, pcd)

            if frame_idx % 100 == 0:
                print(f"Processed {frame_idx + 1}/{total_frames} frames")

        except Exception as e:
            print(f"Error processing frame {frame_idx}: {str(e)}")
            continue

    print(f"Successfully converted {total_frames} frames to PCD files in {output_folder}")
