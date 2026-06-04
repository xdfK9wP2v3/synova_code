import os
from datetime import datetime

from radar_config_parser import read_radar_config
from data_parser import rosbag_parser

def find_closest_bag(target_path, target_side):
    path_parts = target_path.split('/')
    base_path = '/'.join(path_parts[:-2])
    target_timestamp_str = target_path.split('rosbag2_')[-1]

    opposite_side = 'ego' if target_side == 'rsu' else 'rsu'
    opposite_path = os.path.join(base_path, opposite_side)

    if not os.path.exists(opposite_path):
        print(f"Warning: Opposite side path does not exist: {opposite_path}")
        return None

    try:
        target_time = datetime.strptime(target_timestamp_str, "%Y_%m_%d-%H_%M_%S")
    except ValueError as e:
        print(f"Error parsing target timestamp: {e}")
        return None

    rosbag_folders = []
    for item in os.listdir(opposite_path):
        if item.startswith('rosbag2_') and os.path.isdir(os.path.join(opposite_path, item)):
            try:
                timestamp_str = item.split('rosbag2_')[-1]
                timestamp = datetime.strptime(timestamp_str, "%Y_%m_%d-%H_%M_%S")
                rosbag_folders.append((item, timestamp))
            except ValueError:
                continue

    if not rosbag_folders:
        print(f"No valid rosbag2 folders found in {opposite_path}")
        return None

    closest_bag = min(rosbag_folders, key=lambda x: abs((x[1] - target_time).total_seconds()))
    closest_bag_path = os.path.join(opposite_path, closest_bag[0])

    print(f"Found closest bag: {closest_bag_path}")
    print(f"Time difference: {abs((closest_bag[1] - target_time).total_seconds())} seconds")

    return closest_bag_path


def process_bag_pair(bag_path, radar_params, save_folder):
    bag_side = bag_path.split('/')[-2]
    bag_id = bag_path.split('rosbag2_')[-1]

    if bag_side not in ['ego', 'rsu', 'ros_workspace']:
        print("Error: The provided bag path does not belong to a valid side (ego, rsu, or ros_workspace).")
        return None, None

    save_folder = os.path.join(save_folder, bag_id)
    os.makedirs(os.path.join(f"{save_folder}", 'ego'), exist_ok=True)
    os.makedirs(os.path.join(f"{save_folder}", 'rsu'), exist_ok=True)

    # only for the testing case
    print(f'Parsing {bag_side} rosbag: {bag_path}')
    data_path = f"{save_folder}/{os.environ['USER'] if bag_side == 'ros_workspace' else bag_side}/data.h5"
    rosbag_parser(bag_path, data_path, radar_params)

    if bag_side == 'ros_workspace':
        print("No corresponding bag for ros_workspace, returning only the original bag path.")
        return f"{save_folder}/{bag_side}/data.h5", None

    corresponding_bag_path = find_closest_bag(bag_path, bag_side)

    if corresponding_bag_path:
        corresponding_bag_side = corresponding_bag_path.split('/')[-2]
        print(f'Parsing corresponding {corresponding_bag_side} rosbag: {corresponding_bag_path}')
        corresponding_data_path = f"{save_folder}/{corresponding_bag_side}/data.h5"
        rosbag_parser(corresponding_bag_path, corresponding_data_path, radar_params)

        return data_path, corresponding_data_path
    else:
        print("No corresponding bag found")
        return data_path, None


if __name__ == "__main__":
    path = "0807/ego/rosbag2_2025_08_07-10_04_31"

    adc_params, _, _ = read_radar_config(radar_config_path = f"{os.environ['HOME']}/Documents/ROS2_docker/ros_workspace/src/radar_driver_cpp/src/configFiles/AWR2243_mmwaveconfig_max15.txt",
                           dca_config_path = f"{os.environ['HOME']}/Documents/ROS2_docker/ros_workspace/src/radar_driver_cpp/src/configFiles/dca_config.txt",
                           display_config = False)

    folder = f"."
    original_folder, corresponding_folder = process_bag_pair(path, adc_params, folder)
    print(f"Original bag processed to: {original_folder}")
    if corresponding_folder:
        print(f"Corresponding bag processed to: {corresponding_folder}")
