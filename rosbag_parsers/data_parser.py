import os
import re
import cv2
import lz4.block
import yaml
import h5py
import struct
import rosbag2_py
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from tqdm import tqdm

from ros_msg_define import *
from collections import namedtuple
from datetime import datetime, timezone
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def timestamp_to_sec(ts: Timestamp):
    if ts.utc_sec == 0 or ts.tick_per_sec == 0 or ts.subtick_per_tick == 0:
        return None, None
    nanosec = int(1e9 / float(ts.tick_per_sec) * (ts.tick + float(ts.subtick) / float(ts.subtick_per_tick)))
    utc_sec = ts.utc_sec + int(nanosec // 1e9)
    nanosec = int(nanosec % 1e9)
    return utc_sec, nanosec

def rosbag_parser(bag_path: str, save_file: str, radar_param: dict):
    SensorFrame = namedtuple('SensorFrame', ['sensor_type', 'utc_sec', 'utc_nanosec', 'index', 'data'])
    sensor_indices = {sensor: 0 for sensor in topic_map.values()}

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    h5f_writer = h5py.File(save_file, 'w')
    resize_step_small = 1000
    resize_step_data = 1

    match = re.search(r'rosbag2_(\d{4})_(\d{2})_(\d{2})-(\d{2})_(\d{2})_(\d{2})', bag_path)
    if match:
        year, month, day = int(match.group(1)), int(match.group(2)), int(match.group(3))
    else:
        raise ValueError(f"no utc time in the filename: {bag_path}")

    radar_frame_buffer = {}
    radar_buffered_bytes = 0
    byte_per_frame = radar_param['tx'] * radar_param['chirps'] * radar_param['rx'] * radar_param['samples'] * radar_param['IQ'] * radar_param['bytes']

    utc_sec = None
    nanosec = None
    camera_actual_cnt =0
    radar_first_valid_packet_seq = None
    radar_frame_idx = 0

    metadata_file = os.path.join(bag_path, 'metadata.yaml')
    with open(metadata_file, 'r') as f:
        metadata = yaml.safe_load(f)
    total_messages = 0
    for topic_info in metadata['rosbag2_bagfile_information']['topics_with_message_count']:
        total_messages += topic_info['message_count']

    with tqdm(total=total_messages, desc="Reading rosbag2 messages") as pbar:
        while reader.has_next():
            try:
                topic, data, ros_t = reader.read_next()
                pbar.update(1)
                if topic not in topic_map:
                    continue

                sensor_data = None
                sensor_type = topic_map[topic]
                msg = deserialize_message(data, get_message(type_map[topic]))

                if sensor_type == 'rtk':
                    utc_datetime = datetime(year, month, day, msg.utc_hour, msg.utc_min, int(msg.utc_sec), tzinfo=timezone.utc)
                    utc_sec, nanosec = utc_datetime.timestamp(), int(round((msg.utc_sec - int(msg.utc_sec)) * 1e9))
                    sensor_data = np.array([msg.lat, msg.lon, msg.alt, msg.heading, msg.bs_id, msg.state, msg.rtcm_age])
                elif sensor_type == 'adis':
                    utc_sec, nanosec = timestamp_to_sec(Timestamp(msg.ts.pwr_sec, msg.ts.utc_sec, msg.ts.tick, msg.ts.tick_per_sec, msg.ts.subtick, msg.ts.subtick_per_tick, msg.ts.clk, msg.ts.clk_per_sec))
                    sensor_data = np.array([msg.gyr.x, msg.gyr.y, msg.gyr.z, msg.acc.x, msg.acc.y, msg.acc.z])
                elif sensor_type == 'mag':
                    utc_sec, nanosec = timestamp_to_sec(Timestamp(msg.ts.pwr_sec, msg.ts.utc_sec, msg.ts.tick, msg.ts.tick_per_sec, msg.ts.subtick, msg.ts.subtick_per_tick, msg.ts.clk, msg.ts.clk_per_sec))
                    sensor_data = np.array([msg.mag.x, msg.mag.y, msg.mag.z])
                elif sensor_type in ('lps28', 'lps22'):
                    utc_sec, nanosec = timestamp_to_sec(Timestamp(msg.ts.pwr_sec, msg.ts.utc_sec, msg.ts.tick, msg.ts.tick_per_sec, msg.ts.subtick, msg.ts.subtick_per_tick, msg.ts.clk, msg.ts.clk_per_sec))
                    sensor_data = np.array([msg.pres, msg.temp, msg.alt])
                elif sensor_type == 'camera' or sensor_type == 'camera2':
                    utc_sec, nanosec = timestamp_to_sec(Timestamp(msg.ts.pwr_sec, msg.ts.utc_sec, msg.ts.tick, msg.ts.tick_per_sec, msg.ts.subtick, msg.ts.subtick_per_tick, msg.ts.clk, msg.ts.clk_per_sec))
                    image = np.frombuffer(lz4.block.decompress(msg.data, msg.height * msg.width), dtype=np.uint8).reshape(msg.height, msg.width)
                    sensor_data = cv2.cvtColor(image, cv2.COLOR_BayerRGGB2RGB)[::-1, ::-1]
                elif sensor_type == 'lidar':
                    utc_sec, nanosec = msg.header.stamp.sec, msg.header.stamp.nanosec
                    points = pc2.read_points(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True)
                    sensor_data = np.array([[p[0], p[1], p[2], p[3]] for p in points], dtype=np.float32)
                elif sensor_type == 'radar':
                    utc_sec, nanosec = timestamp_to_sec(Timestamp(msg.ts.pwr_sec, msg.ts.utc_sec, msg.ts.tick, msg.ts.tick_per_sec, msg.ts.subtick, msg.ts.subtick_per_tick, msg.ts.clk, msg.ts.clk_per_sec))

                    if len(msg.data) % BYTES_IN_PACKET != 0:
                        print(f"Warning: msg.data length {len(msg.data)} not divisible by packet size {BYTES_IN_PACKET}")

                    for i in range(len(msg.data) // BYTES_IN_PACKET):
                        raw_packet = msg.data[i * BYTES_IN_PACKET: (i + 1) * BYTES_IN_PACKET]

                        pkt_seq = struct.unpack('<1L', raw_packet[:4])[0]
                        pkt_data = raw_packet[10:]

                        if radar_first_valid_packet_seq is None:
                            total_bytes = pkt_seq * DATA_IN_PACKET
                            invalid_bytes = (total_bytes // byte_per_frame) * byte_per_frame
                            left_bytes = total_bytes - invalid_bytes
                            if 0 < left_bytes <= DATA_IN_PACKET:
                                radar_first_valid_packet_seq = pkt_seq - 1
                                pkt_data = pkt_data[-left_bytes:]
                            else:
                                continue

                        adjusted_pkt_seq = pkt_seq - radar_first_valid_packet_seq

                        if radar_frame_idx not in radar_frame_buffer:
                            radar_frame_buffer[radar_frame_idx] = {
                                'utc_sec': utc_sec,
                                'utc_nanosec': nanosec,
                                'packets': [],
                                'received_pkt_indices': []
                            }

                        radar_frame_buffer[radar_frame_idx]['packets'].append(pkt_data)
                        radar_frame_buffer[radar_frame_idx]['received_pkt_indices'].append(adjusted_pkt_seq)
                        radar_buffered_bytes += len(pkt_data)

                        if radar_buffered_bytes >= byte_per_frame:
                            frame_bytes = b''.join(radar_frame_buffer[radar_frame_idx]['packets'])
                            radar_frame_idx += 1

                            if len(frame_bytes) > byte_per_frame:
                                current_frame_bytes = frame_bytes[:byte_per_frame]
                                extra_bytes = frame_bytes[byte_per_frame:]

                                radar_frame_buffer[radar_frame_idx] = {
                                    'utc_sec': utc_sec,
                                    'utc_nanosec': nanosec,
                                    'packets': [],
                                    'received_pkt_indices': []
                                }
                                radar_buffered_bytes = len(extra_bytes)
                                radar_frame_buffer[radar_frame_idx]['packets'].append(extra_bytes)
                                radar_frame_buffer[radar_frame_idx]['received_pkt_indices'].append(adjusted_pkt_seq)
                            else:
                                current_frame_bytes = frame_bytes
                                radar_buffered_bytes = 0

                            data_int16 = np.frombuffer(current_frame_bytes, dtype=np.uint16)

                            if data_int16.size != byte_per_frame // 2:
                                print(f"Incomplete frame detected at frame {sensor_indices[sensor_type]}: {data_int16.size} != {byte_per_frame // 2}")
                                del radar_frame_buffer[radar_frame_idx - 1]
                                continue

                            adc_data_raw = np.reshape(data_int16,
                                                      (-1, radar_param['chirps'] * radar_param['tx'],
                                                       radar_param['samples'], radar_param['IQ'], radar_param['rx']))

                            adc_data = np.zeros((adc_data_raw.shape[0], radar_param['tx'], radar_param['chirps'],
                                                 radar_param['samples'], radar_param['IQ'], radar_param['rx']),
                                                dtype=adc_data_raw.dtype)
                            for tx_idx in range(radar_param['tx']):
                                adc_data[:, tx_idx, :, :, :, :] = adc_data_raw[:, tx_idx::radar_param['tx'], :, :, :]

                            if radar_param['IQ'] == 2:  # Complex
                                adc_data = (adc_data[..., 0, :] + 1j * adc_data[..., 1, :]).astype(np.complex64)
                            else:  # Real
                                adc_data = adc_data[..., 0, :].astype(np.float32)

                            # (tx, rx, frame, samples, chirp)
                            sensor_data = np.transpose(adc_data, (1, 4, 0, 3, 2))
                            del radar_frame_buffer[radar_frame_idx - 1]

                if utc_sec is None:
                    utc_sec = ros_t // 1e9
                    nanosec = ros_t % 1e9

                if sensor_data is None:
                    continue

                frame = SensorFrame(sensor_type, utc_sec, nanosec, sensor_indices[sensor_type], sensor_data)
                sensor_group = h5f_writer.require_group(sensor_type)

                for field in ['utc_sec', 'utc_nanosec', 'index', 'data']:
                    value = getattr(frame, field)
                    value = np.asarray(value)
                    if field not in sensor_group:
                        maxshape = (None,) + value.shape if field == 'data' else (None,)

                        if field == 'data':
                            initial_shape = (resize_step_data,) + value.shape
                            chunks = (resize_step_data,) + value.shape
                        else:
                            initial_shape = (resize_step_small,)
                            chunks = (resize_step_small,)

                        sensor_group.create_dataset(
                            field, shape=initial_shape, maxshape=maxshape,
                            dtype=value.dtype, chunks=chunks, compression='gzip'
                        )

                    ds = sensor_group[field]
                    current_idx = sensor_indices[sensor_type]
                    step = resize_step_data if field == 'data' else resize_step_small
                    if current_idx >= ds.shape[0]:
                        ds.resize(ds.shape[0] + step, axis=0)
                    ds[current_idx] = value

                sensor_indices[sensor_type] += 1
            except Exception as e:
                h5f_writer.close()
                pbar.close()
                raise RuntimeError(f"read bag error: {e}")

    # Resize the datasets to the actual size
    for sensor_type, sensor_group in h5f_writer.items():
        actual_size = sensor_indices[sensor_type]
        for field, ds in sensor_group.items():
            ds.resize(actual_size, axis=0)

    h5f_writer.close()
    print(f"Data saved to {save_file}")
