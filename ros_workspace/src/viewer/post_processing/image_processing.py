#!/usr/bin/env python3
import os
import sys
import numpy as np
import cv2
import lz4.block

# ROS2 bag API
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy
from rclpy.serialization import deserialize_message
from my_msgs.msg import RawImg


def process_image(img: np.ndarray) -> np.ndarray:
    # TODO: your code here
    return img


def main():
    rclpy.init(args=[])

    # Open bag
    reader = SequentialReader()
    bag_path = "/home/ego/Documents/ROS2_docker/ros_workspace/rosbag2_2025_05_30-10_01_42/rosbag2_2025_05_30-10_01_42_0.mcap"
    storage_opts = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_opts, converter_opts)

    # get all topics and types
    topics = reader.get_all_topics_and_types()
    topic_type_map = {t.name: t.type for t in topics}
    if '/cam/raw_img' not in topic_type_map.keys():
        print("Error: can't find topic '/cam/raw_img' in bag.")
        sys.exit(2)
    print("Found '/cam/raw_img' of type:", topic_type_map['/cam/raw_img'])

    # Read data
    accum_idx = 0
    accum_N = 10
    accum_img = np.zeros((1100, 1600, 3), dtype=np.float32)

    img_idx = 0
    while reader.has_next():
        topic, serialized_msg, t = reader.read_next()
        if topic != '/cam/raw_img':
            continue

        # deserialize messages
        img_msg = deserialize_message(serialized_msg, RawImg)

        image = np.frombuffer(lz4.block.decompress(img_msg.data, 1100 * 1600), dtype=np.uint8).reshape(1100, 1600)
        image = cv2.cvtColor(image, cv2.COLOR_BayerRGGB2RGB)
        image = image[::-1, ::-1]

        accum_idx += 1
        accum_img += image
        if accum_idx >= accum_N:
            accum_img /= accum_N
            accum_img = accum_img.astype(np.uint8)
            cv2.imwrite(f"{os.environ['HOME']}/Documents/data/camera/img_{img_idx:03d}.png", cv2.cvtColor(accum_img, cv2.COLOR_RGB2BGR))

            img_idx += 1
            accum_idx = 0
            accum_img = np.zeros((1100, 1600, 3), dtype=np.float32)
        # processed = process_image(image)
        #
        # cv2.imshow("proc", processed)
        # key = cv2.waitKey(1)
        # if key == 27:  # ESC
        #     break

    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
