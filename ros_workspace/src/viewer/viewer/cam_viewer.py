import cv2
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from my_msgs.msg import RawImg
import numpy as np
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterNotDeclaredException, ParameterUninitializedException
from rcl_interfaces.msg import SetParametersResult

from tcp_sensor_driver.utils.device_time import SyncDeviceTime
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor

import lz4.block

class CamSubscriber(Node):
    def __init__(self, cam_name: str = 'cam', phase_shift: int = 0):
        super().__init__(f'viewer_{cam_name}')

        self.pub_fps: float = 0.5
        self.frame_gap = 1 / self.pub_fps
        self.time_offset = phase_shift * self.frame_gap / 2
        self.last_pub_cycle = -1

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.raw_img_sub = self.create_subscription(RawImg, f"/{cam_name}/raw_img", self.recv_data_callback, qos_profile)
        self.img_pub = self.create_publisher(Image, f"/{cam_name}/image", qos_profile)

        self.get_logger().info('Subscriber is ready and listening.')

    def recv_data_callback(self, raw_msg: RawImg) -> None:
        current_ts = self.get_clock().now().to_msg()
        wall_ts = current_ts.sec + current_ts.nanosec * 1e-9

        adjusted_time = wall_ts + self.time_offset
        current_cycle = int(adjusted_time / self.frame_gap)

        if current_cycle > self.last_pub_cycle:
            self.last_pub_cycle = current_cycle
            self.get_logger().info(f"Publishing image at {wall_ts:.3f} seconds")

            img_msg = Image()

            sync_ts = SyncDeviceTime()
            sync_ts.load_from_ts_msg(raw_msg.ts)
            img_msg.header.stamp.sec = sync_ts.sec
            img_msg.header.stamp.nanosec = sync_ts.nanosec

            img_msg.width = raw_msg.width
            img_msg.height = raw_msg.height
            img_msg.encoding = "rgb8"
            img_msg.step = raw_msg.width * 3

            image = np.frombuffer(lz4.block.decompress(raw_msg.data, 1100 * 1600), dtype=np.uint8).reshape(1100, 1600)
            image = cv2.cvtColor(image, cv2.COLOR_BayerRGGB2RGB)
            img_msg.data = image[::-1, ::-1].tobytes()

            self.img_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CamSubscriber('cam', 0)
    node_cam2 = CamSubscriber('cam2', 1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node_cam2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down CamSubscriber.')
    finally:
        executor.shutdown()
        node.destroy_node()
        node_cam2.destroy_node()
        rclpy.shutdown()