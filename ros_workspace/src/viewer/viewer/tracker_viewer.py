import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

from my_msgs.msg import TrackerState
from tcp_sensor_driver.utils.device_time import SyncDeviceTime

class TrackerTFBroadcaster(Node):
    def __init__(self):
        super().__init__('ukf_tf_broadcaster')
        
        self.br = TransformBroadcaster(self)
        self.ukf_subscriber = self.create_subscription(TrackerState, '/tracker', self._ukf_callback, 10)

    def _ukf_callback(self, msg: TrackerState):
        tf = TransformStamped()

        timestamp = SyncDeviceTime.ts_msg_to_ts_nanosec_int(msg.ts)
        tf.header.stamp.sec = int(timestamp // 1e9)
        tf.header.stamp.nanosec = int(timestamp % 1e9)
        tf.header.frame_id = 'world'     # TODO: how to set
        tf.child_frame_id = msg.frame_id

        tf.transform.translation.x = msg.pos.x
        tf.transform.translation.y = msg.pos.y
        tf.transform.translation.z = msg.pos.z

        tf.transform.rotation.x = msg.qua.x
        tf.transform.rotation.y = msg.qua.y
        tf.transform.rotation.z = msg.qua.z
        tf.transform.rotation.w = msg.qua.w

        self.br.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = TrackerTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()