import rclpy
from .utils.tcp_config import tcpIP, tcpPorts, topicName, capInCh, pkgCycle
from .imu_base_node import TcpBaseNode

class LPS28Node(TcpBaseNode):
    def __init__(self):
        super().__init__(
            node_name   = "lps28_node",
            sensor_name = "LPS28",
            ip          = tcpIP.LPS28.value,
            port        = tcpPorts.LPS28.value,
            topic_name  = topicName.LPS28.value,
            capin_ch_idx= capInCh.LPS28.value,
            data_fps    = 10,
            pkg_cycle   = pkgCycle.LPS28.value
        )

def main(args=None):
    rclpy.init(args=args)

    node = LPS28Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()