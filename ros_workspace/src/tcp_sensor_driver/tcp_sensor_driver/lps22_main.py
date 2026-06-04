import rclpy
from .utils.tcp_config import tcpIP, tcpPorts, topicName, capInCh, pkgCycle
from .imu_base_node import TcpBaseNode

class LPS22Node(TcpBaseNode):
    def __init__(self):
        super().__init__(
            node_name   = "lps22_node",
            sensor_name = "LPS22",
            ip          = tcpIP.LPS22.value,
            port        = tcpPorts.LPS22.value,
            topic_name  = topicName.LPS22.value,
            capin_ch_idx= capInCh.LPS22.value,
            data_fps    = 10,
            pkg_cycle   = pkgCycle.LPS22.value
        )

def main(args=None):
    rclpy.init(args=args)

    node = LPS22Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()