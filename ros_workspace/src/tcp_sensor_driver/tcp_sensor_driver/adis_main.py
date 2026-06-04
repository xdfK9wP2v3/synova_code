import rclpy
from .utils.tcp_config import tcpIP, tcpPorts, topicName, capInCh, pkgCycle
from .imu_base_node import TcpBaseNode

class ADISNode(TcpBaseNode):
    def __init__(self):
        super().__init__(
            node_name   = "adis_node",
            sensor_name = "ADIS",
            ip          = tcpIP.ADIS.value,
            port        = tcpPorts.ADIS.value,
            topic_name  = topicName.ADIS.value,
            capin_ch_idx= capInCh.ADIS.value,
            data_fps    = 2000,
            pkg_cycle   = pkgCycle.ADIS.value
        )

def main(args=None):
    rclpy.init(args=args)

    node = ADISNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()