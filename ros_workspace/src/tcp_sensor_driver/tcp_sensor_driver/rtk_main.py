import rclpy
from .utils.tcp_config import tcpIP, tcpPorts, topicName, pkgCycle
from .imu_base_node import TcpBaseNode

class RTKNode(TcpBaseNode):
    def __init__(self):
        super().__init__(
            node_name   = "rtk_node",
            sensor_name = "RTK",
            ip          = tcpIP.RTK_NMEA.value,
            port        = tcpPorts.RTK_NMEA.value,
            topic_name  = topicName.RTK.value,
            capin_ch_idx= None,
            data_fps    = None,
            pkg_cycle   = pkgCycle.RTK.value
        )

def main(args=None):
    rclpy.init(args=args)

    node = RTKNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()