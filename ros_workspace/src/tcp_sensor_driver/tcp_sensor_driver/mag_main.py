import rclpy
from .utils.tcp_config import tcpIP, tcpPorts, topicName, capInCh, pkgCycle
from .imu_base_node import TcpBaseNode

class MAGNode(TcpBaseNode):
    def __init__(self):
        super().__init__(
            node_name   = "mag_node",
            sensor_name = "MAG",
            ip          = tcpIP.MAG.value,
            port        = tcpPorts.MAG.value,
            topic_name  = topicName.MAG.value,
            capin_ch_idx= capInCh.MAG.value,
            data_fps    = 80,
            pkg_cycle   = pkgCycle.MAG.value
        )

def main(args=None):
    rclpy.init(args=args)

    node = MAGNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()