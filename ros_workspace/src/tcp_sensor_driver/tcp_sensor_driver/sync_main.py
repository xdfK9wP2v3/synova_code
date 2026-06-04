import rclpy
from .sync_base_node import SyncBaseNode
from .utils.tcp_config import tcpIP, tcpPorts, pkgCycle

class SyncNode(SyncBaseNode):
    def __init__(self):
        super().__init__(
            node_name   = "sync_node",
            ip          = tcpIP.SYNC.value,
            port        = tcpPorts.SYNC.value,
            pkg_cycle   = pkgCycle.SYNC.value
        )

def main(args=None):
    rclpy.init(args=args)

    # TODO: Ensure sync node to be executed first and shut down at last
    node = SyncNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
