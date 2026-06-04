import rclpy
from .tracker_node import TrackerNode

class UKFNode(TrackerNode):
    def __init__(self):
        super().__init__()

def main(args=None):
    rclpy.init(args=args)

    node = UKFNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
