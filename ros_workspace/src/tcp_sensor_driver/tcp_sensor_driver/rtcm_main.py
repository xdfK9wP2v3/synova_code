import time

import rclpy
import socket
import threading
from rclpy.node import Node

from .utils.tcp_config import tcpIP, tcpPorts


class RtcmNode(Node):
    def __init__(self):
        super().__init__("rtcm_node")
        self.running = True

        self.s_bs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s_rover = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s_bs.settimeout(5)
        self.s_rover.settimeout(5)

        try:
            self.s_rover.connect((tcpIP.RTK_NMEA.value, tcpPorts.RTK_RTCM.value))
            self.get_logger().info(f"Connected to {tcpIP.RTK_NMEA.value}:{tcpPorts.RTK_RTCM.value} (Rover)")
        except Exception as e:
            self.get_logger().error(f"Error connecting to Rover: {e}")
            self.s_rover.close()
            self.s_bs.close()
            raise e

        time.sleep(2)  # wait vpn start
        try:
            self.s_bs.connect((tcpIP.RTK_RTCM.value, tcpPorts.RTK_RTCM.value))
            self.get_logger().info(f"Connected to {tcpIP.RTK_RTCM.value}:{tcpPorts.RTK_RTCM.value} (Base Station)")
        except Exception as e:
            self.get_logger().error(f"Error connecting to Base Station: {e}")
            self.s_rover.close()
            self.s_bs.close()
            raise e

        self.thread = threading.Thread(target=self._forward_data)
        self.thread.daemon = True
        self.thread.start()

    def _forward_data(self):
        while self.running:
            try:
                data = self.s_bs.recv(8)
                self.s_rover.sendall(data)
            except TimeoutError:
                self.get_logger().warning("No RTCM received from Base Station, waiting...")
                time.sleep(1)
                continue
        self.get_logger().info("Data forwarding thread terminated.")

    def destroy_node(self):
        self.get_logger().info("Shutting down RtcmNode, closing socket connections.")
        self.running = False

        try:
            self.s_bs.shutdown(socket.SHUT_RDWR)
        except Exception as e:
            self.get_logger().debug(f"Exception during s_bs shutdown: {e}")
        try:
            self.s_rover.shutdown(socket.SHUT_RDWR)
        except Exception as e:
            self.get_logger().debug(f"Exception during s_rover shutdown: {e}")

        self.s_bs.close()
        self.s_rover.close()

        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtcmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
