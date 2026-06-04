import time

from .utils.tcp_connection import TCPConnect
from rclpy.node import Node
from .utils.tcp_config import tcpIP, tcpPorts


class SyncTimeCapConfigNode(Node):
    def __init__(self):
        self.sensor_name = "time_cap_config_node"
        super().__init__(self.sensor_name)
        self.ip = tcpIP.SYNC.value
        self.port = tcpPorts.SYNC.value

        for i in range(1, 13):
            self.declare_parameter(f'activate_ch{i}', False)
            self.declare_parameter(f'pre_divider_{i}', 0)
            self.declare_parameter(f'shift_{i}', 0)
            self.declare_parameter(f'mode_{i}', 'N')

        self.tcp_cont = TCPConnect(sensor_name=self.sensor_name, ip=self.ip, port=self.port, logger=self.get_logger(), recv_callback=self._recv_callback)
        self.ready_for_config: bool = True
        self.configure_time_cap()

    def configure_time_cap(self):
        for i in range(12):
            activate_ch = self.get_parameter(f"activate_ch{i}").value
            if activate_ch:
                pre_divider = self.get_parameter(f"pre_divider_{i}").value
                shift = self.get_parameter(f'shift_{i}').value
                mode = self.get_parameter(f'mode_{i}').value

                cmd_payload = f"CONFIG,TIMEC,{i},{pre_divider},{shift},{mode}"
                cmd_full = self.tcp_cont.construct_msg(cmd_payload)

                self.tcp_cont.send(cmd_full)
                self.ready_for_config = False
                self.get_logger().info(f"Sent TIMEC command: {cmd_full.strip()}")

                time_start = time.time()
                while not self.ready_for_config:
                    if time.time() - time_start > 10:
                        raise RuntimeError("Timeout for receiving RSPON message.")
                    time.sleep(0.1)


    def _recv_callback(self, pkg: str):
        pkg_split = pkg.split(',')
        pkg_type = pkg_split[0]

        if pkg_type == 'RSPON':
            config_state = pkg_split[10]
            cmd_type = pkg_split[11]

            self.get_logger().info(f"Config cmmand state: {config_state} - {cmd_type}")

            if config_state == 'OK':
                self.ready_for_config = True
