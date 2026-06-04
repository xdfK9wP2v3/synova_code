import os
import sys
import time
from enum import Enum
from typing import Type

import rclpy
from numpy.f2py.auxfuncs import throw_error
from rclpy.duration import Duration
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32
from my_msgs.msg import TimeStamp, State
from .utils.tcp_connection import TCPConnect
from .utils.tcp_config import TICKS_PER_SEC


# TODO: handle sync time retro pkgs!

class syncMsgType(Enum):
    SECON = 'SECON'
    CAPIN = 'CAPIN'
    TROUT = 'TROUT'
    PULSE = 'PULSE'
    UTCIN = 'UTCIN'
    RTCUP = 'RTCUP'


class SyncBaseNode(Node):
    def __init__(self,
                 node_name: str,
                 ip: str,
                 port: int,
                 pkg_cycle: int):
        super().__init__(node_name)
        self.sensor_name = "SYNC"
        self.topic_name = "/sync_data"

        self.pwr_ts_start: int | None = None
        self.PKG_CYCLE: int = pkg_cycle
        self.pkg_idx: int | None = None
        self.pkg_loss: int = 0

        # create publisher
        self.sync_pkg_types = ["CAPIN", "TROUT", "SECON", "PULSE", "UTCIN", "RTCUP"]
        qos_profile = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=1)
        )
        self.capin_msg_pub_map = {f'{i}' : self.create_publisher(TimeStamp, self.topic_name + f'/capin/C{i}', qos_profile) for i in range(1, 13)}
        self.trout_msg_pub_map = {f'{i}' : self.create_publisher(TimeStamp, self.topic_name + f'/trout/C{i}', qos_profile) for i in range(1, 13)}

        self.raw_str_pub_map = {pkg_type: self.create_publisher(String, self.topic_name + f'/{pkg_type.lower()}/raw', qos_profile) for pkg_type in self.sync_pkg_types}
        self.state_msg_publisher = self.create_publisher(State, '/state/sync/state', qos_profile)

        # create TCP connection
        self.tcp_conk = TCPConnect(sensor_name=self.sensor_name, ip=ip, port=port, logger=self.get_logger(), recv_callback=self._recv_callback)

    def _recv_callback(self, pkg: str):
        pkg_split = pkg.split(',')
        pkg_type = pkg_split[0]

        if self.pwr_ts_start is None:
            self.pwr_ts_start = int(pkg_split[6])
            assert self.pwr_ts_start > 0

        # Check package loss
        pkg_sn = int(pkg_split[1])
        if self.pkg_idx is not None:
            self.pkg_loss += ((pkg_sn + self.PKG_CYCLE) - (self.pkg_idx + 1) % self.PKG_CYCLE) % self.PKG_CYCLE
            if self.pkg_loss > 0:
                raise RuntimeError("Message loss exists!")
        self.pkg_idx = pkg_sn

        match pkg_type:
            case k if k in self.sync_pkg_types:
                if pkg_type == 'TROUT':
                    self._publish_trout_msg(pkg_split)
                elif pkg_type == 'CAPIN':
                    self._publish_capin_msg(pkg_split)

                raw_msg = String()
                raw_msg.data = pkg
                self.raw_str_pub_map[pkg_type].publish(raw_msg)

            case 'ERROR':
                match pkg.split(','):
                    # ERROR,0014,B0000140,,,,1365,00000,00000/20000,000000000,20000000
                    case 'ERROR', msg_sn, system_state, _, _, _, _, _, _, _, '20000000':
                        pass
                    case _:
                        self.get_logger().error(f"Sensor {self.sensor_name} error: {pkg}")
            case _:
                self.get_logger().warn(f"Unhandled message type: {pkg_type}")

    @staticmethod
    def _parse_common_data(pkg_split: list) -> dict:
        common_data = {
            'type': pkg_split[0],
            'msg_sn': int(pkg_split[1]),
            'system_state': pkg_split[2],
            'utc_year': int(val.split('/')[0]) if (val := pkg_split[3]) else 0,
            'utc_mon':  int(val.split('/')[1]) if (val := pkg_split[3]) else 0,
            'utc_day':  int(val.split('/')[2]) if (val := pkg_split[3]) else 0,
            'utc_hour': int(val.split(':')[0]) if (val := pkg_split[4]) else 0,
            'utc_min':  int(val.split(':')[1]) if (val := pkg_split[4]) else 0,
            'utc_sec':  int(val.split(':')[2]) if (val := pkg_split[4]) else 0,
            'utc_ts': int(val) if (val := pkg_split[5]) else 0,
            'pwr_ts': int(pkg_split[6]),
            'tick': int(pkg_split[7]),
            'subtick': int(pkg_split[8].split('/')[0]),
            'subtick_per_tick': int(pkg_split[8].split('/')[1]),
            'clk': int(pkg_split[9]),
        }

        return common_data

    def _get_ts_msg(self, pkg_split: list[str]) -> tuple[TimeStamp, State]:
        common_data = self._parse_common_data(pkg_split)

        assert common_data['pwr_ts'] - self.pwr_ts_start >= 0
        sync_msg = TimeStamp()
        sync_msg.pwr_sec          = int(common_data['pwr_ts'] - self.pwr_ts_start)
        sync_msg.utc_sec          = common_data['utc_ts']
        sync_msg.tick             = common_data['tick']
        sync_msg.tick_per_sec     = int(TICKS_PER_SEC)
        sync_msg.subtick          = common_data['subtick']
        sync_msg.subtick_per_tick = int(common_data['subtick_per_tick'])
        sync_msg.clk              = common_data['clk']
        sync_msg.clk_per_sec      = 0      # TODO: Set as 0 for now

        state_msg = State()
        state_msg.ts = sync_msg
        state_msg.state = common_data['system_state']

        return sync_msg, state_msg

    def _publish_capin_msg(self, pkg_split: list[str]) -> None:
        """
        Example CAPIN pkg:
            $CAPIN,0009,E0001FFF,2024/11/21,09:33:21,1732181601,519,11993,07228/20000,239869610,2,F*17
            $CAPIN, [COMMON PART], in_channel_idx (10), edge_type (11)
        """
        # check edge type
        edge_type = pkg_split[11]
        in_channel_idx = int(pkg_split[10])
        if edge_type in ['FX', 'RX']:
            self.get_logger().warn(f"CAPIN C{in_channel_idx} has {edge_type} edge type.")

        if edge_type in ['R', 'RX']:
            # publish TimeStamp msg
            sync_msg, state_msg = self._get_ts_msg(pkg_split)
            self.capin_msg_pub_map[str(in_channel_idx)].publish(sync_msg)
            self.state_msg_publisher.publish(state_msg)

    def _publish_trout_msg(self, pkg_split: list[str]) -> None:
        """
        Example TROUT pkg:
             # $TROUT,0010,E0001FFF,2024/11/21,09:33:21,1732181601,519,11994,00000/20000,239882382,4,R,0*0F
             # $TROUT, [COMMON PART], out_channel_idx (10), edge_type (11), rf_compensation (12)
        """
        # check edge type
        edge_type = pkg_split[11]
        out_channel_idx = int(pkg_split[10])
        if edge_type in ['FX', 'RX']:
            self.get_logger().warn(f"TROUT C{out_channel_idx} has {edge_type} edge type.")

        if edge_type in ['R', 'RX']:
            # publish TimeStamp msg
            sync_msg, state_msg = self._get_ts_msg(pkg_split)
            sync_msg.tick += 1  # Attention here!
            self.trout_msg_pub_map[str(out_channel_idx)].publish(sync_msg)
            self.state_msg_publisher.publish(state_msg)

    # def _parse_secon_msg(self, message: list[str], common_data: dict):
    #     # $SECON,0012,E0001FFF,2024/11/21,09:33:22,1732181602,520,0000,00000/20000,240002382,519*4A
    #     # $SECON,...,prev_device_ts=message[10]
    #     prev_device_ts= int(message[10])
    #     return prev_device_ts

    # def _compute_ts(self, device_ts: int, tick: int, subtick: int, subtick_per_tick: int) -> float:
    #     # compute ns-level timestamp from tick info
    #     ts = (device_ts - self.pwr_ts_start) * 1e9  # align with device_ts
    #     ts += float(tick) * 1e9 / ticks_per_sec
    #     ts += float(subtick) * 1e9 / (subtick_per_tick * ticks_per_sec)
    #     return ts

    def destroy_node(self):
        self.get_logger().info(f"Shutting down {self.sensor_name} node.")
        self.tcp_conk.is_running = False
        if self.tcp_conk.recv_thread.is_alive():
            self.tcp_conk.recv_thread.join(timeout=1.0)
        if self.tcp_conk.socket_:
            self.tcp_conk.socket_.close()
        super().destroy_node()
