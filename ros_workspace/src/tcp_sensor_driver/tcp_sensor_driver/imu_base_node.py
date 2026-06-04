from rclpy.duration import Duration
from rclpy.node import Node
import traceback
from std_msgs.msg import String, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import re
import socket
import threading
import time

from .utils.tcp_config import topicName, trOutCh, FrameType
from .utils.time_align import TimeAlign
from .utils.device_time import SyncDeviceTime, SensorDeviceTime
from .utils.sensor_pkg_handler import SensorPkgHandler
from my_msgs.msg import State, TimeStamp, IMU, Mag, Baro, Rtk
from .utils.tcp_connection import TCPConnect

# a: sync, b: sensor
# sensor: pkgs - (dev_time, pkg)       - data
# sync:   pkgs - (dev_time, None)      - capin
# sensor: tms - (wall_time, dev_time)  - tms
# sync:   tms - (wall_time, dev_time)  - trout

class TcpBaseNode(Node):
    def __init__(self,
                 node_name: str,
                 sensor_name: str,
                 topic_name: str,
                 ip: str,
                 port: int,
                 capin_ch_idx: int | None,
                 data_fps: int | None,
                 pkg_cycle: int | None):
        super().__init__(node_name)
        self.sensor_name = sensor_name
        self.topic_name = topic_name
        self.capin_ch_idx = capin_ch_idx

        # initialize publisher
        qos_profile = QoSProfile(
            depth=600,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=1)
        )
        self.rtk_pkg_types = ['GNGGA', 'GNGSA', 'GNRMC', 'GNGST', 'GNVTG', 'GNHDT', 'GNZDA',
                              'GPGGA', 'GPGSA', 'GPRMC', 'GPGST', 'GPVTG', 'GPHDT', 'GPZDA', 'GPGSV',
                              'GBGGA', 'GBGSA', 'GBRMC', 'GBGST', 'GBVTG', 'GBHDT', 'GBZDA', 'GBGSV',
                              'GLGGA', 'GLGSA', 'GLRMC', 'GLGST', 'GLVTG', 'GLHDT', 'GLZDA', 'GLGSV',
                              'GAGGA', 'GAGSA', 'GARMC', 'GAGST', 'GAVTG', 'GAHDT', 'GAZDA', 'GAGSV',
                              'GQGSV', 'GIGSV']
        self.imu_pkg_types = ['ADIS', 'MAG', 'LPS22', 'LPS28', 'TMS']
        self.sensor_msg_types = {"RTK": Rtk, "ADIS": IMU, "MAG": Mag, "LPS22": Baro, "LPS28": Baro}
        self.sensor_msg_topics = {"ADIS": self.topic_name + '/sensor_msg', "MAG": self.topic_name + '/sensor_msg',
                                  "RTK": self.topic_name + '/sensor_msg', "LPS22": self.topic_name + '/sensor_msg', "LPS28": self.topic_name + '/sensor_msg'}

        self._sensor_msg_publisher = self.create_publisher(self.sensor_msg_types[self.sensor_name], self.sensor_msg_topics[self.sensor_name], qos_profile)
        self._tcp_pkg_publisher = self.create_publisher(String, self.topic_name + '/tcp_str', qos_profile)
        self._tracker_fo_publisher = self.create_publisher(Float64, self.topic_name + '/fo', qos_profile)
        self._tracker_to_publisher = self.create_publisher(Float64, self.topic_name + '/to', qos_profile)

        # initialize subscriber
        if self.capin_ch_idx is not None:
            self._trout_subscriber = self.create_subscription(TimeStamp, f'/sync_data/trout/C{trOutCh.TMS.value:d}', self._trout_sub_callback, qos_profile)
            self._capin_subscriber = self.create_subscription(TimeStamp, f'/sync_data/capin/C{self.capin_ch_idx:d}', self._capin_sub_callback, qos_profile)

        # initialize time aligner
        if data_fps is not None:
            self.time_align = TimeAlign(data_fps=data_fps, logger=self.get_logger())

        # initialize pkg info
        self.init_sec: int | None = None
        self.init_nanosec: int | None = None

        self.PKG_CYCLE: int = pkg_cycle
        self.pkg_idx: int | None = None
        self.pkg_loss: int = 0

        self.rtk_msg = Rtk()
        self.rtk_msg.heading = 361.0
        self.gga_time = None

        # create TCP connection
        self.tcp_conk = TCPConnect(sensor_name=self.sensor_name, ip=ip, port=port, logger=self.get_logger(), recv_callback=self._recv_callback)

    def _capin_sub_callback(self, sync_msg: TimeStamp):
        """
         - Match the predicted capin timestamps with the data timestamps
         - Update self.freq_offset and self.time_offset
         - Publish sensor data with aligned timestamps (i.e., the timestamps from sync unit)
        """
        t_capin = SyncDeviceTime()
        t_capin.load_from_ts_msg(sync_msg)

        ret = self.time_align.add_a_pkg(a_t=t_capin, a_pkg=t_capin)
        if ret is not None:
            t_capin, msg_str = ret
            if t_capin is not None:
                sync_msg = t_capin.convert_to_ts_msg()
                self._publish_imu(msg=msg_str, sync_ts=sync_msg)    # TODO: sync_msg or t_capin?

                fo_msg, to_msg = Float64(), Float64()
                fo_msg.data = self.time_align.f_o
                to_msg.data = self.time_align.t_o
                self._tracker_fo_publisher.publish(fo_msg)
                self._tracker_to_publisher.publish(to_msg)

    def _trout_sub_callback(self, sync_msg: TimeStamp):
        # Update self.freq_offset and self.time_offset
        t_trout = SyncDeviceTime()
        t_trout.load_from_ts_msg(sync_msg)

        current_ts = self.get_clock().now().to_msg()
        wall_ts = current_ts.sec + current_ts.nanosec * 1e-9

        self.time_align.add_a_tms(a_wall=wall_ts, a_t=t_trout)

    def _recv_callback(self, pkg: str):
        recv_time = time.time()

        # Publish initial tcp string messages
        tcp_pkg = String()
        tcp_pkg.data = str(pkg)
        self._tcp_pkg_publisher.publish(tcp_pkg)

        pkg_split = pkg.split(',')
        pkg_type = pkg_split[0]
        match pkg_type:
            # publish rtk msg
            case k if k in self.rtk_pkg_types:
                self.rtk_msg, _ = SensorPkgHandler.handle_rtk_pkg(self.rtk_msg, msg=pkg)
                if pkg_type == 'GNGGA':
                    self.gga_time = recv_time

                    if self.rtk_msg.state != 7 and self.rtk_msg.heading == 361.0:
                        # waiting for heading value
                        return
                    else:
                        # state 7 or GGA with heading value
                        self._publish_rtk_and_reset()

                # receive HDT and have a waiting GGA message
                elif pkg_type == 'GNHDT':
                    if self.gga_time is not None:
                        self._publish_rtk_and_reset()

            # publish imu msg
            case k if k in self.imu_pkg_types:
                t_capin, msg = self._align_imu_pkg(msg=pkg)
                if t_capin is not None:
                    sync_msg = t_capin.convert_to_ts_msg()
                    self._publish_imu(msg=msg, sync_ts=sync_msg)

                    fo_msg, to_msg = Float64(), Float64()
                    fo_msg.data = self.time_align.f_o
                    to_msg.data = self.time_align.t_o
                    self._tracker_fo_publisher.publish(fo_msg)
                    self._tracker_to_publisher.publish(to_msg)
            case 'ERROR':
                match pkg.split(','):
                    case 'ERROR', msg_sn, msg_ts, '20000000':
                        pass
                    case _:
                        self.get_logger().warning(f"Sensor {self.sensor_name} error: {pkg}")
            case _:
                self.get_logger().warning(f"Unknown message: {pkg}")

    def _publish_rtk_and_reset(self):
        if self.rtk_msg.frame_id == FrameType.rtk.value:
            self._sensor_msg_publisher.publish(self.rtk_msg)
            self.rtk_msg = Rtk()
            self.rtk_msg.heading = 361.0
            self.gga_time = None

    def _align_imu_pkg(self, msg: str) -> None | tuple[SyncDeviceTime, str] | tuple[None, None]:
        pkg_split = msg.split(',')
        pkg_type = pkg_split[0]

        # get sensor device time
        if self.init_sec is None:
            self.init_sec = int(pkg_split[2].split('.')[0])
            self.init_nanosec = int(pkg_split[2].split('.')[1])
            pkg_sec = 0
            pkg_microsec = 0
        else:
            pkg_sec = int(pkg_split[2].split('.')[0]) - self.init_sec
            pkg_microsec = int(pkg_split[2].split('.')[1]) - self.init_nanosec
        t_sensor = SensorDeviceTime(sec=pkg_sec, microsec=pkg_microsec)

        # align timestamps with sync
        if pkg_type == 'TMS':
            current_ts = self.get_clock().now().to_msg()
            wall_ts = current_ts.sec + current_ts.nanosec * 1e-9

            self.time_align.add_b_tms(b_wall=wall_ts, b_t=t_sensor)
            return None, None
        else:
            # check package loss
            pkg_sn = int(pkg_split[1])
            if self.pkg_idx is not None:
                pkg_loss = ((pkg_sn + self.PKG_CYCLE) - (self.pkg_idx + 1) % self.PKG_CYCLE) % self.PKG_CYCLE
                self.pkg_loss += pkg_loss
                if pkg_loss > 0:
                    raise RuntimeError("Message loss exists!")
            self.pkg_idx = pkg_sn

            t_capin, pkg = self.time_align.add_b_pkg(b_t=t_sensor, b_pkg=msg)
            if t_capin is not None:
                return t_capin, pkg
            else:
                return None, None

    def _publish_imu(self, msg: str, sync_ts: TimeStamp):
        # publish sensor msg
        match self.sensor_name:
            case 'ADIS':
                msg = SensorPkgHandler.handle_adis_pkg(msg=msg, sync_ts=sync_ts)
            case 'MAG':
                msg, _ = SensorPkgHandler.handle_mag_pkg(msg=msg, sync_ts=sync_ts)
            case 'LPS22':
                msg, _ = SensorPkgHandler.handle_lps22_pkg(msg=msg, sync_ts=sync_ts)
            case 'LPS28':
                msg, _ = SensorPkgHandler.handle_lps28_pkg(msg=msg, sync_ts=sync_ts)
            case _:
                self.get_logger().error(f"Unknown IMU sensor: {self.sensor_name}")

        if msg is None:
            self.get_logger().warn(f"Invalid {self.sensor_name} message: {msg}")
        else:
            self._sensor_msg_publisher.publish(msg)

    def destroy_node(self):
        self.get_logger().info(f"Shutting down {self.sensor_name} node.")
        self.tcp_conk.is_running = False
        if self.tcp_conk.recv_thread.is_alive():
            self.tcp_conk.recv_thread.join(timeout=1.0)
        if self.tcp_conk.socket_:
            self.tcp_conk.socket_.close()
        super().destroy_node()
