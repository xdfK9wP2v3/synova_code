import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from my_msgs.msg import Rtk, IMU, Mag, Baro, TrackerState
from .ukf_utils.imu_tracker import IMUTracker
from .ukf_utils.mag_tracker import MAGTracker
from .ukf_utils.rtk_tracker import RTKTracker
from tcp_sensor_driver.utils.device_time import SyncDeviceTime

# TODO: check the monotonicity of ts!

class TrackerNode(Node):
    def __init__(self):
        super().__init__(node_name='tracker_node')

        self.sensors_list: list[str] = ["RTK", "ADIS", "MAG", "LPS22", "LPS28"]
        self.sensor_topic_map = {sensor: '/' + sensor.lower() + '_data' for sensor in self.sensors_list}
        self.sensor_msg_topic = {k: v + '/sensor_msg' for k, v in self.sensor_topic_map.items()}
        self.sensor_msg_types = {"RTK": Rtk, "ADIS": IMU, "MAG": Mag, "LPS22": Baro, "LPS28": Baro}

        qos_profile = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=1)
        )
        self.adis_subscriber_ = self.create_subscription(IMU, self.sensor_msg_topic['ADIS'], self._adis_callback, qos_profile)
        # self.mag_subscriber_ = self.create_subscription(Mag, self.sensor_msg_topic['MAG'], self._mag_callback, 10)  # TODO: coordinate transformation!

        self.tracker_state_publisher = self.create_publisher(TrackerState, '/tracker', 10)

        class CurrTracker(IMUTracker, MAGTracker): pass
        self.tracker = CurrTracker()

        self.prev_ts: int | None = None
        self.update_idx = 0

    def _initialize_tracker(self, ts: float):
        # TODO: make it adjustable with received msg

        init_imu_p_n = np.array([0., 0., 0.])
        init_imu_q_n = np.array([1., 0., 0., 0.])
        init_p_cov = np.eye(3) * np.array([1e-3, 1e-3, 1e-3]) ** 2   # TODO: Need fine-tuning
        init_q_cov = np.eye(4) * 1e-2 ** 2
        self.tracker.reset_state(init_p=init_imu_p_n, init_q=init_imu_q_n,
                                 init_p_cov=init_p_cov, init_q_cov=init_q_cov, current_time=ts)

    def _adis_callback(self, msg: IMU):
        ts = SyncDeviceTime.ts_msg_to_ts_nanosec_int(msg.ts)

        if self.tracker.state is None:
            self._initialize_tracker(ts)
            self.tracker.imu_update(data=msg)

            self.prev_ts = ts
            # self.get_logger().info(f"Init: {ts=}")
        else:
            self.tracker.predict(ts)
            self.tracker.imu_update(data=msg)

            # self.get_logger().info(f"Update: {ts=}")
            # self.get_logger().info(f"{self.update_idx=}, {ts - self.prev_ts}, {self.tracker.state.m['ALL'].shape}")
            # self.prev_ts = ts
            # self.update_idx += 1
            # self.get_logger().info(f"{self.tracker.state.m['Q']}")

        # Publish tracker state
        ukf_state_msg = TrackerState()
        ukf_state_msg.ts = msg.ts
        ukf_state_msg.frame_id = msg.frame_id

        ukf_state_msg.pos.x = self.tracker.state.m['P'][0]
        ukf_state_msg.pos.y = self.tracker.state.m['P'][1]
        ukf_state_msg.pos.z = self.tracker.state.m['P'][2]

        ukf_state_msg.qua.w = self.tracker.state.m['Q'][-1]   # Note that the order of quaternion is different
        ukf_state_msg.qua.x = self.tracker.state.m['Q'][0]
        ukf_state_msg.qua.y = self.tracker.state.m['Q'][1]
        ukf_state_msg.qua.z = self.tracker.state.m['Q'][2]

        self.tracker_state_publisher.publish(ukf_state_msg)


    def _mag_callback(self, msg: Mag):
        ts = SyncDeviceTime.ts_msg_to_ts_nanosec_int(msg.ts)
        if self.tracker.state is None:
            self._initialize_tracker(ts)
            self.tracker.mag_update(data=msg)
        else:
            self.tracker.predict(ts)
            self.tracker.mag_update(data=msg)