import os.path

from tcp_sensor_driver.tcp_sensor_driver.utils.collect_utils import today_folder
from rclpy.node import Node
from std_msgs.msg import String

class TCPSubscriber(Node):
    def __init__(self,
                 node_name: str,
                 log_idx: int):
        super().__init__(node_name)
        self.log_idx = log_idx
        self.log_path = today_folder()

        self.topic_to_logfile = {
            'sync_data/tcp_str': f"{self.log_path}/{self.log_idx}_sync.log",
            'rtk_data/tcp_str': f"{self.log_path}/{self.log_idx}_rtk.log",
            'adis_data/tcp_str': f"{self.log_path}/{self.log_idx}_adis.log",
            'mag_data/tcp_str': f"{self.log_path}/{self.log_idx}_mag.log",
            'lps22_data/tcp_str': f"{self.log_path}/{self.log_idx}_lps22.log",
            'lps28_data/tcp_str': f"{self.log_path}/{self.log_idx}_lps28.log",
        }

        for logfile in self.topic_to_logfile.values():
            if os.path.exists(logfile):     # TODO: handle the case when the file already exists
                # raise FileExistsError(f'{logfile} already exists!')
                pass

        self.sync_subscriber_ = self.create_subscription(String, 'sync_data/tcp_str', lambda msg: self._sub_callback(msg, 'sync_data/tcp_str'), 10)
        self.rtk_subscriber_ = self.create_subscription(String, 'rtk_data/tcp_str', lambda msg: self._sub_callback(msg, 'rtk_data/tcp_str'), 10)
        self.adis_subscriber_ = self.create_subscription(String, 'adis_data/tcp_str', lambda msg: self._sub_callback(msg, 'adis_data/tcp_str'), 10)
        self.mag_subscriber_ = self.create_subscription(String, 'mag_data/tcp_str', lambda msg: self._sub_callback(msg, 'mag_data/tcp_str'), 10)
        self.lps22_subscriber_ = self.create_subscription(String, 'lps22_data/tcp_str', lambda msg: self._sub_callback(msg, 'lps22_data/tcp_str'), 10)
        self.lps28_subscriber_ = self.create_subscription(String, 'lps28_data/tcp_str', lambda msg: self._sub_callback(msg, 'lps28_data/tcp_str'), 10)

    def _sub_callback(self, msg: String, topic_name: str):
        log_file = self.topic_to_logfile.get(topic_name, None)

        if log_file:
            with open(f"{self.log_path}/{self.log_idx}_rtk.log", "w") as f:
                f.write(msg.data)
        else:
            self.get_logger().warn(f"Unrecognized topic {topic_name}.")