import socket
import re
import logging
import threading
import traceback
import os

def my_excepthook(args):
    traceback.print_exception(args.exc_type, args.exc_value, args.exc_traceback)
    os._exit(1)

threading.excepthook = my_excepthook


class TCPConnect:
    def __init__(self, sensor_name: str, ip: str, port: int, logger, recv_callback):
        self.sensor_name = sensor_name
        self.ip = ip
        self.port = port
        self.logger = logger
        self.re_parser = re.compile(r'\$([a-zA-Z0-9,.\-:/\+]*)\*([a-zA-Z0-9]{2})')

        # create TCP connection
        self.is_running: bool = True
        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_.settimeout(3.0)
        try:
            self.socket_.connect((self.ip, self.port))
            self.logger.info(f"Connected to {self.ip}:{self.port} ({self.sensor_name})")
        except Exception as e:
            self.logger.error(f"Failed to connect {self.sensor_name} at {self.ip}:{self.port}, error: {e}")
            raise e

        # start receiving
        self.receive_callback = recv_callback
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

    def _recv_loop(self):    # TODO: Refine the receive logic
        buffer = ''
        while self.is_running:
            try:
                data = self.socket_.recv(1024)
                if not data:
                    self.logger.warn(f"No data received from {self.sensor_name}, connection may be closed.")
                    self.is_running = False
                    break

                decoded_data = data.decode('utf-8', errors='ignore')
                buffer += decoded_data
                self.logger.debug(f"Received data: {decoded_data}")

                pkgs = buffer.split('\r\n')
                for pkg in pkgs[:-1]:
                    pkg = self.tcp_parser(pkg, self.re_parser)

                    if not pkg:
                        continue

                    self.receive_callback(pkg)
                buffer = pkgs[-1]

            except Exception as e:
                self.logger.error(f"Error reading data from {self.sensor_name}: {e}")
                raise e

    def send(self, command):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
                server_socket.connect((self.ip, self.port))
                msg = self.construct_msg(command)
                server_socket.send(msg.encode('utf-8'))

                self.logger.info(f"Sent to [{self.ip}:{self.port}] {msg}")
        except Exception as e:
            self.logger.error(f"Error sending command: {e}")

    @staticmethod
    def checksum_xor(msg: str) -> str:
        checksum = 0
        msg_bytes = msg.encode()
        for b in msg_bytes:
            checksum = (checksum ^ b) & 0xff

        return f"{checksum:02x}".upper()


    @staticmethod
    def tcp_parser(msg_str: str, re_parser: re.Pattern) -> str | None:
        msg_str = msg_str.strip()
        if not msg_str:
            return None

        re_match = re_parser.search(msg_str)
        if re_match is None:
            logging.warning(f"Invalid message format: {msg_str}")
            return None

        calculated_checksum = TCPConnect.checksum_xor(re_match.group(1))
        if calculated_checksum != re_match.group(2):
            logging.warning(f'Checksum failed for message: {msg_str}')
            return None

        return re_match.group(1)

    @staticmethod
    def construct_msg(payload: str) -> str:
        return "$" + payload + "*" + TCPConnect.checksum_xor(payload) + "\r\n"