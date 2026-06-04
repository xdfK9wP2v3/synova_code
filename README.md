# SYNOVA

A multi-modal and multi-node collaborative sensing platform integrating cameras, LiDAR, mmWave radar, RTK-GNSS, IMU, magnetometer, barometer, and a custom STM32-based synchronization unit into a unified ROS2 pipeline. The hardware-software stack supports both real-time online sensor fusion and offline high-precision ground-truth trajectory estimation.

## Repository structure

```
SYNOVA/
├── PCB/                    # PCB design documents
├── STM32/                  # Embedded firmware (STM32H7)
├── localization/           # Pose estimation & LiDAR tracking toolkit
├── ros_workspace/          # ROS2 workspace
├── rosbag_parsers/         # Offline rosbag parsing utilities
├── network/                # Network configuration scripts
├── docker-compose.yml      # Multi-service Docker orchestration
├── Dockerfile.base         # Base ROS2 Docker image
├── Dockerfile.app          # Application-level Docker image
├── enterpoint.sh           # Container entrypoint script
├── sync_imu_config.py      # Sync-unit & IMU configuration tool
└── requirements.txt        # Python dependencies
```

## Open-source release overview

| Item | Hardware           | Software             |
|------|--------------------|----------------------|
| Sync unit | PCB                | STM32 firmware       |
| mmWave radar interface | PCB                | ROS2 driver          |
| RTK + 10-axis IMU board | PCB | STM32 firmware       |
| Time alignment | /                  | ROS2 driver          |
| Online pose estimation | /                  | ROS2 driver          |
| Offline ground-truth pose | /                  | Localization toolkit |
| ROS2 sensor drivers | /                  | C++/Python source    |
| Containerized runtime | /                  | Dockerfiles, scripts |
| Networking service | /                  | VPN config, scripts  |
| Mechanical mounts | -                  | /                    |

### PCB

Design files for three custom printed circuit boards used in the SYNOVA hardware platform:

| File | Board                                                                         |
|------|-------------------------------------------------------------------------------|
| `sync.pdf` | Central synchronization unit (STM32-based, multi-channel trigger & timestamp) |
| `imu.pdf` | IMU interface board (ADIS16470, LPS28, LPS22, MAG3110, RTK-GNSS)              |
| `radar_interface.pdf` | Radar interface board (AWR2243 with DCA1000EVM)                               |

The PDFs contain schematic and layout previews. Original Gerber files and design sources will be released after the de-anonymization period.

### STM32

Embedded source code for the STM32H7 microcontrollers running on the platform boards. Two projects are included:

- `Digital_PLL/` —Digital phase-locked loop for GPS-disciplined timing on the sync board. Includes a Python-based GUI debug tool (`DPLL_reader/`).
- `IMU/` — Sensor aggregation firmware managing IMU, barometer, magnetometer, and RTK-GNSS data streams over TCP.

### Localization

The SYNOVA Pose Estimation Toolkit. Implements online multi-sensor ES-UKF fusion (IMU, RTK, magnetometer, barometer) and offline LiDAR-based ground-truth tracking with Monte Carlo UKF and RTS smoothing. See the [localization README](localization/README.md) for full details.

### ros_workspace

A standard ROS2 (Humble) workspace containing the following packages:

| Package | Description |
|---------|-------------|
| `my_msgs` | Custom ROS2 message definitions shared across packages |
| `tcp_sensor_driver` | Python drivers for ADIS IMU, LPS22/LPS28 barometers, MAG3110 magnetometer, RTK-GNSS, and RTCM correction data, all over TCP |
| `camera_driver` | C++ driver for MvCamera with hardware-triggered image capture and LZ4 compression |
| `radar_driver_cpp` | C++ driver for TI AWR2243 mmWave cascade radar over FTDI + UDP |
| `ukf_tracker` | Real-time unscented Kalman filter for online pose estimation |
| `viewer` | Lightweight visualization nodes for camera feeds and tracking results |
| `all_sensors_launch` | Top-level launch files orchestrating the full sensor stack |

### rosbag_parsers

Offline tools for processing recorded rosbag (MCAP format) data:

- `data_parser.py` — Parse raw sensor messages into structured HDF5 datasets
- `bag_to_h5.py` — Batch convert paired ego/roadside rosbags to HDF5
- `ros_msg_define.py` — Message type definitions for deserialization
- `radar_config_parser.py` — Extract radar chirp and ADC configuration from onboard config files
- `lidar_helper.py` — LiDAR point cloud utilities

### network

Shell scripts for configuring the host network stack (MTU, ring buffer sizes, macvlan interfaces) for high-throughput sensor data streaming.

## Quick start

### Prerequisites

- Ubuntu 22.04 with ROS2 Humble
- Docker and Docker Compose

### Build the Docker image

```bash
docker build -f Dockerfile.base -t ros2-base-image .
```

### Build all ROS2 packages

```bash
cd ros_workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Launch ROS2 nodes with Docker
```bash
docker compose up -d --build
```

### Stop collecting
```bash
docker compose down --rmi all
```
