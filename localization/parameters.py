from enum import Enum
import numpy as np


# IMU error
# RTK IMU translation
# Camera intrinsics
# Camera error
# Tag size and margin


class Intrinsics:
    cam_intrinsics: tuple[float, float, float, float] = (1792.3479240773381, 1791.781820623154, 810.5107939731503, 550.5480797700516)
    cam_intrinsics_mtx = np.array([[cam_intrinsics[0], 0, cam_intrinsics[2]], [0, cam_intrinsics[1], cam_intrinsics[3]], [0, 0, 1]])


class Extrinsics:
    # translation (unit: m) from imu to rtk in imu_body coordinate
    # imu2rtk_b: np.ndarray = np.array([-97.37, -11.6, -89.]) * 1e-3
    imu2rtk_b: np.ndarray = np.array([0., 0., 0.]) * 1e-3  # @@ can be optimized into a single param file


class Params:
    # Kinetic Parameters
    # Assuming angular_acc(rad/s^2) & acc(m/s^2) is following N(0, Σ) when IMU is not available
    angular_acceleration_cov = np.diag(np.deg2rad(np.array([3, 3, 5])) ** 2)
    acceleration_cov = np.diag((np.array([4, 4, 2]) * 9.80665) ** 2)
    # Assuming angular_jerk(rad/s^3) & snap(m/s^4) is following N(0, Σ) when IMU is available
    angular_jerk_cov = np.diag(np.deg2rad(np.array([0.5, 0.5, 1])) ** 2)  # @@ Need fine-tuning
    snap_cov = np.diag((np.array([2, 2, 1])) ** 2)
    # Damping model (the movement of an object will naturally slow down)
    rotation_dumping = np.array([.15, .15, .05])
    translation_dumping = np.array([.05, .05, .15])

    # IMU Parameters
    g_n = np.array([0., 0., 9.8])
    acc_cov: np.ndarray = np.eye(3) * (75 * np.sqrt(260) * 9.8 * 1e-6) ** 2 * 100
    gyr_cov: np.ndarray = np.eye(3) * (0.0028 * np.sqrt(256)) ** 2 * 100

    # Magnetometer Parameters
    # Magnetic declination: deg (W: -1, E: +1)
    mag_var = 4.  # @@ should be updated with RTK data
    mag_cov: np.ndarray = np.eye(3) * np.array([0.1, 0.1, 0.1]) ** 2 * 100  # unit: mGauss

    # Camera Parameters
    # Tag size and margin
    tag_size = 0.173  # @@ can be optimized into a single param file
    tag_margin = 0.026

    # Sync Parameters
    ticks_per_sec: int = 10000
