import numpy as np


def quaternion_raw_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions.
    Usual torch rules for broadcasting apply.

    Args:
        a: Quaternions as tensor of shape (..., 4), real part first.
        b: Quaternions as tensor of shape (..., 4), real part first.

    Returns:
        The product of a and b, a tensor of quaternion shape (..., 4).
    """

    s, v = a[..., 0:1], a[..., 1:]
    t, u = b[..., 0:1], b[..., 1:]
    return np.concatenate([s * t - np.sum(v * u, axis=-1, keepdims=True), s * u + t * v + np.cross(v, u, axis=-1)], axis=-1)


def quaternion_invert(quaternion: np.ndarray) -> np.ndarray:
    """
    Given a quaternion representing rotation, get the quaternion representing
    its inverse.

    Args:
        quaternion: Quaternions as tensor of shape (..., 4), with real part
            first, which must be versors (unit quaternions).

    Returns:
        The inverse, a tensor of quaternions of shape (..., 4).
    """

    return np.array([1, -1, -1, -1]) * quaternion


def quaternion_to_matrix(quaternions: np.ndarray) -> np.ndarray:
    """
    Convert rotations given as quaternions to rotation matrices.

    Args:
        quaternions: quaternions with real part first,
            as tensor of shape (..., 4).

    Returns:
        Rotation matrices as tensor of shape (..., 3, 3).
    """

    r, i, j, k = tuple(np.moveaxis(quaternions, -1, 0))
    two_s = 2.0 / (quaternions * quaternions).sum(-1)

    o = np.stack(
        (
            1 - two_s * (j * j + k * k),
            two_s * (i * j - k * r),
            two_s * (i * k + j * r),
            two_s * (i * j + k * r),
            1 - two_s * (i * i + k * k),
            two_s * (j * k - i * r),
            two_s * (i * k - j * r),
            two_s * (j * k + i * r),
            1 - two_s * (i * i + j * j),
        ),
        -1,
    )
    return o.reshape(quaternions.shape[:-1] + (3, 3))


def standardize_quaternion(quaternions: np.ndarray) -> np.ndarray:
    """
    Convert a unit quaternion to a standard form: one in which the real
    part is non-negative.

    Args:
        quaternions: Quaternions with real part first,
            as tensor of shape (..., 4).

    Returns:
        Standardized quaternions as tensor of shape (..., 4).
    """
    quaternions = quaternions / np.linalg.norm(quaternions, axis=-1, keepdims=True)
    return np.where(quaternions[..., 0:1] < 0, -quaternions, quaternions)


def quaternion_to_str(quaternion: np.ndarray, decimal: int = 1) -> str:
    quaternion = standardize_quaternion(quaternion)
    q_norm = np.linalg.norm(quaternion[1:])
    angle = 2 * np.rad2deg(np.arctan2(q_norm, quaternion[0]))
    rotation_axis = quaternion[1:] / q_norm

    return f"[{angle:{3 + 1 + decimal}.{decimal}f}°, <{rotation_axis[0]:+{2 + 1 + decimal}.{decimal}f}, {rotation_axis[1]:+{2 + 1 + decimal}.{decimal}f}, {rotation_axis[2]:+{2 + 1 + decimal}.{decimal}f}>]"
