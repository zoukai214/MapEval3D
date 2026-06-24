import numpy as np


def normalize_quaternion(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        raise ValueError("Quaternion norm must be non-zero.")
    return quaternion / norm


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x, y, z, w = normalize_quaternion(qx, qy, qz, qw)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_world_points_to_keyframe(
    points_world: np.ndarray,
    translation: np.ndarray,
    rotation_world_from_keyframe: np.ndarray,
) -> np.ndarray:
    centered = points_world - translation.reshape(1, 3)
    return centered @ rotation_world_from_keyframe
