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


def transform_points_with_homogeneous_matrix(
    points_xyz: np.ndarray,
    transform_matrix: np.ndarray,
) -> np.ndarray:
    if transform_matrix.shape != (4, 4):
        raise ValueError("Transform matrix must be 4x4.")
    homogeneous_points = np.concatenate(
        (points_xyz, np.ones((points_xyz.shape[0], 1), dtype=np.float64)),
        axis=1,
    )
    transformed = homogeneous_points @ transform_matrix.T
    return transformed[:, :3]


def transform_world_points_to_car_frame(
    points_world: np.ndarray,
    gnss_translation: np.ndarray,
    rotation_world_from_keyframe: np.ndarray,
    gnss_to_lidar_matrix: np.ndarray,
    lidar_to_car_matrix: np.ndarray,
) -> np.ndarray:
    points_gnss = transform_world_points_to_keyframe(
        points_world,
        gnss_translation,
        rotation_world_from_keyframe,
    )
    points_lidar = transform_points_with_homogeneous_matrix(
        points_gnss,
        gnss_to_lidar_matrix,
    )
    return transform_points_with_homogeneous_matrix(
        points_lidar,
        lidar_to_car_matrix,
    )
