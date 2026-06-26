import numpy as np


def crop_points_by_rectangle(
    points_local: np.ndarray,
    window_size_x: float,
    window_size_y: float,
    window_size_z_min: float,
    window_size_z_max: float,
) -> np.ndarray:
    if window_size_x <= 0.0 or window_size_y <= 0.0:
        raise ValueError("Window sizes must be positive.")
    if window_size_z_min >= window_size_z_max:
        raise ValueError("Z window min must be smaller than max.")
    half_x = window_size_x / 2.0
    half_y = window_size_y / 2.0
    return (
        (points_local[:, 0] >= -half_x)
        & (points_local[:, 0] <= half_x)
        & (points_local[:, 1] >= -half_y)
        & (points_local[:, 1] <= half_y)
        & (points_local[:, 2] >= window_size_z_min)
        & (points_local[:, 2] <= window_size_z_max)
    )
