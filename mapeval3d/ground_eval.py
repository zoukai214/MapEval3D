from typing import Dict
from typing import Optional

import numpy as np


def fit_plane_from_points(points_xyz: np.ndarray) -> np.ndarray:
    centroid = np.mean(points_xyz, axis=0)
    centered_points = points_xyz - centroid
    _, _, vh = np.linalg.svd(centered_points, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    d = -float(np.dot(normal, centroid))
    return np.array([normal[0], normal[1], normal[2], d], dtype=np.float64)


def point_distances_to_plane(points_xyz: np.ndarray, plane: np.ndarray) -> np.ndarray:
    normal = plane[:3]
    d = plane[3]
    return np.abs(points_xyz @ normal + d)


def point_signed_distances_to_plane(points_xyz: np.ndarray, plane: np.ndarray) -> np.ndarray:
    normal = plane[:3]
    d = plane[3]
    return points_xyz @ normal + d


def _sample_non_collinear_triplet(
    points_xyz: np.ndarray,
    rng: np.random.Generator,
) -> Optional[np.ndarray]:
    for _ in range(20):
        sample_indices = rng.choice(points_xyz.shape[0], size=3, replace=False)
        sample_points = points_xyz[sample_indices]
        cross = np.cross(sample_points[1] - sample_points[0], sample_points[2] - sample_points[0])
        if np.linalg.norm(cross) > 1e-8:
            return sample_points
    return None


def fit_plane_ransac(
    points_xyz: np.ndarray,
    distance_threshold_m: float,
    max_iterations: int,
    min_inliers: int,
    random_seed: int = 0,
) -> Dict[str, object]:
    rng = np.random.default_rng(random_seed)
    best_inlier_mask: Optional[np.ndarray] = None
    best_inlier_count = 0
    best_distance_sum = np.inf

    for _ in range(max_iterations):
        sample_points = _sample_non_collinear_triplet(points_xyz, rng)
        if sample_points is None:
            continue
        plane = fit_plane_from_points(sample_points)
        distances = point_distances_to_plane(points_xyz, plane)
        inlier_mask = distances <= distance_threshold_m
        inlier_count = int(np.count_nonzero(inlier_mask))
        if inlier_count < min_inliers:
            continue
        distance_sum = float(np.sum(distances[inlier_mask]))
        if inlier_count > best_inlier_count or (
            inlier_count == best_inlier_count and distance_sum < best_distance_sum
        ):
            best_inlier_mask = inlier_mask
            best_inlier_count = inlier_count
            best_distance_sum = distance_sum

    if best_inlier_mask is None:
        raise ValueError("RANSAC plane fitting failed.")

    refined_plane = fit_plane_from_points(points_xyz[best_inlier_mask])
    return {
        "plane": refined_plane,
        "ransac_inlier_mask": best_inlier_mask,
        "ransac_inlier_count": best_inlier_count,
    }


def evaluate_ground_block(
    points_xyz: np.ndarray,
    fit_method: str,
    distance_threshold_m: float,
    max_iterations: int,
    min_inliers: int,
    p95_percentile: float,
) -> Dict[str, object]:
    if fit_method != "ransac":
        raise ValueError("Unsupported fit_method.")

    fit_result = fit_plane_ransac(
        points_xyz,
        distance_threshold_m=distance_threshold_m,
        max_iterations=max_iterations,
        min_inliers=min_inliers,
    )
    plane = fit_result["plane"]
    distances = point_distances_to_plane(points_xyz, plane)
    signed_distances = point_signed_distances_to_plane(points_xyz, plane)
    signed_p5_threshold = float(np.percentile(signed_distances, 5.0))
    signed_p95_threshold = float(np.percentile(signed_distances, p95_percentile))
    p95_threshold = float(np.percentile(distances, p95_percentile))
    p95_inlier_mask = distances <= p95_threshold
    p95_inlier_distances = distances[p95_inlier_mask]
    total_point_count = int(points_xyz.shape[0])
    ransac_inlier_count = int(fit_result["ransac_inlier_count"])
    return {
        "plane": plane,
        "plane_fit_method": fit_method,
        "plane_distance_mean_p95": float(np.mean(p95_inlier_distances)),
        "plane_distance_variance_p95": float(np.var(p95_inlier_distances)),
        "plane_distance_p95_threshold": p95_threshold,
        "plane_distance_thickness_p95_p5": float(signed_p95_threshold - signed_p5_threshold),
        "plane_inlier_count_p95": int(p95_inlier_distances.shape[0]),
        "ransac_inlier_count": ransac_inlier_count,
        "ransac_outlier_ratio": float((total_point_count - ransac_inlier_count) / total_point_count),
    }
