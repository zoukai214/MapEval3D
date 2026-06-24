import csv
import math
from pathlib import Path
from typing import List
from typing import Sequence
from typing import Tuple

import laspy
import matplotlib.pyplot as plt
import numpy as np


INDEX_FIELDNAMES = [
    "id",
    "timestamp_ms",
    "status",
    "point_count",
    "plane_distance_mean_p95",
    "plane_distance_variance_p95",
    "plane_distance_p95_threshold",
    "plane_distance_thickness_p95_p5",
    "plane_inlier_count_p95",
    "ransac_inlier_count",
    "ransac_outlier_ratio",
]


def read_laz_points_and_classification(
    input_laz: Path,
) -> Tuple[np.ndarray, np.ndarray, laspy.LasData]:
    las = laspy.read(input_laz)
    points_xyz = np.column_stack((las.x, las.y, las.z)).astype(np.float64)
    classifications = np.asarray(las.classification, dtype=np.uint8)
    return points_xyz, classifications, las


def filter_ground_points(
    points_xyz: np.ndarray,
    classifications: np.ndarray,
    ground_classification: int = 1,
) -> Tuple[np.ndarray, np.ndarray]:
    mask = classifications == ground_classification
    return points_xyz[mask], mask


def read_gps_pose_rows(input_gps_msg: Path) -> List[dict]:
    rows = []
    with input_gps_msg.open("r", newline="") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            rows.append(
                {
                    "timestamp_ms": int(row[0]),
                    "timestamp_sec": int(row[0]) / 1000.0,
                    "x": float(row[1]),
                    "y": float(row[2]),
                    "z": float(row[3]),
                    "qw": float(row[10]),
                    "qx": float(row[11]),
                    "qy": float(row[12]),
                    "qz": float(row[13]),
                }
            )
    return rows


def select_keyframes_by_2d_distance(
    pose_rows: Sequence[dict],
    distance_interval_m: float,
) -> List[dict]:
    if distance_interval_m <= 0.0:
        raise ValueError("Distance interval must be positive.")
    if not pose_rows:
        return []
    selected_rows = [pose_rows[0]]
    accumulated_distance = 0.0
    previous_row = pose_rows[0]
    for row in pose_rows[1:]:
        dx = row["x"] - previous_row["x"]
        dy = row["y"] - previous_row["y"]
        accumulated_distance += math.hypot(dx, dy)
        if accumulated_distance >= distance_interval_m:
            selected_rows.append(row)
            accumulated_distance = 0.0
        previous_row = row
    return selected_rows


def build_block_filename(timestamp_ms: int) -> str:
    return f"keyframe_{timestamp_ms}.laz"


def write_index_rows(output_csv: Path, rows: Sequence[dict]) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=INDEX_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def write_cropped_laz(
    output_path: Path,
    template_las: laspy.LasData,
    ground_mask: np.ndarray,
    crop_mask: np.ndarray,
    cropped_local_points: np.ndarray,
) -> None:
    ground_indices = np.flatnonzero(ground_mask)
    selected_indices = ground_indices[crop_mask]
    selected_las = template_las[selected_indices]
    selected_las.x = cropped_local_points[:, 0]
    selected_las.y = cropped_local_points[:, 1]
    selected_las.z = cropped_local_points[:, 2]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    selected_las.write(output_path)


def write_plane_distance_plot(output_path: Path, rows: Sequence[dict]) -> None:
    plot_rows = [
        row for row in rows
        if row["status"] == "ok" and row["plane_distance_mean_p95"] != ""
    ]
    if not plot_rows:
        return

    ids = [int(row["id"]) for row in plot_rows]
    mean_values = [float(row["plane_distance_mean_p95"]) for row in plot_rows]

    plt.figure(figsize=(10, 4))
    plt.plot(ids, mean_values, marker="o", linewidth=1.5, markersize=3)
    plt.xlabel("id")
    plt.ylabel("plane_distance_mean_p95")
    plt.title("Plane Distance Mean P95 by Frame ID")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150)
    plt.close()


def write_plane_thickness_plot(output_path: Path, rows: Sequence[dict]) -> None:
    plot_rows = [
        row for row in rows
        if row["status"] == "ok" and row["plane_distance_thickness_p95_p5"] != ""
    ]
    if not plot_rows:
        return

    ids = [int(row["id"]) for row in plot_rows]
    thickness_values = [float(row["plane_distance_thickness_p95_p5"]) for row in plot_rows]

    plt.figure(figsize=(10, 4))
    plt.plot(ids, thickness_values, marker="o", linewidth=1.5, markersize=3)
    plt.xlabel("id")
    plt.ylabel("plane_distance_thickness_p95_p5")
    plt.title("Plane Distance Thickness P95-P5 by Frame ID")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150)
    plt.close()
