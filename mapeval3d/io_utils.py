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

SUMMARY_FIELDNAMES = [
    "session_name",
    "total_row_count",
    "status_ok_count",
    "status_empty_count",
    "status_insufficient_points_count",
    "status_fit_failed_count",
    "plane_distance_mean_p95_mean",
    "plane_distance_mean_p95_std",
    "plane_distance_p95_threshold_mean",
    "plane_distance_p95_threshold_std",
    "plane_distance_thickness_p95_p5_mean",
    "plane_distance_thickness_p95_p5_std",
]


def read_laz_points_and_classification(
    input_laz: Path,
) -> Tuple[np.ndarray, np.ndarray, laspy.LasData]:
    las = laspy.read(input_laz)
    points_xyz = np.column_stack((las.x, las.y, las.z)).astype(np.float64)
    classifications = np.asarray(las.classification, dtype=np.uint8)
    return points_xyz, classifications, las


def read_laz_points_and_classification_from_inputs(
    input_laz_paths: Sequence[Path],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[laspy.LasData]]:
    if not input_laz_paths:
        raise ValueError("At least one input laz is required.")

    if len(input_laz_paths) == 1:
        points_xyz, classifications, las = read_laz_points_and_classification(input_laz_paths[0])
        source_indices = np.zeros(points_xyz.shape[0], dtype=np.int32)
        return points_xyz, classifications, source_indices, [las]

    points_xyz_parts = []
    classification_parts = []
    source_indices_parts = []
    source_las_list = []
    first_point_format_id = None

    for source_index, input_laz in enumerate(input_laz_paths):
        points_xyz, classifications, las = read_laz_points_and_classification(input_laz)
        point_format_id = int(las.header.point_format.id)
        if first_point_format_id is None:
            first_point_format_id = point_format_id
        elif point_format_id != first_point_format_id:
            raise ValueError(
                "All input laz files must share the same point format for merged evaluation."
            )

        points_xyz_parts.append(points_xyz)
        classification_parts.append(classifications)
        source_indices_parts.append(
            np.full(points_xyz.shape[0], source_index, dtype=np.int32)
        )
        source_las_list.append(las)

    merged_points_xyz = np.vstack(points_xyz_parts)
    merged_classifications = np.concatenate(classification_parts)
    merged_source_indices = np.concatenate(source_indices_parts)
    return merged_points_xyz, merged_classifications, merged_source_indices, source_las_list


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


def write_cropped_laz_from_inputs(
    output_path: Path,
    source_las_list: Sequence[laspy.LasData],
    selected_point_indices: np.ndarray,
    cropped_local_points: np.ndarray,
    source_indices: np.ndarray,
) -> None:
    if len(source_las_list) == 1:
        selected_las = source_las_list[0][selected_point_indices]
        selected_las.x = cropped_local_points[:, 0]
        selected_las.y = cropped_local_points[:, 1]
        selected_las.z = cropped_local_points[:, 2]
        output_path.parent.mkdir(parents=True, exist_ok=True)
        selected_las.write(output_path)
        return

    header = laspy.LasHeader(
        point_format=source_las_list[0].header.point_format.id,
        version=source_las_list[0].header.version,
    )
    header.scales = source_las_list[0].header.scales
    header.offsets = source_las_list[0].header.offsets
    selected_point_arrays = []
    source_point_offsets = []
    point_offset = 0

    for source_las in source_las_list:
        source_point_offsets.append(point_offset)
        point_offset += len(source_las.x)

    # 按来源文件拆分后再重建输出块，避免依赖不存在的整图模板文件。
    for source_index, source_las in enumerate(source_las_list):
        source_mask = source_indices[selected_point_indices] == source_index
        if not np.any(source_mask):
            continue
        source_selected_indices = selected_point_indices[source_mask] - source_point_offsets[source_index]
        source_selected_points = cropped_local_points[source_mask]
        selected_las = source_las[source_selected_indices]
        selected_las.x = source_selected_points[:, 0]
        selected_las.y = source_selected_points[:, 1]
        selected_las.z = source_selected_points[:, 2]
        selected_point_arrays.append(selected_las.points.array.copy())

    if not selected_point_arrays:
        raise ValueError("No cropped points available for writing.")

    output_las = laspy.LasData(header)
    if len(selected_point_arrays) == 1:
        merged_array = selected_point_arrays[0]
    else:
        merged_array = np.concatenate(selected_point_arrays)
    output_las.points = laspy.ScaleAwarePointRecord(
        merged_array,
        header.point_format,
        header.scales,
        header.offsets,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_las.write(output_path)


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


def write_plane_threshold_plot(output_path: Path, rows: Sequence[dict]) -> None:
    plot_rows = [
        row for row in rows
        if row["status"] == "ok" and row["plane_distance_p95_threshold"] != ""
    ]
    if not plot_rows:
        return

    ids = [int(row["id"]) for row in plot_rows]
    threshold_values = [float(row["plane_distance_p95_threshold"]) for row in plot_rows]

    plt.figure(figsize=(10, 4))
    plt.plot(ids, threshold_values, marker="o", linewidth=1.5, markersize=3)
    plt.xlabel("id")
    plt.ylabel("plane_distance_p95_threshold")
    plt.title("Plane Distance P95 Threshold by Frame ID")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150)
    plt.close()


def write_summary_rows(output_csv: Path, rows: Sequence[dict], session_name: str) -> None:
    status_counts = {}
    for row in rows:
        status = str(row["status"])
        status_counts[status] = status_counts.get(status, 0) + 1

    ok_rows = [row for row in rows if row["status"] == "ok"]

    def compute_mean_and_std(fieldname: str) -> Tuple[float, float]:
        values = np.asarray([float(row[fieldname]) for row in ok_rows], dtype=np.float64)
        if values.size == 0:
            return float("nan"), float("nan")
        return float(np.mean(values)), float(np.std(values))

    mean_p95_mean, mean_p95_std = compute_mean_and_std("plane_distance_mean_p95")
    threshold_mean, threshold_std = compute_mean_and_std("plane_distance_p95_threshold")
    thickness_mean, thickness_std = compute_mean_and_std("plane_distance_thickness_p95_p5")

    summary_row = {
        "session_name": session_name,
        "total_row_count": len(rows),
        "status_ok_count": status_counts.get("ok", 0),
        "status_empty_count": status_counts.get("empty", 0),
        "status_insufficient_points_count": status_counts.get("insufficient_points", 0),
        "status_fit_failed_count": status_counts.get("fit_failed", 0),
        "plane_distance_mean_p95_mean": mean_p95_mean,
        "plane_distance_mean_p95_std": mean_p95_std,
        "plane_distance_p95_threshold_mean": threshold_mean,
        "plane_distance_p95_threshold_std": threshold_std,
        "plane_distance_thickness_p95_p5_mean": thickness_mean,
        "plane_distance_thickness_p95_p5_std": thickness_std,
    }

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDNAMES)
        writer.writeheader()
        writer.writerow(summary_row)
