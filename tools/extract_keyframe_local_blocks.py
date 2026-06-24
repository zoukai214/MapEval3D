#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
from typing import Optional
from typing import Sequence

import numpy as np

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mapeval3d.config_utils import load_eval_config
from mapeval3d.calib_utils import find_clip_for_timestamp
from mapeval3d.calib_utils import load_session_calibrations
from mapeval3d.crop_utils import crop_points_by_rectangle
from mapeval3d.ground_eval import evaluate_ground_block
from mapeval3d.io_utils import build_block_filename
from mapeval3d.io_utils import filter_ground_points
from mapeval3d.io_utils import read_gps_pose_rows
from mapeval3d.io_utils import read_laz_points_and_classification
from mapeval3d.io_utils import select_keyframes_by_2d_distance
from mapeval3d.io_utils import write_cropped_laz
from mapeval3d.io_utils import write_index_rows
from mapeval3d.io_utils import write_plane_distance_plot
from mapeval3d.io_utils import write_plane_thickness_plot
from mapeval3d.pose_utils import quaternion_to_rotation_matrix
from mapeval3d.pose_utils import transform_world_points_to_car_frame
from mapeval3d.pose_utils import transform_world_points_to_keyframe


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-laz", required=True)
    parser.add_argument("--input-gps-msg", required=True)
    parser.add_argument("--session-path", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--config", required=True)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    config = load_eval_config(Path(args.config))
    output_dir = Path(args.output_dir)
    blocks_dir = output_dir / "blocks"
    blocks_dir.mkdir(parents=True, exist_ok=True)

    points_xyz, classifications, las = read_laz_points_and_classification(Path(args.input_laz))
    ground_points_xyz, ground_mask = filter_ground_points(points_xyz, classifications)
    pose_rows = read_gps_pose_rows(Path(args.input_gps_msg))
    clip_infos = load_session_calibrations(Path(args.session_path))
    keyframe_rows = select_keyframes_by_2d_distance(
        pose_rows,
        config["distance_interval_m"],
    )

    index_rows = []
    for block_id, row in enumerate(keyframe_rows):
        timestamp_ms = int(row["timestamp_ms"])
        base_row = {
            "id": block_id,
            "timestamp_ms": timestamp_ms,
            "plane_distance_mean_p95": "",
            "plane_distance_variance_p95": "",
            "plane_distance_p95_threshold": "",
            "plane_distance_thickness_p95_p5": "",
            "plane_inlier_count_p95": "",
            "ransac_inlier_count": "",
            "ransac_outlier_ratio": "",
        }

        translation = np.array(
            [float(row["x"]), float(row["y"]), float(row["z"])],
            dtype=np.float64,
        )
        rotation = quaternion_to_rotation_matrix(
            float(row["qx"]),
            float(row["qy"]),
            float(row["qz"]),
            float(row["qw"]),
        )
        clip_info = find_clip_for_timestamp(clip_infos, timestamp_ms)
        points_local = transform_world_points_to_car_frame(
            ground_points_xyz,
            translation,
            rotation,
            clip_info["gnss_to_lidar_matrix"],
            clip_info["lidar_to_car_matrix"],
        )
        crop_mask = crop_points_by_rectangle(
            points_local,
            config["window_size_x_m"],
            config["window_size_y_m"],
        )
        cropped_local_points = points_local[crop_mask]

        if cropped_local_points.shape[0] == 0:
            index_rows.append(
                {
                    **base_row,
                    "status": "empty",
                    "point_count": 0,
                }
            )
            continue

        if cropped_local_points.shape[0] < config["min_points_to_fit"]:
            index_rows.append(
                {
                    **base_row,
                    "status": "insufficient_points",
                    "point_count": int(cropped_local_points.shape[0]),
                }
            )
            continue

        try:
            eval_result = evaluate_ground_block(
                cropped_local_points,
                fit_method=config["fit_method"],
                distance_threshold_m=config["ransac_distance_threshold_m"],
                max_iterations=config["ransac_max_iterations"],
                min_inliers=config["ransac_min_inliers"],
                p95_percentile=config["p95_percentile"],
            )
        except ValueError:
            index_rows.append(
                {
                    **base_row,
                    "status": "fit_failed",
                    "point_count": int(cropped_local_points.shape[0]),
                }
            )
            continue

        index_eval_result = {
            "plane_distance_mean_p95": eval_result["plane_distance_mean_p95"],
            "plane_distance_variance_p95": eval_result["plane_distance_variance_p95"],
            "plane_distance_p95_threshold": eval_result["plane_distance_p95_threshold"],
            "plane_distance_thickness_p95_p5": eval_result["plane_distance_thickness_p95_p5"],
            "plane_inlier_count_p95": eval_result["plane_inlier_count_p95"],
            "ransac_inlier_count": eval_result["ransac_inlier_count"],
            "ransac_outlier_ratio": eval_result["ransac_outlier_ratio"],
        }

        block_name = build_block_filename(timestamp_ms)
        output_path = blocks_dir / block_name
        write_cropped_laz(output_path, las, ground_mask, crop_mask, cropped_local_points)
        index_rows.append(
            {
                **base_row,
                **index_eval_result,
                "status": "ok",
                "point_count": int(cropped_local_points.shape[0]),
            }
        )

    write_index_rows(output_dir / "index.csv", index_rows)
    write_plane_distance_plot(output_dir / "plane_distance_mean_p95.png", index_rows)
    write_plane_thickness_plot(output_dir / "plane_distance_thickness_p95_p5.png", index_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
