import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

import laspy
import numpy as np

from mapeval3d.calib_utils import extract_clip_start_timestamp_ms
from mapeval3d.calib_utils import find_clip_for_timestamp
from mapeval3d.calib_utils import load_session_calibrations
from mapeval3d.config_utils import load_eval_config
from mapeval3d.crop_utils import crop_points_by_rectangle
from mapeval3d.ground_eval import evaluate_ground_block
from mapeval3d.io_utils import build_block_filename
from mapeval3d.io_utils import filter_ground_points
from mapeval3d.io_utils import read_gps_pose_rows
from mapeval3d.io_utils import select_keyframes_by_2d_distance
from mapeval3d.io_utils import write_index_rows
from mapeval3d.io_utils import write_plane_distance_plot
from mapeval3d.io_utils import write_plane_threshold_plot
from mapeval3d.io_utils import write_plane_thickness_plot
from mapeval3d.io_utils import write_summary_rows
from mapeval3d.pose_utils import quaternion_to_rotation_matrix
from mapeval3d.pose_utils import transform_points_with_homogeneous_matrix
from mapeval3d.pose_utils import transform_world_points_to_car_frame
from mapeval3d.pose_utils import transform_world_points_to_keyframe


SCRIPT_PATH = Path(__file__).resolve().parent.parent / "tools" / "extract_keyframe_local_blocks.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("extract_keyframe_local_blocks", SCRIPT_PATH)
SCRIPT_MODULE = importlib.util.module_from_spec(SCRIPT_SPEC)
assert SCRIPT_SPEC.loader is not None
SCRIPT_SPEC.loader.exec_module(SCRIPT_MODULE)

MERGE_SCRIPT_PATH = Path(__file__).resolve().parent.parent / "tools" / "merge_summary_reports.py"
MERGE_SCRIPT_SPEC = importlib.util.spec_from_file_location("merge_summary_reports", MERGE_SCRIPT_PATH)
MERGE_SCRIPT_MODULE = importlib.util.module_from_spec(MERGE_SCRIPT_SPEC)
assert MERGE_SCRIPT_SPEC.loader is not None
MERGE_SCRIPT_SPEC.loader.exec_module(MERGE_SCRIPT_MODULE)

BATCH_SCRIPT_PATH = Path(__file__).resolve().parent.parent / "tools" / "batch_extract_intersections.py"
BATCH_SCRIPT_SPEC = importlib.util.spec_from_file_location("batch_extract_intersections", BATCH_SCRIPT_PATH)
BATCH_SCRIPT_MODULE = importlib.util.module_from_spec(BATCH_SCRIPT_SPEC)
assert BATCH_SCRIPT_SPEC.loader is not None
BATCH_SCRIPT_SPEC.loader.exec_module(BATCH_SCRIPT_MODULE)


class CalibrationUtilsTest(unittest.TestCase):

    def test_extract_clip_start_timestamp_from_name(self) -> None:
        self.assertEqual(
            extract_clip_start_timestamp_ms("GACRT025_1762669096_MT"),
            1762669096000,
        )

    def test_find_clip_for_timestamp_matches_interval(self) -> None:
        clip_infos = [
            {"clip_name": "A_1000_MT", "start_timestamp_ms": 1000},
            {"clip_name": "A_2000_MT", "start_timestamp_ms": 2000},
            {"clip_name": "A_3000_MT", "start_timestamp_ms": 3000},
        ]
        matched = find_clip_for_timestamp(clip_infos, 2500)
        self.assertEqual(matched["clip_name"], "A_2000_MT")

    def test_load_session_calibrations_reads_two_extrinsics(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            session_path = Path(temp_dir)
            clip_dir = session_path / "GACRT025_1762669096_MT" / "calib_extract"
            clip_dir.mkdir(parents=True)
            gnss_to_lidar = {
                "gnss-to-lidar-top": {
                    "param": {
                        "sensor_calib": {
                            "data": [
                                [1.0, 0.0, 0.0, 1.0],
                                [0.0, 1.0, 0.0, 2.0],
                                [0.0, 0.0, 1.0, 3.0],
                                [0.0, 0.0, 0.0, 1.0],
                            ]
                        }
                    }
                }
            }
            lidar_to_car = {
                "lidar-top-to-car": {
                    "param": {
                        "sensor_calib": {
                            "data": [
                                [1.0, 0.0, 0.0, 0.5],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0],
                            ]
                        }
                    }
                }
            }
            (clip_dir / "calib_gnss_to_lidar_top_ENU.json").write_text(
                json.dumps(gnss_to_lidar),
                encoding="utf-8",
            )
            (clip_dir / "calib_lidar_top_to_car.json").write_text(
                json.dumps(lidar_to_car),
                encoding="utf-8",
            )

            clip_infos = load_session_calibrations(session_path)

        self.assertEqual(len(clip_infos), 1)
        np.testing.assert_allclose(
            clip_infos[0]["gnss_to_lidar_matrix"],
            np.array(
                [
                    [1.0, 0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [0.0, 0.0, 1.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        )
        np.testing.assert_allclose(
            clip_infos[0]["lidar_to_car_matrix"][0, 3],
            0.5,
        )

    def test_load_session_calibrations_skips_non_clip_directories(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            session_path = Path(temp_dir)
            (session_path / "prelabel").mkdir()
            (session_path / "INTERSECTION_ws07xvrf56es").mkdir()
            clip_dir = session_path / "GACRT025_1762398530_MT" / "calib_extract"
            clip_dir.mkdir(parents=True)
            gnss_to_lidar = {
                "gnss-to-lidar-top": {
                    "param": {
                        "sensor_calib": {
                            "data": [
                                [1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0],
                            ]
                        }
                    }
                }
            }
            lidar_to_car = {
                "lidar-top-to-car": {
                    "param": {
                        "sensor_calib": {
                            "data": [
                                [1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0],
                            ]
                        }
                    }
                }
            }
            (clip_dir / "calib_gnss_to_lidar_top_ENU.json").write_text(
                json.dumps(gnss_to_lidar),
                encoding="utf-8",
            )
            (clip_dir / "calib_lidar_top_to_car.json").write_text(
                json.dumps(lidar_to_car),
                encoding="utf-8",
            )

            clip_infos = load_session_calibrations(session_path)

        self.assertEqual([info["clip_name"] for info in clip_infos], ["GACRT025_1762398530_MT"])


class PoseUtilsTest(unittest.TestCase):

    def test_quaternion_to_rotation_matrix_identity(self) -> None:
        rotation = quaternion_to_rotation_matrix(0.0, 0.0, 0.0, 1.0)
        np.testing.assert_allclose(rotation, np.eye(3))

    def test_transform_world_points_to_keyframe_translation_only(self) -> None:
        points_world = np.array([[10.0, 5.0, 1.0], [11.0, 7.0, 1.0]])
        translation = np.array([10.0, 5.0, 1.0])
        rotation = np.eye(3)
        transformed = transform_world_points_to_keyframe(
            points_world,
            translation,
            rotation,
        )
        np.testing.assert_allclose(
            transformed,
            np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 0.0]]),
        )

    def test_transform_world_points_to_keyframe_with_yaw_rotation(self) -> None:
        points_world = np.array([[1.0, 0.0, 0.0]])
        translation = np.array([0.0, 0.0, 0.0])
        half_sqrt = np.sqrt(0.5)
        rotation = quaternion_to_rotation_matrix(0.0, 0.0, half_sqrt, half_sqrt)
        transformed = transform_world_points_to_keyframe(
            points_world,
            translation,
            rotation,
        )
        np.testing.assert_allclose(
            transformed,
            np.array([[0.0, -1.0, 0.0]]),
            atol=1e-6,
        )

    def test_transform_points_with_homogeneous_matrix_translation_only(self) -> None:
        points = np.array([[0.0, 0.0, 0.0]])
        matrix = np.array(
            [
                [1.0, 0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0, 2.0],
                [0.0, 0.0, 1.0, 3.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        transformed = transform_points_with_homogeneous_matrix(points, matrix)
        np.testing.assert_allclose(transformed, np.array([[1.0, 2.0, 3.0]]))

    def test_transform_world_points_to_car_frame_chains_gnss_lidar_car(self) -> None:
        points_world = np.array([[10.0, 0.0, 0.0]])
        gnss_translation = np.array([10.0, 0.0, 0.0])
        gnss_rotation = np.eye(3)
        gnss_to_lidar_matrix = np.array(
            [
                [1.0, 0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        lidar_to_car_matrix = np.array(
            [
                [1.0, 0.0, 0.0, 0.5],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        transformed = transform_world_points_to_car_frame(
            points_world,
            gnss_translation,
            gnss_rotation,
            gnss_to_lidar_matrix,
            lidar_to_car_matrix,
        )
        np.testing.assert_allclose(transformed, np.array([[1.5, 0.0, 0.0]]))


class CropUtilsTest(unittest.TestCase):

    def test_crop_points_by_rectangle_keeps_boundary_points(self) -> None:
        points_local = np.array(
            [
                [-2.5, -1.0, 0.0],
                [2.5, 1.0, 0.0],
                [2.6, 0.0, 0.0],
                [0.0, 3.1, 0.0],
            ]
        )
        mask = crop_points_by_rectangle(
            points_local,
            window_size_x=5.0,
            window_size_y=4.0,
            window_size_z_min=-2.0,
            window_size_z_max=2.0,
        )
        np.testing.assert_array_equal(mask, np.array([True, True, False, False]))

    def test_crop_points_by_rectangle_filters_z_range(self) -> None:
        points_local = np.array(
            [
                [0.0, 0.0, -2.0],
                [0.0, 0.0, 2.0],
                [0.0, 0.0, -2.1],
                [0.0, 0.0, 2.1],
                [3.0, 0.0, 0.0],
            ]
        )
        mask = crop_points_by_rectangle(
            points_local,
            window_size_x=4.0,
            window_size_y=4.0,
            window_size_z_min=-2.0,
            window_size_z_max=2.0,
        )
        np.testing.assert_array_equal(mask, np.array([True, True, False, False, False]))

    def test_crop_points_by_rectangle_supports_non_square_window(self) -> None:
        points_local = np.array(
            [
                [3.9, 0.9, 0.0],
                [3.9, 1.1, 0.0],
                [4.1, 0.9, 0.0],
            ]
        )
        mask = crop_points_by_rectangle(
            points_local,
            window_size_x=8.0,
            window_size_y=2.0,
            window_size_z_min=-2.0,
            window_size_z_max=2.0,
        )
        np.testing.assert_array_equal(mask, np.array([True, False, False]))


class IoUtilsTest(unittest.TestCase):

    def test_filter_ground_points_keeps_only_classification_one(self) -> None:
        points_xyz = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [2.0, 2.0, 2.0]])
        classifications = np.array([0, 1, 1], dtype=np.uint8)
        ground_points, ground_mask = filter_ground_points(points_xyz, classifications)
        np.testing.assert_allclose(ground_points, np.array([[1.0, 1.0, 1.0], [2.0, 2.0, 2.0]]))
        np.testing.assert_array_equal(ground_mask, np.array([False, True, True]))

    def test_write_index_rows_writes_expected_columns(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_csv = Path(temp_dir) / "index.csv"
            rows = [
                {
                    "id": 0,
                    "timestamp_ms": 1500,
                    "status": "ok",
                    "point_count": 12,
                    "plane_distance_mean_p95": 0.01,
                    "plane_distance_variance_p95": 0.0001,
                    "plane_distance_p95_threshold": 0.03,
                    "plane_distance_thickness_p95_p5": 0.02,
                    "plane_inlier_count_p95": 11,
                    "ransac_inlier_count": 12,
                    "ransac_outlier_ratio": 0.0,
                }
            ]
            write_index_rows(output_csv, rows)
            with output_csv.open("r", newline="") as handle:
                reader = csv.DictReader(handle)
                data = list(reader)
            self.assertEqual(data[0]["id"], "0")
            self.assertEqual(data[0]["timestamp_ms"], "1500")
            self.assertEqual(data[0]["plane_inlier_count_p95"], "11")

    def test_write_plane_distance_plot_creates_png(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_png = Path(temp_dir) / "plot.png"
            rows = [
                {
                    "id": 0,
                    "timestamp_ms": 1000,
                    "status": "ok",
                    "point_count": 10,
                    "plane_distance_mean_p95": 0.01,
                    "plane_distance_variance_p95": 0.0001,
                    "plane_distance_p95_threshold": 0.02,
                    "plane_distance_thickness_p95_p5": 0.015,
                    "plane_inlier_count_p95": 9,
                    "ransac_inlier_count": 10,
                    "ransac_outlier_ratio": 0.0,
                },
                {
                    "id": 1,
                    "timestamp_ms": 2000,
                    "status": "ok",
                    "point_count": 11,
                    "plane_distance_mean_p95": 0.02,
                    "plane_distance_variance_p95": 0.0002,
                    "plane_distance_p95_threshold": 0.03,
                    "plane_distance_thickness_p95_p5": 0.025,
                    "plane_inlier_count_p95": 10,
                    "ransac_inlier_count": 11,
                    "ransac_outlier_ratio": 0.0,
                },
            ]
            write_plane_distance_plot(output_png, rows)
            self.assertTrue(output_png.exists())

    def test_write_plane_thickness_plot_creates_png(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_png = Path(temp_dir) / "thickness_plot.png"
            rows = [
                {
                    "id": 0,
                    "timestamp_ms": 1000,
                    "status": "ok",
                    "point_count": 10,
                    "plane_distance_mean_p95": 0.01,
                    "plane_distance_variance_p95": 0.0001,
                    "plane_distance_p95_threshold": 0.02,
                    "plane_distance_thickness_p95_p5": 0.015,
                    "plane_inlier_count_p95": 9,
                    "ransac_inlier_count": 10,
                    "ransac_outlier_ratio": 0.0,
                },
                {
                    "id": 1,
                    "timestamp_ms": 2000,
                    "status": "ok",
                    "point_count": 11,
                    "plane_distance_mean_p95": 0.02,
                    "plane_distance_variance_p95": 0.0002,
                    "plane_distance_p95_threshold": 0.03,
                    "plane_distance_thickness_p95_p5": 0.025,
                    "plane_inlier_count_p95": 10,
                    "ransac_inlier_count": 11,
                    "ransac_outlier_ratio": 0.0,
                },
            ]
            write_plane_thickness_plot(output_png, rows)
            self.assertTrue(output_png.exists())

    def test_write_plane_threshold_plot_creates_png(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_png = Path(temp_dir) / "threshold_plot.png"
            rows = [
                {
                    "id": 0,
                    "timestamp_ms": 1000,
                    "status": "ok",
                    "point_count": 10,
                    "plane_distance_mean_p95": 0.01,
                    "plane_distance_variance_p95": 0.0001,
                    "plane_distance_p95_threshold": 0.02,
                    "plane_distance_thickness_p95_p5": 0.015,
                    "plane_inlier_count_p95": 9,
                    "ransac_inlier_count": 10,
                    "ransac_outlier_ratio": 0.0,
                },
                {
                    "id": 1,
                    "timestamp_ms": 2000,
                    "status": "ok",
                    "point_count": 11,
                    "plane_distance_mean_p95": 0.02,
                    "plane_distance_variance_p95": 0.0002,
                    "plane_distance_p95_threshold": 0.03,
                    "plane_distance_thickness_p95_p5": 0.025,
                    "plane_inlier_count_p95": 10,
                    "ransac_inlier_count": 11,
                    "ransac_outlier_ratio": 0.0,
                },
            ]
            write_plane_threshold_plot(output_png, rows)
            self.assertTrue(output_png.exists())

    def test_write_summary_rows_uses_only_ok_rows_for_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_csv = Path(temp_dir) / "summary.csv"
            rows = [
                {
                    "id": 0,
                    "timestamp_ms": 1000,
                    "status": "ok",
                    "point_count": 10,
                    "plane_distance_mean_p95": 1.0,
                    "plane_distance_variance_p95": 0.0,
                    "plane_distance_p95_threshold": 2.0,
                    "plane_distance_thickness_p95_p5": 3.0,
                    "plane_inlier_count_p95": 9,
                    "ransac_inlier_count": 10,
                    "ransac_outlier_ratio": 0.0,
                },
                {
                    "id": 1,
                    "timestamp_ms": 2000,
                    "status": "ok",
                    "point_count": 11,
                    "plane_distance_mean_p95": 3.0,
                    "plane_distance_variance_p95": 0.0,
                    "plane_distance_p95_threshold": 4.0,
                    "plane_distance_thickness_p95_p5": 5.0,
                    "plane_inlier_count_p95": 10,
                    "ransac_inlier_count": 11,
                    "ransac_outlier_ratio": 0.0,
                },
                {
                    "id": 2,
                    "timestamp_ms": 3000,
                    "status": "insufficient_points",
                    "point_count": 5,
                    "plane_distance_mean_p95": "",
                    "plane_distance_variance_p95": "",
                    "plane_distance_p95_threshold": "",
                    "plane_distance_thickness_p95_p5": "",
                    "plane_inlier_count_p95": "",
                    "ransac_inlier_count": "",
                    "ransac_outlier_ratio": "",
                },
            ]
            write_summary_rows(output_csv, rows, session_name="test_session")
            with output_csv.open("r", newline="") as handle:
                reader = csv.DictReader(handle)
                summary = list(reader)[0]
            self.assertEqual(reader.fieldnames[0], "session_name")
            self.assertEqual(summary["session_name"], "test_session")
            self.assertEqual(summary["total_row_count"], "3")
            self.assertEqual(summary["status_ok_count"], "2")
            self.assertEqual(summary["status_insufficient_points_count"], "1")
            self.assertAlmostEqual(float(summary["plane_distance_mean_p95_mean"]), 2.0)
            self.assertAlmostEqual(float(summary["plane_distance_p95_threshold_mean"]), 3.0)
            self.assertAlmostEqual(float(summary["plane_distance_thickness_p95_p5_mean"]), 4.0)
            self.assertAlmostEqual(float(summary["plane_distance_mean_p95_std"]), 1.0)

    def test_build_block_filename_uses_timestamp(self) -> None:
        self.assertEqual(build_block_filename(1500), "keyframe_1500.laz")

    def test_select_keyframes_by_2d_distance_keeps_first_and_every_10m(self) -> None:
        pose_rows = [
            {"timestamp_ms": 1000, "x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            {"timestamp_ms": 2000, "x": 4.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            {"timestamp_ms": 3000, "x": 10.5, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            {"timestamp_ms": 4000, "x": 18.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            {"timestamp_ms": 5000, "x": 21.0, "y": 8.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        ]
        keyframes = select_keyframes_by_2d_distance(pose_rows, distance_interval_m=10.0)
        self.assertEqual([row["timestamp_ms"] for row in keyframes], [1000, 3000, 5000])

    def test_read_gps_pose_rows_parses_gps_msg_format(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            gps_path = Path(temp_dir) / "gps_msg.txt"
            gps_path.write_text(
                "#t,x,y,z,l,l,h,ins_status,pos_type,err,qw,qx,qy,qz\n"
                "1000,1.0,2.0,3.0,0,0,0,0,0,0,1.0,0.1,0.2,0.3\n",
                encoding="utf-8",
            )
            rows = read_gps_pose_rows(gps_path)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["timestamp_ms"], 1000)
        self.assertAlmostEqual(rows[0]["x"], 1.0)
        self.assertAlmostEqual(rows[0]["qy"], 0.2)


class ExtractionToolTest(unittest.TestCase):

    def test_load_eval_config_reads_json_file(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "config.json"
            config_path.write_text(
                json.dumps(
                    {
                        "window_size_x_m": 4.0,
                        "window_size_y_m": 4.0,
                        "distance_interval_m": 4.0,
                        "fit_method": "ransac",
                        "ransac_distance_threshold_m": 0.05,
                        "ransac_max_iterations": 50,
                        "ransac_min_inliers": 10,
                        "p95_percentile": 95.0,
                        "min_points_to_fit": 10,
                    }
                ),
                encoding="utf-8",
            )
            config = load_eval_config(config_path)
        self.assertEqual(config["fit_method"], "ransac")
        self.assertEqual(config["window_size_x_m"], 4.0)
        self.assertEqual(config["window_size_z_min_m"], -2.0)
        self.assertEqual(config["window_size_z_max_m"], 2.0)
        self.assertEqual(config["save_blocks_laz"], True)

    def test_load_eval_config_reads_save_blocks_laz_false(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "config.json"
            config_path.write_text(
                json.dumps(
                    {
                        "window_size_x_m": 4.0,
                        "window_size_y_m": 4.0,
                        "window_size_z_min_m": -1.5,
                        "window_size_z_max_m": 1.5,
                        "distance_interval_m": 4.0,
                        "fit_method": "ransac",
                        "ransac_distance_threshold_m": 0.05,
                        "ransac_max_iterations": 50,
                        "ransac_min_inliers": 10,
                        "p95_percentile": 95.0,
                        "min_points_to_fit": 10,
                        "save_blocks_laz": False,
                    }
                ),
                encoding="utf-8",
            )
            config = load_eval_config(config_path)
        self.assertEqual(config["save_blocks_laz"], False)
        self.assertEqual(config["window_size_z_min_m"], -1.5)
        self.assertEqual(config["window_size_z_max_m"], 1.5)

    def test_evaluate_ground_block_uses_ransac_and_p95(self) -> None:
        rng = np.random.default_rng(0)
        xy = rng.uniform(-1.0, 1.0, size=(200, 2))
        z = 0.5 * xy[:, 0] - 0.2 * xy[:, 1] + 0.1 + rng.normal(0.0, 0.005, size=200)
        plane_points = np.column_stack((xy, z))
        outliers = np.array(
            [
                [0.0, 0.0, 1.0],
                [0.5, -0.5, -1.0],
                [-0.5, 0.5, 1.2],
            ]
        )
        points = np.vstack((plane_points, outliers))
        result = evaluate_ground_block(
            points,
            fit_method="ransac",
            distance_threshold_m=0.03,
            max_iterations=200,
            min_inliers=100,
            p95_percentile=95.0,
        )
        self.assertEqual(result["plane_fit_method"], "ransac")
        self.assertGreaterEqual(result["ransac_inlier_count"], 190)
        self.assertGreater(result["plane_distance_thickness_p95_p5"], 0.0)
        self.assertLess(result["plane_distance_mean_p95"], 0.02)
        self.assertLess(result["plane_distance_variance_p95"], 0.0005)
        self.assertGreaterEqual(result["ransac_outlier_ratio"], 0.0)
        self.assertLess(result["ransac_outlier_ratio"], 0.1)

    def test_plane_distance_thickness_uses_signed_distances(self) -> None:
        xy = np.array(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [0.0, 1.0],
                [1.0, 1.0],
                [0.5, 0.2],
                [0.2, 0.5],
            ]
        )
        signed_offsets = np.array([-0.03, -0.02, -0.01, 0.01, 0.02, 0.03])
        plane_points = np.column_stack((xy, signed_offsets))
        result = evaluate_ground_block(
            plane_points,
            fit_method="ransac",
            distance_threshold_m=0.05,
            max_iterations=50,
            min_inliers=3,
            p95_percentile=95.0,
        )
        self.assertGreater(result["plane_distance_thickness_p95_p5"], 0.04)

    def test_main_writes_block_and_index_for_ground_points(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            input_laz = temp_root / "input.laz"
            input_pose_csv = temp_root / "poses.csv"
            output_dir = temp_root / "output"
            config_path = temp_root / "config.json"
            session_path = temp_root / "session"
            clip_dir = session_path / "GACRT025_1_MT" / "calib_extract"
            clip_dir.mkdir(parents=True)

            config_path.write_text(
                json.dumps(
                    {
                        "window_size_x_m": 10.0,
                        "window_size_y_m": 10.0,
                        "distance_interval_m": 10.0,
                        "fit_method": "ransac",
                        "ransac_distance_threshold_m": 0.05,
                        "ransac_max_iterations": 100,
                        "ransac_min_inliers": 20,
                        "p95_percentile": 95.0,
                        "min_points_to_fit": 20,
                    }
                ),
                encoding="utf-8",
            )
            (clip_dir / "calib_gnss_to_lidar_top_ENU.json").write_text(
                json.dumps(
                    {
                        "gnss-to-lidar-top": {
                            "param": {
                                "sensor_calib": {
                                    "data": [
                                        [1.0, 0.0, 0.0, 1.0],
                                        [0.0, 1.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0],
                                        [0.0, 0.0, 0.0, 1.0],
                                    ]
                                }
                            }
                        }
                    }
                ),
                encoding="utf-8",
            )
            (clip_dir / "calib_lidar_top_to_car.json").write_text(
                json.dumps(
                    {
                        "lidar-top-to-car": {
                            "param": {
                                "sensor_calib": {
                                    "data": [
                                        [1.0, 0.0, 0.0, 0.5],
                                        [0.0, 1.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0],
                                        [0.0, 0.0, 0.0, 1.0],
                                    ]
                                }
                            }
                        }
                    }
                ),
                encoding="utf-8",
            )

            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            local_x = np.linspace(-1.5, 1.5, 25)
            local_y = np.linspace(-1.5, 1.5, 25)
            grid_x, grid_y = np.meshgrid(local_x, local_y)
            flat_x = grid_x.reshape(-1)
            flat_y = grid_y.reshape(-1)
            flat_z = 1.0 + 0.01 * flat_x - 0.02 * flat_y
            las.x = np.concatenate((10.0 + flat_x, np.array([20.0])))
            las.y = np.concatenate((5.0 + flat_y, np.array([20.0])))
            las.z = np.concatenate((flat_z, np.array([1.0])))
            las.classification = np.concatenate(
                (np.ones(flat_x.shape[0], dtype=np.uint8), np.array([0], dtype=np.uint8))
            )
            las.write(input_laz)

            with input_pose_csv.open("w", newline="") as handle:
                handle.write("#t,x,y,z,l,l,h,ins_status,pos_type,err,qw,qx,qy,qz\n")
                handle.write("1500,10.0,5.0,1.0,0,0,0,0,0,0,1.0,0.0,0.0,0.0\n")

            exit_code = SCRIPT_MODULE.main(
                [
                    "--input-laz",
                    str(input_laz),
                    "--input-gps-msg",
                    str(input_pose_csv),
                    "--session-path",
                    str(session_path),
                    "--output-dir",
                    str(output_dir),
                    "--config",
                    str(config_path),
                ]
            )
            self.assertEqual(exit_code, 0)

            output_csv = output_dir / "index.csv"
            self.assertTrue(output_csv.exists())
            with output_csv.open("r", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(
                rows[0].keys(),
                {
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
                },
            )
            self.assertEqual(rows[0]["id"], "0")
            self.assertEqual(rows[0]["status"], "ok")
            self.assertEqual(rows[0]["point_count"], "625")
            self.assertEqual(rows[0]["timestamp_ms"], "1500")
            self.assertNotEqual(rows[0]["plane_distance_mean_p95"], "")
            self.assertNotEqual(rows[0]["plane_distance_thickness_p95_p5"], "")
            output_laz = output_dir / "blocks" / "keyframe_1500.laz"
            self.assertTrue(output_laz.exists())
            output_points = laspy.read(output_laz)
            self.assertEqual(len(output_points.x), 625)
            np.testing.assert_allclose(np.min(np.asarray(output_points.x)), 0.0)
            np.testing.assert_allclose(np.max(np.asarray(output_points.x)), 3.0)
            summary_csv = output_dir / "summary.csv"
            self.assertTrue(summary_csv.exists())
            with summary_csv.open("r", newline="") as handle:
                summary = list(csv.DictReader(handle))[0]
            self.assertEqual(summary["session_name"], "session")
            self.assertEqual(summary["status_ok_count"], "1")
            self.assertTrue((output_dir / "plane_distance_mean_p95.png").exists())
            self.assertTrue((output_dir / "plane_distance_p95_threshold.png").exists())
            self.assertTrue((output_dir / "plane_distance_thickness_p95_p5.png").exists())

    def test_main_skips_block_laz_when_config_disabled(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            input_laz = temp_root / "input.laz"
            input_pose_csv = temp_root / "poses.csv"
            output_dir = temp_root / "output"
            config_path = temp_root / "config.json"
            session_path = temp_root / "session"
            clip_dir = session_path / "GACRT025_1_MT" / "calib_extract"
            clip_dir.mkdir(parents=True)

            config_path.write_text(
                json.dumps(
                    {
                        "window_size_x_m": 10.0,
                        "window_size_y_m": 10.0,
                        "distance_interval_m": 10.0,
                        "fit_method": "ransac",
                        "ransac_distance_threshold_m": 0.05,
                        "ransac_max_iterations": 100,
                        "ransac_min_inliers": 20,
                        "p95_percentile": 95.0,
                        "min_points_to_fit": 20,
                        "save_blocks_laz": False,
                    }
                ),
                encoding="utf-8",
            )
            (clip_dir / "calib_gnss_to_lidar_top_ENU.json").write_text(
                json.dumps(
                    {
                        "gnss-to-lidar-top": {
                            "param": {
                                "sensor_calib": {
                                    "data": [
                                        [1.0, 0.0, 0.0, 1.0],
                                        [0.0, 1.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0],
                                        [0.0, 0.0, 0.0, 1.0],
                                    ]
                                }
                            }
                        }
                    }
                ),
                encoding="utf-8",
            )
            (clip_dir / "calib_lidar_top_to_car.json").write_text(
                json.dumps(
                    {
                        "lidar-top-to-car": {
                            "param": {
                                "sensor_calib": {
                                    "data": [
                                        [1.0, 0.0, 0.0, 0.5],
                                        [0.0, 1.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0],
                                        [0.0, 0.0, 0.0, 1.0],
                                    ]
                                }
                            }
                        }
                    }
                ),
                encoding="utf-8",
            )

            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            local_x = np.linspace(-1.5, 1.5, 25)
            local_y = np.linspace(-1.5, 1.5, 25)
            grid_x, grid_y = np.meshgrid(local_x, local_y)
            flat_x = grid_x.reshape(-1)
            flat_y = grid_y.reshape(-1)
            flat_z = 1.0 + 0.01 * flat_x - 0.02 * flat_y
            las.x = np.concatenate((10.0 + flat_x, np.array([20.0])))
            las.y = np.concatenate((5.0 + flat_y, np.array([20.0])))
            las.z = np.concatenate((flat_z, np.array([1.0])))
            las.classification = np.concatenate(
                (np.ones(flat_x.shape[0], dtype=np.uint8), np.array([0], dtype=np.uint8))
            )
            las.write(input_laz)

            with input_pose_csv.open("w", newline="") as handle:
                handle.write("#t,x,y,z,l,l,h,ins_status,pos_type,err,qw,qx,qy,qz\n")
                handle.write("1500,10.0,5.0,1.0,0,0,0,0,0,0,1.0,0.0,0.0,0.0\n")

            exit_code = SCRIPT_MODULE.main(
                [
                    "--input-laz",
                    str(input_laz),
                    "--input-gps-msg",
                    str(input_pose_csv),
                    "--session-path",
                    str(session_path),
                    "--output-dir",
                    str(output_dir),
                    "--config",
                    str(config_path),
                ]
            )
            self.assertEqual(exit_code, 0)

            self.assertFalse((output_dir / "blocks" / "keyframe_1500.laz").exists())
            self.assertTrue((output_dir / "index.csv").exists())
            self.assertTrue((output_dir / "summary.csv").exists())
            self.assertTrue((output_dir / "plane_distance_mean_p95.png").exists())
            self.assertTrue((output_dir / "plane_distance_p95_threshold.png").exists())
            self.assertTrue((output_dir / "plane_distance_thickness_p95_p5.png").exists())


class MergeSummaryReportsTest(unittest.TestCase):

    def test_main_merges_summaries_in_input_order_and_writes_html(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            summary1 = temp_root / "summary_a.csv"
            summary2 = temp_root / "summary_b.csv"
            output_html = temp_root / "report.html"
            summary1.write_text(
                (
                    "session_name,total_row_count,status_ok_count,status_empty_count,"
                    "status_insufficient_points_count,status_fit_failed_count,"
                    "plane_distance_mean_p95_mean,plane_distance_mean_p95_std,"
                    "plane_distance_p95_threshold_mean,plane_distance_p95_threshold_std,"
                    "plane_distance_thickness_p95_p5_mean,plane_distance_thickness_p95_p5_std\n"
                    "session_a,2,2,0,0,0,0.10,0.01,0.20,0.02,0.30,0.03\n"
                ),
                encoding="utf-8",
            )
            summary2.write_text(
                (
                    "session_name,total_row_count,status_ok_count,status_empty_count,"
                    "status_insufficient_points_count,status_fit_failed_count,"
                    "plane_distance_mean_p95_mean,plane_distance_mean_p95_std,"
                    "plane_distance_p95_threshold_mean,plane_distance_p95_threshold_std,"
                    "plane_distance_thickness_p95_p5_mean,plane_distance_thickness_p95_p5_std\n"
                    "session_b,3,3,0,0,0,0.40,0.04,0.50,0.05,0.60,0.06\n"
                ),
                encoding="utf-8",
            )

            exit_code = MERGE_SCRIPT_MODULE.main(
                [
                    "--output-html",
                    str(output_html),
                    str(summary1),
                    str(summary2),
                ]
            )

            self.assertEqual(exit_code, 0)
            html = output_html.read_text(encoding="utf-8")
            self.assertIn("<table", html)
            self.assertIn("session_a", html)
            self.assertIn("session_b", html)
            self.assertLess(html.index("session_a"), html.index("session_b"))
            self.assertIn("plane_distance_mean_p95_mean", html)
            self.assertIn("plane_distance_p95_threshold_mean", html)
            self.assertIn("plane_distance_thickness_p95_p5_mean", html)
            self.assertIn("<svg", html)
            self.assertIn("Generated at:", html)
            self.assertIn("zoukai", html)
            self.assertIn("chart-legend", html)
            self.assertIn("table-wrap watermarked", html)
            self.assertIn("chart-wrap watermarked", html)
            self.assertIn("session_id", html)
            self.assertIn(">m</text>", html)


class BatchExtractIntersectionsTest(unittest.TestCase):

    def test_run_batch_extracts_single_laz_sessions_and_skips_invalid_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_root = Path(temp_dir)
            csv_path = temp_root / "intersection_30.csv"
            dataset_root = temp_root / "datasets"
            output_root = temp_root / "output"
            config_path = temp_root / "local_block_eval.json"
            csv_path.write_text(
                "valid_session\n"
                "multi_laz_session\n"
                "missing_gps_session\n",
                encoding="utf-8",
            )
            config_path.write_text("{}", encoding="utf-8")

            valid_laz_dir = (
                dataset_root
                / "valid_session"
                / "prelabel"
                / "RoadMark_label"
                / "laz"
            )
            valid_laz_dir.mkdir(parents=True)
            (valid_laz_dir / "map.laz").write_text("", encoding="utf-8")
            valid_gps = valid_laz_dir.parent / "gps_msg.txt"
            valid_gps.write_text("", encoding="utf-8")

            multi_laz_dir = (
                dataset_root
                / "multi_laz_session"
                / "prelabel"
                / "RoadMark_label"
                / "laz"
            )
            multi_laz_dir.mkdir(parents=True)
            (multi_laz_dir / "a.laz").write_text("", encoding="utf-8")
            (multi_laz_dir / "b.laz").write_text("", encoding="utf-8")
            (multi_laz_dir.parent / "gps_msg.txt").write_text("", encoding="utf-8")

            missing_gps_laz_dir = (
                dataset_root
                / "missing_gps_session"
                / "prelabel"
                / "RoadMark_label"
                / "laz"
            )
            missing_gps_laz_dir.mkdir(parents=True)
            (missing_gps_laz_dir / "map.laz").write_text("", encoding="utf-8")

            captured_commands = []

            def fake_runner(command):
                captured_commands.append(command)
                if "extract_keyframe_local_blocks.py" in " ".join(command):
                    output_dir = Path(command[command.index("--output-dir") + 1])
                    output_dir.mkdir(parents=True, exist_ok=True)
                    (output_dir / "summary.csv").write_text("", encoding="utf-8")
                return 0

            exit_code = BATCH_SCRIPT_MODULE.run_batch(
                csv_path=csv_path,
                dataset_root=dataset_root,
                output_root=output_root,
                config_path=config_path,
                runner=fake_runner,
                report_html=temp_root / "report.html",
            )

            self.assertEqual(exit_code, 0)
            self.assertEqual(len(captured_commands), 2)
            command = captured_commands[0]
            self.assertIn("--input-laz", command)
            self.assertIn(str(valid_laz_dir / "map.laz"), command)
            self.assertIn("--input-gps-msg", command)
            self.assertIn(str(valid_gps), command)
            self.assertIn("--session-path", command)
            self.assertIn(str(dataset_root / "valid_session"), command)
            self.assertIn("--output-dir", command)
            self.assertIn(str(output_root / "valid_session"), command)
            self.assertIn("--config", command)
            self.assertIn(str(config_path), command)
            merge_command = captured_commands[1]
            self.assertIn("merge_summary_reports.py", " ".join(merge_command))
            self.assertIn(str(output_root / "valid_session" / "summary.csv"), merge_command)
            self.assertIn(str(temp_root / "report.html"), merge_command)


if __name__ == "__main__":
    unittest.main()
