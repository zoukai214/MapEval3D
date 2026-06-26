import json
from pathlib import Path
from typing import Dict
from typing import List
from typing import Sequence

import numpy as np


def extract_clip_start_timestamp_ms(clip_name: str) -> int:
    parts = clip_name.split("_")
    if len(parts) < 2:
        raise ValueError(f"Invalid clip name: {clip_name}")
    return int(parts[1]) * 1000


def _load_extrinsic_matrix(calib_path: Path, top_level_key: str) -> np.ndarray:
    calib_json = json.loads(calib_path.read_text(encoding="utf-8"))
    try:
        matrix_data = calib_json[top_level_key]["param"]["sensor_calib"]["data"]
    except KeyError as exc:
        raise ValueError(f"Invalid calibration format: {calib_path}") from exc

    matrix = np.asarray(matrix_data, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"Calibration matrix must be 4x4: {calib_path}")
    return matrix


def load_session_calibrations(session_path: Path) -> List[Dict[str, object]]:
    clip_infos: List[Dict[str, object]] = []
    for clip_dir in sorted(path for path in session_path.iterdir() if path.is_dir()):
        try:
            start_timestamp_ms = extract_clip_start_timestamp_ms(clip_dir.name)
        except (ValueError, IndexError):
            continue
        calib_dir = clip_dir / "calib_extract"
        gnss_to_lidar_path = calib_dir / "calib_gnss_to_lidar_top_ENU.json"
        lidar_to_car_path = calib_dir / "calib_lidar_top_to_car.json"
        if not gnss_to_lidar_path.exists():
            raise ValueError(f"Missing calibration file: {gnss_to_lidar_path}")
        if not lidar_to_car_path.exists():
            raise ValueError(f"Missing calibration file: {lidar_to_car_path}")

        clip_infos.append(
            {
                "clip_name": clip_dir.name,
                "start_timestamp_ms": start_timestamp_ms,
                "gnss_to_lidar_matrix": _load_extrinsic_matrix(
                    gnss_to_lidar_path,
                    "gnss-to-lidar-top",
                ),
                "lidar_to_car_matrix": _load_extrinsic_matrix(
                    lidar_to_car_path,
                    "lidar-top-to-car",
                ),
            }
        )

    if not clip_infos:
        raise ValueError(f"No clip directories found in session_path: {session_path}")
    clip_infos.sort(key=lambda info: int(info["start_timestamp_ms"]))
    return clip_infos


def find_clip_for_timestamp(
    clip_infos: Sequence[Dict[str, object]],
    timestamp_ms: int,
) -> Dict[str, object]:
    for index, clip_info in enumerate(clip_infos):
        start_timestamp_ms = int(clip_info["start_timestamp_ms"])
        next_start_timestamp_ms = None
        if index + 1 < len(clip_infos):
            next_start_timestamp_ms = int(clip_infos[index + 1]["start_timestamp_ms"])
        if timestamp_ms < start_timestamp_ms:
            continue
        if next_start_timestamp_ms is None or timestamp_ms < next_start_timestamp_ms:
            return dict(clip_info)
    raise ValueError(f"No clip found for timestamp_ms={timestamp_ms}")
