import json
from pathlib import Path
from typing import Any
from typing import Dict


DEFAULT_CONFIG = {
    "fit_method": "ransac",
    "p95_percentile": 95.0,
}

REQUIRED_CONFIG_KEYS = {
    "window_size_x_m",
    "window_size_y_m",
    "distance_interval_m",
    "ransac_distance_threshold_m",
    "ransac_max_iterations",
    "ransac_min_inliers",
    "min_points_to_fit",
}


def _as_positive_float(config: Dict[str, Any], key: str) -> float:
    value = float(config[key])
    if value <= 0.0:
        raise ValueError(f"{key} must be positive.")
    return value


def _as_positive_int(config: Dict[str, Any], key: str) -> int:
    value = int(config[key])
    if value <= 0:
        raise ValueError(f"{key} must be positive.")
    return value


def load_eval_config(config_path: Path) -> Dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as handle:
        raw_config = json.load(handle)

    merged_config = {**DEFAULT_CONFIG, **raw_config}
    missing_keys = REQUIRED_CONFIG_KEYS - merged_config.keys()
    if missing_keys:
        raise ValueError(f"Missing config keys: {sorted(missing_keys)}")

    fit_method = str(merged_config["fit_method"]).lower()
    if fit_method != "ransac":
        raise ValueError("fit_method must be 'ransac'.")

    p95_percentile = float(merged_config["p95_percentile"])
    if not 0.0 < p95_percentile <= 100.0:
        raise ValueError("p95_percentile must be in (0, 100].")

    return {
        "window_size_x_m": _as_positive_float(merged_config, "window_size_x_m"),
        "window_size_y_m": _as_positive_float(merged_config, "window_size_y_m"),
        "distance_interval_m": _as_positive_float(merged_config, "distance_interval_m"),
        "fit_method": fit_method,
        "ransac_distance_threshold_m": _as_positive_float(
            merged_config,
            "ransac_distance_threshold_m",
        ),
        "ransac_max_iterations": _as_positive_int(merged_config, "ransac_max_iterations"),
        "ransac_min_inliers": _as_positive_int(merged_config, "ransac_min_inliers"),
        "p95_percentile": p95_percentile,
        "min_points_to_fit": _as_positive_int(merged_config, "min_points_to_fit"),
    }
