# MapEval3D

`MapEval3D` 用于从整张地面点云地图中按关键帧切出局部地面点云块，并对每个局部块做平面拟合与地面厚度评估。

当前实现能力：

- 读取一个或多个 `laz` 地图文件
- 读取 `gps_msg.txt`
- 读取原始数据 `session_path`
- 按轨迹距离间隔选取关键帧
- 按 `gps_msg.txt` 每帧时间戳匹配所属 `clip`
- 从每个 `clip` 读取 `GNSS -> LiDAR` 与 `LiDAR -> Car` 外参
- 将局部点云块转到关键帧自车坐标系
- 使用 `RANSAC` 拟合局部地面平面
- 输出局部地面质量评估表格
- 输出三张折线图

## 目录结构

主要目录：

- `config/`
- `mapeval3d/`
- `tests/`
- `tools/`

主要脚本：

- `tools/extract_keyframe_local_blocks.py`
- `tools/merge_summary_reports.py`

默认配置：

- `config/local_block_eval.json`

## 输入说明

脚本运行需要以下输入：

- 一个导出根目录，例如 `/path/to/output_lio_x`
- 一个原始数据 `session_path`

其中导出根目录下默认需要包含：

- `prelabel/RoadMark_label/gps_msg.txt`
- `prelabel/RoadMark_label/laz/*.laz`

`session_path` 下需要包含多个 `clip` 目录。每个 `clip` 目录下都需要有：

- `calib_extract/calib_gnss_to_lidar_top_ENU.json`
- `calib_extract/calib_lidar_top_to_car.json`

当前实现默认把：

- `classification == 1`

视为地面点。

## 运行命令

基本命令：

```bash
python3 tools/extract_keyframe_local_blocks.py \
  --input-root /path/to/output_lio_x \
  --session-path /path/to/session_path \
  --output-dir /path/to/output_dir \
  --config config/local_block_eval.json
```

真实数据示例：

```bash
python3 tools/extract_keyframe_local_blocks.py \
  --input-root /mnt/workspace/test_data/output \
  --session-path /mnt/workspace/test_data/input_raw_data \
  --output-dir /mnt/workspace/mapping_ground/MapEval3D/output/real_run_eval \
  --config /mnt/workspace/mapping_ground/MapEval3D/config/local_block_eval.json
```

合并多个 `summary.csv` 生成 HTML 报告：

```bash
python3 tools/merge_summary_reports.py \
  --output-html /path/to/summary_report.html \
  /path/to/run_1/summary.csv \
  /path/to/run_2/summary.csv \
  /path/to/run_3/summary.csv
```

## 命令行参数说明

- `--input-root`
  输入导出根目录路径。程序会自动读取 `prelabel/RoadMark_label/gps_msg.txt` 与 `prelabel/RoadMark_label/laz/*.laz`。

- `--session-path`
  原始数据目录路径。程序会扫描该目录下的 `clip`，并按关键帧时间戳匹配对应标定。

- `--output-dir`
  输出目录。

- `--config`
  配置文件路径，使用 JSON 格式。

`tools/merge_summary_reports.py` 参数说明：

- `--output-html`
  输出的 HTML 报告路径。

- `summary.csv` 列表
  一个或多个 `summary.csv` 文件，按命令行输入顺序合并。输出表格第一列 `id` 即该输入顺序，折线图横轴也使用该顺序编号。

## 配置文件说明

默认配置文件内容如下：

```json
{
  "window_size_x_m": 4.0,
  "window_size_y_m": 4.0,
  "window_size_z_min_m": -2.0,
  "window_size_z_max_m": 2.0,
  "distance_interval_m": 4.0,
  "fit_method": "ransac",
  "ransac_distance_threshold_m": 0.05,
  "ransac_max_iterations": 200,
  "ransac_min_inliers": 20,
  "p95_percentile": 95.0,
  "min_points_to_fit": 30,
  "save_blocks_laz": true
}
```

各字段含义：

- `window_size_x_m`
  局部块在关键帧坐标系下 `x` 方向长度，单位米。

- `window_size_y_m`
  局部块在关键帧坐标系下 `y` 方向长度，单位米。

- `window_size_z_min_m`
  局部块在关键帧自车坐标系下允许保留的最小 `z` 值，单位米。用于过滤高架桥、上下层道路等同一 `x/y` 范围内但高度差较大的地面点。

- `window_size_z_max_m`
  局部块在关键帧自车坐标系下允许保留的最大 `z` 值，单位米。只有 `window_size_z_min_m <= z <= window_size_z_max_m` 的点会进入局部块。

- `distance_interval_m`
  沿轨迹选取关键帧的距离间隔，单位米。

- `fit_method`
  平面拟合方法。当前仅支持 `ransac`。

- `ransac_distance_threshold_m`
  `RANSAC` 内点判断阈值。若点到候选平面的绝对距离不超过该值，则判为 `RANSAC` 内点。

- `ransac_max_iterations`
  `RANSAC` 最大迭代次数。

- `ransac_min_inliers`
  一个候选平面被接受时要求的最少内点数。

- `p95_percentile`
  绝对距离统计时使用的百分位阈值。当前通常设置为 `95.0`。

- `min_points_to_fit`
  一个局部块允许做平面拟合的最小点数。

- `save_blocks_laz`
  是否保存每个关键帧对应的局部地面点云块。设置为 `true` 时输出 `blocks/*.laz`，设置为 `false` 时只输出评估表格和折线图，不保存小块点云文件。默认值为 `true`。

## 坐标变换链路

当前输出点云块和评估指标都在关键帧自车坐标系下计算。

每个关键帧的坐标变换链路为：

```text
world -> keyframe_gnss -> keyframe_lidar -> keyframe_car
```

具体来源：

- `world -> keyframe_gnss`
  来自 `gps_msg.txt` 当前帧位姿

- `keyframe_gnss -> keyframe_lidar`
  来自所属 `clip/calib_extract/calib_gnss_to_lidar_top_ENU.json`

- `keyframe_lidar -> keyframe_car`
  来自所属 `clip/calib_extract/calib_lidar_top_to_car.json`

说明：

- 一个 `gps_msg.txt` 可能由多个 `clip` 拼接而成
- 不同关键帧可能对应不同 `clip`
- 程序按每一帧 `timestamp_ms` 匹配所属 `clip`

## clip 匹配规则

程序从 `session_path` 下每个 `clip` 目录名提取起始时间戳，例如：

- `GACRT025_1762669096_MT`
- `GACRT025_1762669116_MT`

然后按时间排序，对某一帧 `timestamp_ms` 使用以下区间匹配：

```text
clip_i_start <= timestamp_ms < clip_{i+1}_start
```

最后一个 `clip` 的时间区间向后开放。

以下情况会直接报错并终止：

- 某个 `clip` 缺少任一标定文件
- 标定文件格式非法
- 某一帧时间戳找不到所属 `clip`

## 输出说明

运行完成后，输出目录下通常包含：

- `index.csv`
- `summary.csv`
- `blocks/`
- `plane_distance_mean_p95.png`
- `plane_distance_p95_threshold.png`
- `plane_distance_thickness_p95_p5.png`

含义：

- `index.csv`
  每个关键帧局部块对应的一行评估结果。

- `summary.csv`
  全局汇总结果。第一列记录 `session_name`，其值来自 `--session-path` 最后一级目录名；随后统计各类 `status` 数量，并仅使用 `status == "ok"` 的行计算三个核心指标的均值和标准差。

- `blocks/`
  每个关键帧对应的局部地面点云块，文件名为 `keyframe_<timestamp_ms>.laz`。当前块坐标系为关键帧自车坐标系。局部块同时按 `x/y` 矩形和 `z` 高度范围裁剪。仅当 `save_blocks_laz` 为 `true` 时输出。

- `plane_distance_mean_p95.png`
  以 `id` 为横轴、`plane_distance_mean_p95` 为纵轴的折线图。

- `plane_distance_p95_threshold.png`
  以 `id` 为横轴、`plane_distance_p95_threshold` 为纵轴的折线图。

- `plane_distance_thickness_p95_p5.png`
  以 `id` 为横轴、`plane_distance_thickness_p95_p5` 为纵轴的折线图。

`tools/merge_summary_reports.py` 输出内容：

- `summary_report.html`
  单文件 HTML 报告，包含两部分：
  1. 按输入顺序合并后的 `summary.csv` 表格，第一列增加顺序 `id`
  2. 一张三折线图，横轴为该 `id`，三条线分别为 `plane_distance_mean_p95_mean`、`plane_distance_p95_threshold_mean`、`plane_distance_thickness_p95_p5_mean`

## 输出表格 index.csv 各列说明

- `id`
  被选中参与评估的关键帧编号，从 `0` 开始递增。两张折线图横轴也使用该字段。

- `timestamp_ms`
  当前关键帧的原始时间戳，单位毫秒。

- `status`
  当前关键帧局部块的处理状态。
  可能值：
  - `ok`
  - `empty`
  - `insufficient_points`
  - `fit_failed`

- `point_count`
  当前局部块中的地面点数量。

- `plane_distance_mean_p95`
  基于点到平面的绝对距离计算。先对绝对距离分布取 `P95` 阈值，再对阈值内点的绝对距离求均值。表示主体地面平均残差水平。

- `plane_distance_variance_p95`
  基于点到平面的绝对距离计算。使用与 `plane_distance_mean_p95` 相同的一批 `P95` 内点，对绝对距离求方差。表示主体地面残差分布稳定性。

- `plane_distance_p95_threshold`
  点到平面的绝对距离分布 `P95` 阈值，单位米。

- `plane_distance_thickness_p95_p5`
  基于点到平面的带符号距离计算。对带符号距离分布取 `P95` 与 `P5`，再用 `P95 - P5` 表示局部地面厚度跨度，单位米。

- `plane_inlier_count_p95`
  绝对距离不超过 `plane_distance_p95_threshold` 的点数。该批点参与 `plane_distance_mean_p95` 和 `plane_distance_variance_p95` 计算。

- `ransac_inlier_count`
  `RANSAC` 拟合阶段的内点数。定义为点到候选平面的绝对距离小于等于 `ransac_distance_threshold_m` 的点数。

- `ransac_outlier_ratio`
  `RANSAC` 外点比例，计算方式为：

```text
(point_count - ransac_inlier_count) / point_count
```

## 输出表格 summary.csv 各列说明

- `session_name`
  当前输入 `--session-path` 的最后一级目录名。例如输入 `/mnt/oss/gacrnd-annotation/ruqi/upm/datasets/INTERSECTION_ws07zb1ybjwy/` 时，该列为 `INTERSECTION_ws07zb1ybjwy`。

- `total_row_count`
  `index.csv` 中的总行数。

- `status_ok_count`
  `status == "ok"` 的关键帧数量。

- `status_empty_count`
  `status == "empty"` 的关键帧数量。

- `status_insufficient_points_count`
  `status == "insufficient_points"` 的关键帧数量。

- `status_fit_failed_count`
  `status == "fit_failed"` 的关键帧数量。

- `plane_distance_mean_p95_mean` / `plane_distance_mean_p95_std`
  仅基于 `status == "ok"` 的行，统计 `plane_distance_mean_p95` 的均值和标准差。

- `plane_distance_p95_threshold_mean` / `plane_distance_p95_threshold_std`
  仅基于 `status == "ok"` 的行，统计 `plane_distance_p95_threshold` 的均值和标准差。

- `plane_distance_thickness_p95_p5_mean` / `plane_distance_thickness_p95_p5_std`
  仅基于 `status == "ok"` 的行，统计 `plane_distance_thickness_p95_p5` 的均值和标准差。

## 三张折线图说明

### 1. plane_distance_mean_p95.png

横轴：

- `id`

纵轴：

- `plane_distance_mean_p95`

含义：

- 用于观察沿轨迹方向上，主体地面平均残差是否出现升高。

一般解释：

- 值越小，说明主体地面越平整
- 局部尖峰，说明某些关键帧附近地面残差显著增大

### 2. plane_distance_thickness_p95_p5.png

横轴：

- `id`

纵轴：

- `plane_distance_thickness_p95_p5`

含义：

- 用于观察沿轨迹方向上，局部地面在法向方向上的厚度变化。

一般解释：

- 值越小，说明地面上下厚度越薄
- 值越大，可能表示厚化、双层、重影或局部起伏异常

### 3. plane_distance_p95_threshold.png

横轴：

- `id`

纵轴：

- `plane_distance_p95_threshold`

含义：

- 用于观察每个关键帧局部块中，主体 `95%` 点到拟合平面的绝对距离上边界。

一般解释：

- 值越小，说明大多数点更贴近平面
- 值越大，说明主体点分布更厚或更散

## 指标计算规则摘要

### 绝对距离类指标

以下指标基于点到平面的绝对距离：

- `plane_distance_mean_p95`
- `plane_distance_variance_p95`
- `plane_distance_p95_threshold`
- `plane_inlier_count_p95`
- `ransac_inlier_count`
- `ransac_outlier_ratio`

### 带符号距离类指标

以下指标基于点到平面的带符号距离：

- `plane_distance_thickness_p95_p5`

说明：

- 厚度指标之所以使用带符号距离，是为了表达“平面上方到平面下方”的厚度跨度
- 若用绝对距离计算 `P95 - P5`，会丢掉点位于平面哪一侧的信息

## 测试

运行单元测试：

```bash
python3 -m unittest tests.test_extract_keyframe_local_blocks -v
```

真实数据验证示例：

```bash
python3 tools/extract_keyframe_local_blocks.py \
  --input-laz /mnt/workspace/test_data/output/prelabel/RoadMark_label/laz/1762669099099.laz \
  --input-gps-msg /mnt/workspace/test_data/output/prelabel/RoadMark_label/gps_msg.txt \
  --session-path /mnt/workspace/test_data/input_raw_data \
  --output-dir /mnt/workspace/mapping_ground/MapEval3D/output/real_run_eval \
  --config /mnt/workspace/mapping_ground/MapEval3D/config/local_block_eval.json
```

## 当前未上传目录

仓库当前默认忽略以下目录或文件类型：

- `docs/`
- `output/`
- `__pycache__/`
- `.ipynb_checkpoints/`
- `*.pyc`
