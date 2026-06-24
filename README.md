# MapEval3D

`MapEval3D` 用于从整张地面点云地图中按关键帧切出局部地面点云块，并对每个局部块做平面拟合与地面厚度评估。

当前实现能力：

- 读取单个 `laz` 地图文件
- 读取 `gps_msg.txt`
- 按轨迹距离间隔选取关键帧
- 在关键帧局部坐标系下切出固定大小的地面点云块
- 使用 `RANSAC` 拟合局部地面平面
- 输出局部地面质量评估表格
- 输出两张折线图

## 目录结构

主要目录：

- `config/`
- `mapeval3d/`
- `tests/`
- `tools/`

主要脚本：

- `tools/extract_keyframe_local_blocks.py`

默认配置：

- `config/local_block_eval.json`

## 输入说明

脚本运行需要以下输入：

- 一个带 `classification` 字段的 `laz` 文件
- 一个 `gps_msg.txt` 文件

当前实现默认把：

- `classification == 1`

视为地面点。

## 运行命令

基本命令：

```bash
python3 tools/extract_keyframe_local_blocks.py \
  --input-laz /path/to/input.laz \
  --input-gps-msg /path/to/gps_msg.txt \
  --output-dir /path/to/output_dir \
  --config config/local_block_eval.json
```

真实数据示例：

```bash
python3 tools/extract_keyframe_local_blocks.py \
  --input-laz /mnt/workspace/test_data/output/prelabel/RoadMark_label/laz/1762669099099.laz \
  --input-gps-msg /mnt/workspace/test_data/output/prelabel/RoadMark_label/gps_msg.txt \
  --output-dir /mnt/workspace/mapping_ground/MapEval3D/output/real_run_eval \
  --config /mnt/workspace/mapping_ground/MapEval3D/config/local_block_eval.json
```

## 命令行参数说明

- `--input-laz`
  输入点云地图文件路径。

- `--input-gps-msg`
  输入轨迹与位姿文件 `gps_msg.txt` 路径。

- `--output-dir`
  输出目录。

- `--config`
  配置文件路径，使用 JSON 格式。

## 配置文件说明

默认配置文件内容如下：

```json
{
  "window_size_x_m": 4.0,
  "window_size_y_m": 4.0,
  "distance_interval_m": 4.0,
  "fit_method": "ransac",
  "ransac_distance_threshold_m": 0.05,
  "ransac_max_iterations": 200,
  "ransac_min_inliers": 20,
  "p95_percentile": 95.0,
  "min_points_to_fit": 30
}
```

各字段含义：

- `window_size_x_m`
  局部块在关键帧坐标系下 `x` 方向长度，单位米。

- `window_size_y_m`
  局部块在关键帧坐标系下 `y` 方向长度，单位米。

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

## 输出说明

运行完成后，输出目录下通常包含：

- `index.csv`
- `blocks/`
- `plane_distance_mean_p95.png`
- `plane_distance_thickness_p95_p5.png`

含义：

- `index.csv`
  每个关键帧局部块对应的一行评估结果。

- `blocks/`
  每个关键帧对应的局部地面点云块，文件名为 `keyframe_<timestamp_ms>.laz`。

- `plane_distance_mean_p95.png`
  以 `id` 为横轴、`plane_distance_mean_p95` 为纵轴的折线图。

- `plane_distance_thickness_p95_p5.png`
  以 `id` 为横轴、`plane_distance_thickness_p95_p5` 为纵轴的折线图。

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

## 两张折线图说明

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

## 当前未上传目录

仓库当前默认忽略以下目录或文件类型：

- `docs/`
- `output/`
- `__pycache__/`
- `.ipynb_checkpoints/`
- `*.pyc`
