# 地形分析改进（阶段 0–3）改造记录

## User Intent

在 `terrain_dev` 分支实现已商定的地形改进方案阶段 0–3，不合并。每阶段 `colcon build --packages-select ma_nav` 通过后本地 commit（用户本次明确授权构建与本地 commit，不 push / merge）。20°/23° 坡上高地功能暂不确定，max_slope 默认 25° 做成 yaml 参数。输入点云已是世界系。

## Scope

- map 模块：`ProbMap` 射线起点与遗忘、`TerrainAnalyzer`、新增 `GroundLayer`、`MaMap` 接线、`config.hpp` 参数、`config/map.yaml`。
- test：新增离线回放工具 `test_terrain_replay` 与 bag 导出脚本。

## Out of Scope

- planner / controller / ros2 节点、topic、frame、QoS 未改。
- 建图模式遗忘（`forget_threshold` 仍为 -1）。
- 实车 / Gazebo 联调未做。

## Explorer Findings

### Files inspected

`prob_map.cpp/.h`、`rog_map.cpp`、`config.hpp`、`ma_map.hpp`、`terrain_analysis.cpp/.hpp`、`ros2_node.cpp`（odom/cloud 回调时序）、`config/map.yaml`、`config/planner.yaml`。

### Active logic path

`ros2_node map_callback → MaMap::update_cloud → update_map → ROGMap::updateProbMap → update_terrain → TerrainAnalyzer::analyze → 2D occupancy`。

### Data flow

ROG 射线从 `pose.first`（base_odom，贴地）出发，掠射角下会把车周地面体素擦掉，列内最低体素就不再是地面。旧判据（中值地面 + 梯度 + 邻域最低）依赖列内最低体素。

### Risk notes

- `sensor_offset` z=0.194 取自 bag 中 `/lio/odom` 与 `/lio/base_odom` 高差，tf_static 写 0.15，**未核实以哪个为准**。
- 足迹种子高度：bag 统计 base_odom z 与车周 0.5–1 m 地面的差值在 -0.06～+0.18 之间跳动，不是常数。因此种子优先取足迹外一圈真实地面中位数，`footprint_ground_offset` 只作兜底（默认 0，未核实）。

### Recommended modification boundary

新逻辑全部有 yaml 开关；`ground_layer.enable=false` 且 `reach.enable=false`、`sensor_offset=[0,0,0]`、`nav_forget_threshold<=0` 时与旧行为一致。

## Modifier Changes

### Files changed

| commit | 内容 |
|---|---|
| 13a1513 | `src/test/src/test_terrain_replay.cpp`、`src/test/scripts/terrain_replay_extract.py`、`src/test/CMakeLists.txt` |
| 03a750f | 阶段 0：`raycasting/sensor_offset`；`terrain/obstacle_band`、`ground_jump_thresh`、`ground_min_radius` 移到 yaml（值不变） |
| 0a8fe82 | 阶段 1：`ground_layer.hpp/.cpp`；`analyze` 中列内最低高于持久地面或列为空时以持久地面为准 |
| 778d6af | 阶段 2：`terrain/reach/*`；上下沿、5×5 平面陡坡、从车所在格可达生长；只有可达格给空闲证据；TRAV 格不因地面判据判障碍 |
| 21c3332 | 阶段 3：`nav_forget_threshold: 2.0`、`forget_keep_radius: 1.5`；`set_mapping_model` 按模式切换遗忘阈值 |

### Key changes

- 地面层：每帧每格最低点，σ = 0.01 + 0.003·距离，方差加权融合；下降 2 帧、上升 3 帧一致才生效；高于地面超过 `rise_max`(0.24) 当悬空物；最低点下方 3 个已知空闲体素视为悬空下沿，不当地面；下行射线给未观测格射线上界，射线从已记地面下方穿过且该格 5 帧未命中、非走过格时撤销地面。
- 种子地面（启动 1.5 m 盲区、足迹 0.20 m）不参与上下沿与平面拟合，生长时视为可跨越。
- 下沿判定 `drop_edge_enable` 默认 **false**（用户选择）。

### Behavior preserved

- 悬空杆、限高杆判据、空列空闲证据逻辑保留，可达模式下空列证据额外要求可达。
- 建图模式遗忘仍关闭。

### Behavior intentionally adjusted

- 射线起点上移 0.194 m。
- 地面高度来源改为持久地面层；陡坡判据由 `steep_threshold` 梯度改为 25° 坡度阈值（可达模式下 `steep_threshold` 不再生效）。
- 导航模式 ROG 占据 2 s 遗忘。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查（新增参数读取名与 yaml 一致）
- [x] XML / launch / yaml 检查（未改 launch）
- [x] 离线回放 `rosbag2_2026_10_23-16_26_25`，408 帧
- [x] 构建已获用户明确许可：`colcon build --packages-select ma_nav --cmake-args -DCMAKE_BUILD_TYPE=Release`，每阶段通过

回放结果（`traversed_obs` = 车走过的格中最终被判障碍的数量，应为 0）：

| 配置 | 建图 obs_final | 建图 traversed_obs | 导航 obs_final | 足迹内有障碍的帧 | update_map 中位 ms |
|---|---|---|---|---|---|
| 旧（offset=0） | 2293 | 0 | 897 | 10 | 31 |
| 阶段 0 | 3048 | 1 | 1506 | 0 | 33 |
| 阶段 1 | 3250 | 1 | 1607 | 0 | 45 |
| 阶段 2 | 5004 | 0 | 2054 | 0 | 40 |
| 阶段 3（实时节拍） | 5004 | 0 | 1872 | 0 | 42 |

### Issues found

- 阶段 2 障碍数约为旧方案 2 倍，新增格集中在 x 3–6、y -6.6～-7.8 的高台/坡区。其中有一部分是持久地面里 ±0.1 m 跳变的"台阶"，是否全部为真台沿**未用真值核实**。
- bag 里没有 20°/23° 坡道段，25° 坡度判据未经实测验证。
- 回放只测了一个 bag；update_map 耗时比旧方案多约 9 ms。

### Final result

NEEDS_FIX：需要实车或 Gazebo 在 20°/23° 坡和高台边缘验证，确认阶段 2 新增障碍格是否为误检后再定参数。
