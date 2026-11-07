# 地图侧算法架构图文档 改造记录

> 本记录为**文档新增**任务，未修改任何源代码、配置、launch 或参数。

## User Intent

用户要求把项目架构拆成「地图」「导航」两部分，先交付**地图算法架构图**，并指定写入 `docs/架构图/` 目录。
约束（来自 AGENTS.md）：

- 最小改动，不重构、不改话题 / 参数 / 模块边界。
- 只读事实作为图与表的依据，不把未验证推断写成确定事实。
- 不执行构建、不执行任何 Git 写操作。

## Scope

- 新增 `docs/架构图/地图算法架构图.md`。
- 内容范围：`map` 库（CMake target `3d_occ_map`）的子系统划分、每帧数据流、3D 概率图内部结构、地面层与地形判定、2D 栅格与双缓冲、尺度窗口、对外接口、架构评价与风险。
- 新增本记录文件。

## Out of Scope

- 导航侧（path_planning / traj_optimize / controller / replan / ros2 调度）架构图：另文，本次不做。
- 不修改 `docs/架构.md`（用户已有顶层架构草图）。
- 不修改 `docs/nav/` 那套 9 文件契约文档，不重跑 `docs/tools` 校验流水线。
- 不修改任何 `src/`、`config/`、`launch/` 文件。

## Explorer Findings

### Files inspected

- `src/map/include/map/ma_map.hpp`、`src/map/src/grid_map.cpp`、`src/map/include/map/grid_map.hpp`
- `src/map/include/map/3d_occ_map/{rog_map.h,prob_map.h,sliding_map.h,counter_map.h,inf_map.h,esdf_map.h,free_cnt_map.h,raycaster.h,config.hpp}`
- `src/map/src/3d_occ_map/prob_map.cpp`、`src/map/src/terrain_analysis.cpp`、`src/map/src/ground_layer.cpp`
- `src/ros2/src/ros2_node.cpp`、`src/ros2/include/ros2/ros2_node.h`
- `config/map.yaml`、`config/planner.yaml`、`CMakeLists.txt`、`src/*/CMakeLists.txt`
- `docs/架构.md`（用户新建）、`docs/MA_Nav_设计联动与实机部署问题审计.md`（风险条目对照）

### Active logic path

点云回调 → `MaMap::update_cloud`（缓存 + odom 新鲜度过滤）→ `update_map()`：
`ROGMap::updateProbMap`（滑窗 / 射线 / 概率批量更新 / 遗忘）→ `GroundLayer::update` →
`BoxSearch(OCCUPIED)` → `TerrainAnalyzer::analyze` → 2D 动态层重建或回写、
与 `global_2d_occ_` 取 max → `GridMap::setMap`（有变化才 `updateESDF`）→ 深拷贝快照 swap。

### Data flow

见文档 §2 时序图与 §7 接口契约；核心边界是「map 库不依赖 ROS，对外只暴露只读快照 + 查询 API」。

### Risk notes

- 2D 层无 unknown 态；导航模式时间遗忘（2 s）+ 车周 1.5 m 不遗忘，强依赖先验地图。
- 地形分析窗口（10×10 m，`local_update_box`）小于 2D 图（28×20 m），窗口外行为由建图/导航模式决定。
- `config/map.yaml:28` 注释「禁用滑动窗口」与值 `enable: true` 不一致（配置漂移）。
- `visualization.range [100,60,4]` 被 `update_terrain` 当作取点范围，实际被局部图裁剪，参数名与语义脱节。
- 以上均按【事实】/【推断】在文档中标注，不写成确定结论。

### Recommended modification boundary

仅新增文档；不改代码、配置与既有文档。

## Modifier Changes

### Files changed

- 新增 `docs/架构图/地图算法架构图.md`（约 350 行，含 5 张 mermaid 图与 6 张表）。
- 新增 `docs/ai_refactor_records/20261105_map_architecture_diagram_doc.md`（本文件）。

### Key changes

- 输出地图侧一页总图、子系统职责表、每帧时序图、3D 概率栈类图、地形判定决策流、2D 双缓冲图、尺度窗口表、接口契约表、架构评价与风险表、证据锚点索引。
- 所有行号锚点与 `config/map.yaml` 键已按当前 HEAD `006e7b2` 核对。

### Behavior preserved

未改动任何可执行逻辑、参数、话题、模块边界。

### Behavior intentionally adjusted

无。

### Notes

- 文档按「事实 / 推断 / 待确认」约定标注；风险级别参考既有审计文档，不新增未经核实的实机结论。
- 导航侧架构图未在本次范围内。

## Auditor Review

### Checks performed

- [x] 关键路径检查：地图写路径与对外接口逐个对照源码
- [x] diff 检查：`git status` 仅新增 2 个未跟踪文件
- [x] 链接检查：文档内 19 个相对链接全部可解析，markdown 代码围栏成对
- [x] 行号锚点抽查：`ma_map.hpp` / `prob_map.cpp` / `grid_map.cpp|hpp` / `terrain_analysis.cpp` / `config/map.yaml` 行号已修正
- [x] XML / launch / yaml 检查：未修改，无需改动
- [x] 用户允许范围内的测试或静态检查：仅 markdown 结构与链接静态校验
- [x] 如需构建，已取得用户明确许可：**未构建**

### Issues found

- 初稿存在 5 处行号/参数区间偏差（`update_map` 结束行、`p_hit..p_max` 区间、射线起点行、双缓冲区间、语义区间），已在提交前修正。
- 未发现与源码冲突的结论。

### Final result

PASS
