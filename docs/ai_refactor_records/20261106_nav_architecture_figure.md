# 导航算法架构图 改造记录

> 本记录为**文档图片资产新增**任务，未修改任何源代码、配置、launch 或参数。

## User Intent

用户要求读项目后生成 **nav 部分（不含 map）的算法思路架构**，参照 `docs/架构图/` 既有地图架构图风格：

- 先出**整体 nav 架构**，不含算法细节（例如不含重规划 FSM、重规划判据的具体实现）。
- 简洁、美观。
- **不含特定数值**（不写节拍、阈值、分辨率等）。
- **不含话题名**（不写 `/cmd_vel_chassis`、`/goal_pose` 等）。

约束（AGENTS.md）：不改比赛主逻辑、参数、话题、模块边界；不把未验证推断写成事实；不执行构建；不执行任何 Git 写操作。

## Scope

- 新增 `docs/架构图/nav_architecture.svg`（手排矢量源，1720 × 640）。
- 新增 `docs/架构图/nav_architecture.png`（2× 渲染，3440 × 1280）。
- 新增 `docs/架构图/导航算法架构图.md`（图注 + 分层职责 + 架构约束）。
- 新增本记录。

## Out of Scope

- 不修改地图侧既有产物：`map_architecture.svg/png`、`地图算法架构图.svg/png/.md`、`算法模块架构图.svg/png`。
- 不修改 `docs/nav/` 契约文档与 `docs/nav/算法思路/` 算法文档。
- 不改任何 `src/`、`config/`、`launch/`、`CMakeLists.txt` 文件。
- 不展开算法细节（搜索算法、后处理公式、优化代价项、MPC 权重、重规划触发判据）。

## Explorer Findings

### Files inspected

- `docs/nav/算法思路/planner/01-导航框架与数据流.md`、`06-重规划策略.md`、`docs/nav/算法思路/README.md`
- `docs/架构图/map_architecture.svg`、`地图算法架构图.md`、`算法模块架构图.svg`（提取文字版式，作为风格基准）
- 源码：`src/planner/replan/include/replan/fsm_replanner.h`、`src/planner/path_planning/include/path_planning/path_planning.hpp`、`post_processing.h`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h`、`src/planner/controller/include/controller/traj_interface.hpp`、`mpc.h`、`yaw_plan.hpp`、`src/ros2/include/ros2/ros2_node.h`
- 配置与构建：`CMakeLists.txt`、`src/planner/path_planning/CMakeLists.txt`、`config/planner.yaml`

### Active logic path

主链（与 `docs/nav/算法思路/planner/01` 一致）：`目标/状态/地图快照/语义` → `FsmReplan 编排` → `JPS/A* 搜索` → `后处理（时间参数化）` → `MA-Spline 优化` → `轨迹接口 + MPC` → `world→body` → `速度指令 / 折叠事件 / 导航状态`。

### Data flow

规划侧与地图侧的唯一耦合点是只读栅格快照（含距离场与语义）；控制器不接触地图，只接触轨迹接口。编排层与求解层之间以「规划任务 / 结果」为界。

### Risk notes

- `target_tracking/`（目标估计、视点选择）与 `yaw_plan.hpp` 虽有源码，但当前**无外部调用点**（`grep` 仅命中自身目录），故不入图，避免把未启用模块画成生效链路。
- 图中「失败不兜底」「语义只影响限速与云台时序」均来自既有算法文档与代码路径，属已核实行为，不是性能承诺。
- 按用户要求**刻意省略**全部数值与话题名；因此本图不能替代模块文档做参数核查。

### Recommended modification boundary

只新增 `docs/架构图/` 下 nav 图与其说明文档，不改既有地图侧文件，不动源码。

## Modifier Changes

### Files changed

- 新增 `docs/架构图/nav_architecture.svg`
- 新增 `docs/架构图/nav_architecture.png`
- 新增 `docs/架构图/导航算法架构图.md`
- 新增 `docs/ai_refactor_records/20261106_nav_architecture_figure.md`

### Key changes

- 版式沿用地图架构图：白底 + 虚线外框 + 圆角卡片组 + 语义色 + 开放箭头 + 图注；配色沿用既有色板（蓝 / 琥珀 / 绿 / 紫 / 石板灰）。
- 结构：左侧输入列（目标、状态、只读快照、语义）→ ① 编排 · 决策 → ② 规划链 · 分层降维（路径搜索 / 时间参数化 / 轨迹优化，各自标注产出）→ ③ 控制 · 闭环（轨迹接口 / 滚动优化）→ 右侧输出列（底盘速度、云台时序、导航状态）。
- 底部「运行时基座」条带承载 4 条架构约束；底部虚线箭头表示位姿与跟踪误差回流触发重算。

### Behavior preserved

未改动任何可执行逻辑、参数、话题、模块边界、launch 组合。

### Behavior intentionally adjusted

无。

### Notes

- 渲染方式：Chrome headless，`--force-device-scale-factor=2`，窗口 = SVG 尺寸，输出 3440 × 1280 PNG；使用仓库内 `.chrome-home` / `.chrome` 作为临时 profile，不写入仓库产物。
- 依据用户「不含数值 / 话题名」要求，图中所有具体数字和 ROS 名称均已剔除。

## Auditor Review

### Checks performed

- [x] 关键路径检查：图中三层与四个环节逐项对照 `docs/nav/算法思路/planner/01` 与源码头文件
- [x] diff 检查：`git status` 仅新增文档与图片，无源码改动
- [x] grep 检查：`target_tracking` / `yaw_plan` 无外部调用点，已排除出图
- [x] XML 检查：`xml.etree.ElementTree` 解析 SVG 通过
- [x] 渲染检查：Chrome headless 输出 3440 × 1280 PNG
- [x] 视觉检查：整图复检两次，修正输入/输出卡与流水线带高度不齐、图注对编排层职责表述偏窄
- [x] XML / launch / yaml 检查：未修改
- [x] 用户允许范围内的测试或静态检查：仅结构与渲染检查
- [x] 如需构建，已取得用户明确许可：**未构建**

### Issues found

- 首版输入/输出卡高度与流水线分组带不一致，视觉上不齐；已统一为同一底边。
- 首版图注写「编排层只决定是否重算」，弱化了其驱动求解的职责；已改为「决定是否重算并驱动后续求解」。

### Final result

PASS
