# 规划算法架构图 改造记录

> 本记录为**文档图片资产新增**任务，未修改任何源代码、配置、launch 或参数。

## User Intent

在上一张 nav 总架构图之后，用户要求：

1. **只生成规划部分**的算法架构（不含地图侧、不含控制层）。
2. **接口由各个模块之间的箭头表示，不要写在模块内** —— 即模块框内只放算法步骤，跨模块的数据契约一律标在箭头上。
3. 参考 `docs/架构图/地图算法架构图.png` 的视觉语言（白底卡片 + 中文标题 + 斜体英文副标 + 灰色机制注记 + 虚线分组框 + 彩色箭头 + 底部多行图注 + 示意图面板）。

沿用上一轮约束：不含特定数值、不含话题名。

AGENTS.md 约束：不改比赛主逻辑、参数、话题、模块边界；不把未验证推断写成事实；不执行构建；不执行任何 Git 写操作。

## Scope

- 新增 `docs/架构图/nav_planner_architecture.svg`（手排矢量源，1720 × 870）。
- 新增 `docs/架构图/nav_planner_architecture.png`（2× 渲染，3440 × 1740）。
- 新增 `docs/架构图/规划算法架构图.md`（图注 + 四段职责 + 编排特殊性 + 地图接法 + 三种表达）。
- 新增本记录。
- 同步把 `docs/架构图/nav_architecture.svg` 的定位明确为「总架构」，本图为其 ①②③ 中规划部分的展开。

## Out of Scope

- 不修改地图侧既有产物（`map_architecture.*`、`地图算法架构图.*`、`算法模块架构图.*`）。
- 不修改 `docs/nav/` 契约文档与 `docs/nav/算法思路/` 算法文档。
- 不改任何 `src/`、`config/`、`launch/`、`CMakeLists.txt`。
- 不展开控制层（轨迹接口、MPC、world→body 转换）。

## Explorer Findings

### Files inspected

- `docs/架构图/地图算法架构图.png`（本图风格基准，重读以对齐视觉语法）
- `docs/架构图/地图算法架构图.md`、`docs/架构图/README.md`
- `docs/nav/算法思路/planner/01-导航框架与数据流.md`、`06-重规划策略.md`
- 源码：`src/planner/replan/include/replan/fsm_replanner.h`、`src/planner/path_planning/include/path_planning/path_planning.hpp`、`post_processing.h`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h`

### Active logic path

`到达判定 → 触发判定 → 起终点外推 → set_use_jps / set_velocity → 搜索+后处理 → 距离场注入 → 两阶段优化 → 返回 expected`。
接口对应：`get_safe_pos` 产出的安全起终点、`Trajectory.path_states/time_segments`、`MAsplineOutput.trajectory`。

### Data flow

编排包裹整条链：向下下发任务，向上接收结果；地图以只读快照从底部供给；规划不向地图回写。

### Risk notes

- 图中 `两阶段求解` 用红色实心卡标为核心步骤，对应参考图中「对数几率更新 / 持久地面层」的高亮用法，不表示性能优劣。
- 「只读，一次规划内不变」来自地图侧只读快照契约，属已核实行为。
- 按用户要求剔除全部数值：原 `五次连续曲线` 已改为 `平滑连续曲线`（「五次」是具体阶数，属数值信息）。

### Recommended modification boundary

只新增规划部分的图与其说明文档，不改总架构图、不改源码。

## Modifier Changes

### Files changed

- 新增 `docs/架构图/nav_planner_architecture.svg`
- 新增 `docs/架构图/nav_planner_architecture.png`
- 新增 `docs/架构图/规划算法架构图.md`
- 新增 `docs/ai_refactor_records/20261106_nav_planner_architecture_figure.md`

### Key changes

- 四个模块改为**虚线分组框**，框内是该段的算法步骤小卡；框与框之间留出宽间隙，把接口写在箭头上：
  - `可通行起终点 ＋ 当前速度`、`栅格折线路径`、`带时间的路点与分段时长`、`连续轨迹`。
- 模块标题采用「中文 + 斜体英文」双行；每张小卡为「步骤名 + 一行灰色机制注记」。
- `两阶段求解` 用红色实心卡高亮核心步骤；`网格折线 / 时间采样 / 光滑曲线` 三张示意图面板说明同一段路的三种表达。
- 顶部虚线回流表示「优化结果 → 编排裁决」；底部绿色供给轨表示地图只读输入。

### Behavior preserved

未改动任何可执行逻辑、参数、话题、模块边界、launch 组合。

### Behavior intentionally adjusted

无。

### Notes

- 渲染方式：Chrome headless，`--force-device-scale-factor=2`，窗口 = SVG 尺寸，输出 3440 × 1740 PNG。
- 编排框内四张卡按**执行顺序**自上而下排列（到达 → 触发 → 修正 → 裁决），跨框箭头按**阶段**连接，不对应框内某一行。

## Auditor Review

### Checks performed

- [x] 关键路径检查：四段与六个接口逐项对照源码与 `01-导航框架与数据流.md`
- [x] diff 检查：`git status` 仅新增文档与图片，无源码改动
- [x] grep 检查：图中文字块 72 个，无阿拉伯数字与话题名（仅图注编号「图 1」）
- [x] XML 检查：`xml.etree.ElementTree` 解析 SVG 通过
- [x] 渲染检查：Chrome headless 输出 3440 × 1740 PNG
- [x] 视觉检查：整图复检两轮 —— 修正折线示意离障碍过近、`五次` 数值化表述
- [x] XML / launch / yaml 检查：未修改
- [x] 用户允许范围内的测试或静态检查：仅结构与渲染检查
- [x] 如需构建，已取得用户明确许可：**未构建**

### Issues found

- 首版示意图面板一的折线离障碍块过近（净空不足），已抬高折线并左移竖直段。
- 首版小卡写「五次连续曲线」，属具体阶数，按「不含特定数值」改为「平滑连续曲线」。

### Final result

PASS
