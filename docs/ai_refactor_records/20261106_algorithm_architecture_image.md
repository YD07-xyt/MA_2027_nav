# 算法模块架构图图片 改造记录

> 本记录为**文档图片资产新增**任务，未修改任何源代码、配置、launch 或参数。

## User Intent

用户要求把算法模块架构图做成"美观的图片"，并给出参考材料 `docs/nav/开源参考/`（三份兄弟队伍技术报告）。

约束（AGENTS.md）：

- 不修改比赛主逻辑、参数、话题、模块边界。
- 不把未验证推断写成事实。
- 不执行构建；不执行任何 Git 写操作。
- 图片内容以当前工作区源码为准（HEAD `006e7b2`）。

## Scope

- 新增 `docs/架构图/算法模块架构图.svg`（可编辑矢量源）。
- 新增 `docs/架构图/算法模块架构图.png`（4096×2400，2× 渲染）。
- 新增本记录。
- 生成脚本放在 gitignore 的 `.cache/arch/gen.py`，不入库（可复现，不产生仓库垃圾）。

## Out of Scope

- 不修改 `docs/架构图/地图算法架构图.md`、`docs/架构.md`。
- 不修改 `docs/nav/` 9 文件契约文档，不重跑 `docs/tools` 校验流水线。
- 不改任何 `src/`、`config/`、`launch/` 文件。
- 不处理 `docs/架构图/` 中非本次任务产生的文件（`_ref/`、`_test.*`、`.chrome/`）。

## Explorer Findings

### Files inspected

- `docs/nav/开源参考/RM2025中科大导航.md`、`RM2026北京理工大学追梦战队哨兵导航.md`、`RM2026哈工大(威海)哨兵导航方案.md`
- `docs/nav/开源参考/_ref` 中提取的参考图（`hit_arch1.webp` 等）
- 本仓库地图/导航源码与配置（前序地图架构分析已核对：`ma_map.hpp`、`prob_map.cpp`、`terrain_analysis.cpp`、`grid_map.cpp`、`fsm_replanner.cpp`、`traj_optimizer.h`、`mpc.h`、`ros2_node.cpp`、`config/*.yaml`）

### Active logic path

图片表达的主链：`点云/位姿 → 3D 概率占据 → 持久地面层 → 地形判定 → 2D 栅格 + EDT → 只读快照 → 重规划 FSM → JPS/A* → 后处理 → MA-Spline 优化 → 轨迹接口 → MPC → world→body → /cmd_vel_chassis`；旁路为折叠事件与导航反馈。

### Reference style notes

参考报告的共同写法是「整体架构 / 数据链」：模块框 + 箭头 + 分组边界 + 契约标注（如北理工 §1 的 `Livox Driver → Point-LIO → ROGMap → MincoPlanner → MPC → 底盘`）。本图沿用该结构，视觉采用当前默认墨图风格（暖中性纸底、细边框、开放箭头、语义色），避免照抄参考图配色。

### Risk notes

- 图中参数值（0.35 m、33/10 ms、N=20、dt=0.05 s、0.24 m、25° 等）均取自 `config/*.yaml` 与源码，未写未验证的运行期结论。
- 「失败语义：本帧不下发」来自既有审计结论与代码路径，图中作为契约注记而非性能承诺。

## Modifier Changes

### Files changed

- 新增 `docs/架构图/算法模块架构图.svg`
- 新增 `docs/架构图/算法模块架构图.png`
- 新增 `docs/ai_refactor_records/20261106_algorithm_architecture_image.md`

### Key changes

- 采用两大部分布局：PART 1 地图（4 级流水 + 只读快照）、PART 2 导航（规划行 33 ms、控制行 10 ms，蛇形数据流）。
- 契约标注：`只读 GridMap 快照`、`MAsplineOutput（五次样条 + 刹车段）`、虚线反馈 `重规划触发`。
- 输出与运行时契约：`/cmd_vel_chassis`、`/ma_nav/fold`、`/ma_nav/nav_feedback`，以及单进程/定时器/world 系约定。
- 生成方式：Python 生成固定版式 SVG → Chrome headless `--force-device-scale-factor=2` 截图 4096×2400 PNG。

### Behavior preserved

未改动任何可执行逻辑、参数、话题、模块边界。

### Behavior intentionally adjusted

无。

### Notes

- SVG 为手排版矢量源，可继续编辑；PNG 由同一 SVG 渲染，二者内容一致。
- 渲染命令（不入库）：`google-chrome --headless=new --force-device-scale-factor=2 --window-size=2048,1200 --screenshot=... file://.../算法模块架构图.svg`。

## Auditor Review

### Checks performed

- [x] 关键路径检查：图中模块与数据流对照源码/配置逐项核对
- [x] diff 检查：`git status` 仅新增文档与图片（无源码改动）
- [x] SVG 结构检查：`xml.etree.ElementTree` 解析通过（2048×1200 viewBox）
- [x] 渲染检查：Chrome headless 输出 4096×2400 PNG
- [x] 视觉检查：整图 + 4 处局部裁剪（反馈区、右缘、MPC 卡、总线标签、图例）确认无文字溢出/重叠
- [x] 配色检查：按当前墨图风格语义色（绿/蓝/紫/琥珀/红/暖灰）
- [x] XML / launch / yaml 检查：未修改
- [x] 用户允许范围内的测试或静态检查：仅结构与渲染检查
- [x] 如需构建，已取得用户明确许可：**未构建**

### Issues found

- 首版（冷色卡片风）存在右栏标题溢出、反馈标签与控制行标题拥挤；已在第二版调整卡片宽度、缩短标题并重排纵向节奏。
- 第二版按墨图风格替换配色与细边框、开放箭头，消除重投影阴影。
- 最终版修正文案：`提前折叠 · fold_time 1.0 s`、`3 SUCCESSED`。

### Final result

PASS
