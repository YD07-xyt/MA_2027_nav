# MA_Nav 模块设计、联动与部署问题文档改造记录

## User Intent

用户要求读项目，详细分析各模块设计问题、模块联动后的算法与工程问题，以及MID360视野等实际部署考量，写成问题文档。

## Scope

仅新增中文问题审计文档和本事后记录。单人静态审计；未使用多Agent；不改变运行行为。遵循仓库禁止未授权构建、禁止Git提交类操作与禁止writingplans的限制。

## Out of Scope

不修复运行代码，不调比赛参数，不改topic/frame/QoS/timer/启动组合，不删除旧逻辑，不全面审计所有第三方项目，不运行构建/节点/rosbag/实机，不声称系统已获得安全认证。

## Explorer Findings

### Files inspected

- [规则](</home/xyt/map/src/MA_Nav/AGENTS.md>)、[README](</home/xyt/map/src/MA_Nav/README.md>)、[入口](</home/xyt/map/src/MA_Nav/src/ma_nav.cpp>)、[launch](</home/xyt/map/src/MA_Nav/launch/run.launch.py>)。
- [planner配置](</home/xyt/map/src/MA_Nav/config/planner.yaml>)、[map配置](</home/xyt/map/src/MA_Nav/config/map.yaml>)、[配置加载](</home/xyt/map/src/MA_Nav/src/ros2/include/ros2/config.hpp>)、[ROS头文件](</home/xyt/map/src/MA_Nav/src/ros2/include/ros2/ros2_node.h>)、[ROS实现](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp>)、[可视化](</home/xyt/map/src/MA_Nav/src/ros2/include/ros2/misc/visualizer.hpp>)。
- [MaMap](</home/xyt/map/src/MA_Nav/src/map/include/map/ma_map.hpp>)、[GridMap头文件](</home/xyt/map/src/MA_Nav/src/map/include/map/grid_map.hpp>)、[GridMap实现](</home/xyt/map/src/MA_Nav/src/map/src/grid_map.cpp>)、[ROG](</home/xyt/map/src/MA_Nav/src/map/src/3d_occ_map/rog_map.cpp>)、[概率射线/遗忘](</home/xyt/map/src/MA_Nav/src/map/src/3d_occ_map/prob_map.cpp>)、[滑动地图](</home/xyt/map/src/MA_Nav/src/map/src/3d_occ_map/sliding_map.cpp>)、[地面层](</home/xyt/map/src/MA_Nav/src/map/src/ground_layer.cpp>)、[地形分析](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp>)及对应配置/头文件。
- [路径规划](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/path_planning.cpp>)、[JPS](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/search/jps.cpp>)、[A*](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/search/astar.cpp>)、[后处理](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/post_processing.cpp>)及头文件。
- [重规划](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp>)、[优化器](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h>)、[优化代价](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp>)、[ESDF适配](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/grid_map_esdf.hpp>)与优化配置类型。
- [MPC](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/mpc.h>)、[轨迹接口](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/traj_interface.hpp>)、[yaw骨架](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/yaw_plan.hpp>)、目标跟踪的[Tracker](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/target_tracking/target_tracker.cpp>)、[Estimator](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/target_tracking/target_estimator.cpp>)、[ViewpointSelector](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/target_tracking/viewpoint_selector.cpp>)。
- [构建入口](</home/xyt/map/src/MA_Nav/CMakeLists.txt>)、[测试构建配置](</home/xyt/map/src/MA_Nav/src/test/CMakeLists.txt>)与部分子模块CMake、[包元数据](</home/xyt/map/src/MA_Nav/package.xml>)、[离线地形回放](</home/xyt/map/src/MA_Nav/src/test/src/test_terrain_replay.cpp>)、[通用YAML加载](</home/xyt/map/src/MA_Nav/src/utils/include/utils/yaml_loader.hpp>)、[状态类型](</home/xyt/map/src/MA_Nav/src/utils/include/utils/type_utils.hpp>)、[日志](</home/xyt/map/src/MA_Nav/src/utils/include/utils/logger.hpp>)。
- 外部新增的[轨迹优化测试](</home/xyt/map/src/MA_Nav/src/test/src/test_traj_optimize.cpp>)只抽查前70行，不声称全量审计或运行。

### Active logic path

单导航节点 → 外部LIO云/odom → ROG/GroundLayer/Terrain → 2D ESDF快照 → JPS/后处理 → MA spline OMNI_XY → 轨迹投影/MPC → body Twist；独立Bool折叠命令与Int16导航反馈。默认mapping_model=true。目标追踪库未接入当前ROS链。spin_some与真正多线程spin的风险分开记录。

### Data flow

详见[主文档第2章](</home/xyt/map/src/MA_Nav/docs/MA_Nav_设计联动与实机部署问题审计.md>)；同步追踪pose/地图/轨迹/事件/速度状态及其所有者与时间戳。未把注释中的安全兜底、配置键存在或第三方源码存在当成已生效保证。

### Risk notes

53项问题，分P0/P1/P2，标注事实/推导/待核实。重点是旧轨迹失效与停车、最终样条验收、未知/盲区、时间/动力学一致性及折叠物理反馈。硬件数据来自[Livox官方规格](https://www.livoxtech.com/mid-360/specs)；执行器语义交叉核对[ROS2 Humble Executor](https://raw.githubusercontent.com/ros2/rclcpp/humble/rclcpp/src/rclcpp/executor.cpp)及[多线程实现](https://raw.githubusercontent.com/ros2/rclcpp/humble/rclcpp/src/rclcpp/executors/multi_threaded_executor.cpp)。本机实际ROS版本/部署二进制一致性仍未验证。

### Recommended modification boundary

本次只文档。后续修复必须逐项确认边界，优先闭合安全契约而非整体重构。本次检查涉及比赛主链，但不修改比赛验证逻辑，采用最小改动策略。

## Modifier Changes

### Files changed

1. [问题审计文档](</home/xyt/map/src/MA_Nav/docs/MA_Nav_设计联动与实机部署问题审计.md>)。
2. [本记录](</home/xyt/map/src/MA_Nav/docs/ai_refactor_records/20261102_design_integration_deployment_audit.md>)。

### Key changes

新增模块问题、6条跨模块故障链、MID360视场/安装/停车距离推导、工程部署问题、问题关闭证据和现场待核实清单。主文档包含132个本地源码链接，逐一校验存在与行号范围。

### Behavior preserved

运行源码、参数、接口和启动方式均未改动；未触碰用户已有源码/图片/测试/报告内容。没有编译、生成代码、Git暂存/提交/推送。

### Behavior intentionally adjusted

无运行行为调整；仅增加问题分析和风险说明。

### Notes

初始HEAD为`4b4bcb0d12578574a714878a5f6216b244bcfc33`，工作区已有9个已跟踪变更及多个未跟踪文件。审计期间外部HEAD推进至`c9d1aa2a6af5479bf8840052990d94aa917b5126`，原有变更部分被外部提交，并新增轨迹优化测试/结果。本Agent未执行该提交。已通过git show读取外部提交文件清单，复读当前测试CMake；核心ROS/地图/重规划/MPC/优化器实现不在该次外部提交列表中。新测试未进入完整运行/安全验收范围；最终不承诺仓库clean。

## Auditor Review

同一执行者重新读取关键源码进行自审，非独立Agent审计。文档写入先被read-only沙箱拒绝，随后按工具要求以workspace-write审批进行文档写入；没有绕过文件权限。

### Checks performed

- [x] 关键路径检查：复读失败返回、轨迹交接/事件顺序、MPC状态与模型、未知合并、悬空放行和PGM映射。
- [x] diff检查：初始/最终git status、diff统计、git diff --check；外部提交用只读git show核对，未归入本次变更。
- [x] grep检查：目标追踪调用者、碰撞验收调用者、热启动/限频/轨迹截断参数消费者、u_prev与地形参数活跃使用。
- [x] launch/yaml/XML检查：Python ast.parse、yaml.safe_load、ElementTree.parse均通过，不生成字节码/代码。
- [x] 文档静态检查：53个唯一问题ID；132个本地文件链接存在且行号不越界；关键几何量计算通过。
- [x] 用户允许范围内的静态检查；运行测试未执行，不宣称通过。
- [ ] 构建：未请求也未取得构建许可，未执行。

### Issues found

审计期间外部工作区发生漂移，已在主文档开头与本记录明确披露；没有覆盖外部内容。安全问题仅记录，未修复。静态语法通过不代表算法安全或实机可部署。MID360离地0.194 m只是条件示例，不能将sensor_offset自动等同实际离地高度。

### Final result

**PASS（本次文档交付与静态核对范围）**。证据等级B：目标文档有源码与官方资料支持，运行/现场边界明确；系统部署安全结论仍为 **NEEDS_FIX / needs-verification**。最高价值下一证据为失效停车闭环、最终执行轨迹验收、真实雷达安装及制动包络。

未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
