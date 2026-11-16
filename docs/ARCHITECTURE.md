# NAV 架构索引

本文件记录当前源码事实；接口概览见 [interface.md](interface.md)，历史设计材料见 [report](report/)。

## 当前启动与配置

`launch/run.launch.py` 启动 `ma_nav` 和 RViz；包 share 的 planner.yaml 或 launch 覆盖值经 `planner_params_path` 传入 [src/ma_nav.cpp](../src/ma_nav.cpp)。入口构造 `GlobalPlanner2d`，其 `planner::Config` 通过 yaml-cpp 读取配置，并构造地图、控制及重规划对象。

`GlobalPlanner2d` 在 [src/ros2](../src/ros2/) 中承担地图更新、规划/控制回调、状态与通信编排；这不是已完成的薄壳隔离。

## 当前 CMake 直接依赖

箭头为调用/链接方 → 依赖；这里只列一方链接声明，不声称已验证完整传递依赖。

```text
ma_nav → ros2 + replan + controller + traj_optimize + path_planning + 3d_occ_map + utils
ros2 → replan + controller + traj_optimize + path_planning + 3d_occ_map + utils
replan → controller + traj_optimize + path_planning + utils
controller → traj_optimize + path_planning + utils
traj_optimize → path_planning + utils
path_planning → 3d_occ_map + utils
3d_occ_map → utils
```

`src/test/` 构建离线比较、优化、地图转换和地形回放程序；当前无 CTest 注册闭环。

## 通信与执行边界

| 数据/任务 | 当前入口与归属 |
| :--- | :--- |
| 点云、里程计 | `map_callback` / `odom_callback`，同一 map callback group |
| 目标 | `target_callback`，control callback group |
| 规划 | planner callback group，33 ms timer |
| 控制与可视化 | control callback group，各 10 ms timer |
| 速度 | GlobalPlanner2d 发布到 YAML 的 `ros2.cmd_vel_name` |
| 折叠/导航反馈 | GlobalPlanner2d 发布 `/ma_nav/fold`、`/ma_nav/nav_feedback` |
| 地图/隧道保存 | 配置命名的 Trigger 服务，输出路径来自 YAML |

回调组划分不证明跨组数据安全；地图快照、目标/轨迹等共享状态的锁与生命周期按实际成员读写核查。现有 topic、frame、QoS、频率和比赛状态优先级变更均按根规范确认。

## 当前验证能力

静态工具面向本仓库；ROS 入口例外为 `src/ma_nav.cpp` 与 `src/ros2/`。全量依赖图、运行行为和无 ROS 独立核心构建尚未验收，见 [STANDARD_GAPS.md](STANDARD_GAPS.md)。
