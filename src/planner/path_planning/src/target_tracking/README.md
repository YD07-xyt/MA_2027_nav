# target_tracking：可见性约束的目标跟踪（第一阶段）

给定目标物体在世界系中的位置观测，持续输出一个**能看见目标、离目标合适距离、机器人能站**的观测点（作为导航 goal），以及云台 yaw 指令。

- 命名空间：`path_planning::target_tracking`
- 头文件：`src/planner/path_planning/include/path_planning/target_tracking/`
- 源文件：`src/planner/path_planning/src/target_tracking/`（本目录）
- 编进 `path_planning` 库（`CMakeLists.txt` 递归 GLOB `src/*.cpp`，新增文件无需改 CMake）
- 依赖：`map/core/grid_map.hpp`（`grid_map::GridMap`）、Eigen。**不依赖 ROS**，不持有地图、不开线程。
- 设计参考：`3rd/Elastic-Tracker`（原版）、`3rd/SentryNav2026_XDU/.../elastic_tracker_nav2`（移植版）

## 1. 背景与约束

| 项 | 约定 |
|---|---|
| 相机 | 跟踪用相机装在云台上；**云台 yaw 可 360° 连续转** |
| 云台控制权 | **由导航模块下发**云台角；导航决定何时把 yaw 释放给视觉自瞄 |
| 避障地图 | mid360 建的 2D 栅格（`GridMap`），与相机无关 |
| 遮挡模型 | **暂时处理**：2D 地图中所有占据格都挡视线（不区分障碍高度） |
| 轨迹优化 | **第一阶段不改优化器**：只换 goal，下游仍是现有 `FsmReplan` |

## 2. 快速接入

```cpp
#include "path_planning/target_tracking/target_tracker.h"
using namespace path_planning::target_tracking;

TargetTracker tracker;
TargetTrackingParams p;            // 参数含义与调参见 tracking_types.h
tracker.set_param(p);

// 目标观测回调（任意频率）。stamp 与 update 的 now 必须是同一时钟，单位 s
tracker.on_target_measurement(Eigen::Vector2d(tx, ty), stamp);

// 规划定时器（建议 10–20 Hz）
RobotState2D rs{robot_xy, robot_vxy, chassis_yaw};
TrackingOutput out = tracker.update(grid_map, rs, now);

if (out.hold) {
    // 已在平衡位置：停车（或保持当前位置），不要重规划
} else if (out.goal && out.goal_updated) {
    // 把 out.goal->pos（及可选 out.goal->yaw）作为 goal 交给 FsmReplan::plan
} else if (!out.goal) {
    // 无可行观测点或 IDLE：调用方决定（继续执行旧轨迹 / 停车 / 切回其他行为）
}
// out.goal && !out.goal_updated：goal 与上次相同，不必重规划

// 云台
if (out.gimbal.release) { /* 交给视觉自瞄 */ }
else { /* 下发 out.gimbal.yaw（世界系），可用 out.gimbal.yaw_rate 做前馈 */ }
```

注意事项：

- **线程**：`TargetTracker` 非线程安全。观测回调和定时器在不同线程时，调用方加锁。
- **地图**：`update` 每次传入当前地图的 const 引用，不拷贝。传入未膨胀地图时 `viewpoint_clearance` 设为车体半径 + 余量；传入已膨胀地图时调小它，并把膨胀半径加到 `target_ignore_radius` 里。
- **换目标 / 重新开始**：调用 `tracker.reset()`，否则新目标的观测可能被新息门限拒绝最多 `kf_reset_dt` 秒。
- **云台角**：`gimbal.yaw` 是世界系、归一化到 `[-π, π]`；换算到云台相对角和连续转动的解卷绕由调用方负责。

## 3. 文件与类

| 文件 | 类 / 内容 | 说明 |
|---|---|---|
| `tracking_types.h` | `TargetTrackingParams` 及所有输入输出结构 | **参数说明和调参指南在这里** |
| `target_estimator.h/.cpp` | `TargetEstimator` | 2D 匀速 KF，状态 `[px, py, vx, vy]` |
| `viewpoint_selector.h/.cpp` | `ViewpointSelector` | 无状态几何：视线、预测、选点、可见扇形。可单独测试 |
| `target_tracker.h/.cpp` | `TargetTracker` | 对外接口：状态机、hold、云台、goal 防抖，组合上面两个 |

## 4. 每个周期的算法流程（`TargetTracker::update`）

```
age = now - 最近有效观测时刻
1. 状态机       age → TRACKING / PREDICTING / SEARCHING / IDLE（IDLE 直接返回空 goal）
2. 预测         TRACKING/PREDICTING：从 KF 状态匀速外推，遇占据格截断
                  target_pos = 预测轨迹第 0 点（= 外推到 now）
                  center     = 预测轨迹 now + lookahead_time 处的点（观测中心）
                SEARCHING：不外推，target_pos = center = 最后有效观测位置
3. 视线         visible_now = LOS(robot, target_pos)，忽略目标本体半径内的占据
4. 云台         yaw = atan2(target_pos + v·lead - robot)，yaw_rate = (r × v_rel)/|r|²
                release = enable && TRACKING && visible_now && age < release_fresh_time
5. hold 判定    TRACKING && visible_now && |dist - tracking_dist| < tolerance_d
                && |v_target| < hover_target_vel && |v_robot| < hover_robot_vel
                → hold=true，goal 为空，清空上次 goal（离开平衡后会强制重新下发）
6. 选点         ViewpointSelector::select(center, robot)
7. goal 防抖    与上次下发 goal 距离 > goal_update_dist 才更新并置 goal_updated
```

### 4.1 选点 `ViewpointSelector::select`

1. **弹簧点**：`s = center + r_des · (robot - center)/|robot - center|`，`r_des = clamp(tracking_dist, camera_min_dist, camera_max_dist)`。它满足"安全 + 视线畅通"就直接用（`spring_used = true`）。
2. **环上搜索**（弹簧点不可用时）：半径序列 `r_des, r_des ± tolerance_d, r_des ± 2·tolerance_d, …`（限制在 `[camera_min_dist, camera_max_dist]` 内），每个半径从弹簧方向向两侧按 `search_angle_step` 扫描，取**离弹簧点欧氏距离最小**的可用点。已找到的最优代价用于剪枝。
   - 对应原版 `findVisiblePath`（6 邻域 A*，停止条件为"距离合适 + 可见"）。这里改成环上枚举，原因：2D 下候选少，枚举可以保证最优，不用逐格扩展。代价是不考虑从机器人到该点的实际路径长度（那部分交给下游规划器）。
3. **可见扇形 + theta_clearance**：在选中半径上，从选中角度向两侧扫描能看见 center 的最大连续区间 `[t_l, t_r]`（每侧最多 π），离任一边缘不足 `min(theta_clearance, 半宽)` 就推向内部。推移后的点不安全则保留原点。对应原版 `visible_pair`。

> 为什么选"观测点"而不是直接用弹簧点：弹簧点只管距离，被墙挡住时机器人会站在墙后看不见目标（仿真里纯弹簧点的可见率只有 67.9%）。观测点 = 以弹簧点为首选、被挡才搜索的结果，两者在开阔区域完全一致。

### 4.2 视线 `line_of_sight`

- `los_clearance == 0`：Amanatides–Woo 2D DDA，遍历线段经过的每个栅格，遇到占据格（且不在忽略圆内）即遮挡。不会漏掉斜穿的薄墙。
- `los_clearance > 0`：沿线以半个栅格为步长采样 ESDF，`< los_clearance` 即遮挡。
- 地图外视为未知、**不遮挡**（与 `GridMap::isCollision` 约定一致）；但观测点本身必须在地图内（`is_safe`）。

### 4.3 目标估计 `TargetEstimator`

- 匀速模型 + 白噪声加速度过程噪声（`kf_acc_noise`），位置观测（`kf_meas_noise`）。
- 首次观测、或距上次有效观测 > `kf_reset_dt`：用该观测重新初始化，速度置 0，速度方差为 `target_vmax²`。
- 拒绝条件（返回 false，不改状态）：时间戳倒退；新息马氏距离 > `kf_gate_chi2`；更新后速度 > `target_vmax`。
- `pos()/vel()` 是**最近一次有效观测时刻**的状态，不外推；外推由 `ViewpointSelector::predict` 完成。

## 5. 与 Elastic-Tracker 原版的对应关系

| 原版 (`3rd/Elastic-Tracker/src/...`) | 本模块 | 差异 |
|---|---|---|
| `detection/target_ekf`（EKF，相机投影观测，`checkValid` 速度门限） | `TargetEstimator` | 2D、直接用世界系位置观测；多了新息门限 |
| `planning/.../prediction`（加速度空间搜索预测） | `ViewpointSelector::predict` | **暂时处理**：匀速外推 + 障碍截断 |
| `env.hpp: findVisiblePath`（A* 搜到距离合适且可见的点） | `select` 第 1–2 步 | 环上枚举，代价为离弹簧点距离 |
| `env.hpp: visible_pair / generate_visible_regions` | `visible_arc` + `select` 第 3 步 | 只对观测中心一个时刻做，原版对预测轨迹每个时刻都做 |
| `planning_nodelet.cpp` 悬停条件（距离容差 + 速度阈值） | `update` 第 5 步 | 不要求底盘朝向（云台 360°） |
| `traj_opt.cc: grad_cost_p_tracking / grad_cost_visibility` | **无（第一阶段不改优化器）** | 见第 7 节 |

## 6. 参数

**每个参数的含义、影响和建议范围都写在 `tracking_types.h` 的注释里，以那里为准**。下面只列调参顺序和常见现象。

调参顺序：

1. 相机：`camera_min_dist`、`camera_max_dist`、`tracking_dist`
2. 地图：`viewpoint_clearance`、`target_ignore_radius`（以及地图是否膨胀）
3. 稳定性：`goal_update_dist`、`tolerance_d`、`theta_clearance`
4. 估计与丢失：`kf_acc_noise`、`kf_meas_noise`、`kf_gate_chi2`、各 `*_timeout`

常见现象 → 调哪个：

| 现象 | 可能原因 | 调整 |
|---|---|---|
| 目标明明在视野内，`gimbal.release` 一直 false / `visible_now` 为假 | 目标被 mid360 建成障碍挡住了自己 | 调大 `target_ignore_radius`（地图膨胀过还要加膨胀半径） |
| 经常 `goal` 为空 | 观测点安全距离太大，或相机距离范围太窄 | 调小 `viewpoint_clearance`，放宽 `camera_min/max_dist` |
| goal 来回跳、频繁重规划 | 选点在两个等价位置间切换，或目标估计抖 | 调大 `goal_update_dist`；调小 `kf_acc_noise` 或调大 `kf_meas_noise` |
| 目标一动就被墙角挡住 | 站在可见扇形边缘 | 调大 `theta_clearance`，或设 `los_clearance` 0.05–0.15 |
| 机器人总在目标后面追 | 观测中心太靠近当前位置 | 调大 `lookahead_time`（≤ `tracking_dur`） |
| 目标转向后机器人冲过头 | 预测太远 | 调小 `lookahead_time` |
| 墙后目标时贴得很近 | 选点代价只看离弹簧点距离（见第 8 节） | 调大 `camera_min_dist` |
| 快速目标频繁进 PREDICTING | 真实观测被门限拒绝 | 调大 `target_vmax`、`kf_gate_chi2` 或 `kf_acc_noise` |
| 目标停下后机器人仍在微调 | hold 条件太严 | 调大 `tolerance_d`、`hover_target_vel` |
| 云台落后于目标 | 链路延迟 | 调大 `gimbal_lead_time` |

## 7. 后续阶段（未做）

- **第二阶段：优化器加跟踪代价**。参考原版 `traj_opt.cc` 的 `grad_cost_p_tracking`（距离保持）和 `grad_cost_visibility`（可见扇形约束），在 `traj_optimize` 模块中对预测轨迹每个时刻加代价。到时需要 `select` 对 `prediction` 的每个点都输出 `ViewpointInfo`（接口已预留 `prediction` 序列）。这一步会动 `traj_optimize` / `replan`，需要另行确认。
- 预测改为原版的加速度空间搜索（会绕障）。
- 遮挡按障碍高度区分（需要 3D 地图或高度层，低矮障碍不挡视线）。
- SEARCHING 状态做主动搜索（例如沿最后速度方向巡视），而不只是去最后位置。
- 估计器使用观测协方差 / 速度观测。

## 8. 已知问题 / 暂时处理清单

代码里标了 `暂时处理` 的地方：

| 位置 | 内容 | 影响 |
|---|---|---|
| `viewpoint_selector.h` 文件头、`line_of_sight` | 所有占据格都挡视线 | 矮墙、台阶后的目标会被判为不可见，偏保守 |
| `viewpoint_selector.h` `predict`、`.cpp` `predict` | 匀速外推 + 障碍截断 | 目标转弯 / 绕障时预测错误 |
| `target_tracker.cpp` SEARCHING 分支 | 只去最后看见的位置 | 不做主动搜索 |
| `target_estimator.h` 文件头 | 固定观测噪声，只用位置 | 观测质量变化时不自适应 |
| `target_tracker.h` 文件头 | 只保证观测点处可见 | 从当前位置走到观测点的途中可能看不见目标 |

另外两条：

- **墙后目标贴近选点**：选点代价是"离弹簧点的欧氏距离"。目标在墙后时，弹簧点所在半径的所有角度都被挡，最近的可用点可能出现在小半径上（单元测试中 r = 0.6 m，接近 `camera_min_dist`）。需要的话，可以在 `select` 第 2 步的代价里加半径偏差权重，例如 `cost = |p - s| + w·|r - r_des|`。
- **clang 编译**：仓库现有 `build/` 是 clang Debug 配置，`utils/logger.hpp` 在 clang 下本身就编译不过，与本模块无关。本模块用 gcc 编译通过。

## 9. 验证记录（2026-10-25）

临时测试 `.cache/tt_test/test_target_tracking.cpp`（已被 gitignore，不在仓库中）。需要的话可以把它移到 `src/test/` 并加 CMake target。

- 单元用例全部通过：开阔地图直接选中弹簧点；视线检测（正向、反向、斜穿、忽略圆、ESDF 余量）；墙挡住弹簧点后选出安全可见点，半径在相机范围内；预测在墙前截断；KF 速度估计误差 < 0.15 m/s；跳变观测和时间倒退被拒；状态机四个状态切换；hold；目标被遮挡时云台不 release。
- rmuc2025 追击仿真：目标沿 154 m 路线以 1.2 m/s 移动，机器人最高 2.5 m/s、用 JPS 追向 goal，10 Hz 规划。相机距离内可见时间占比：**本模块 100%**，纯弹簧点（只查碰撞）67.9%。`update` 单次最长 0.019 ms。

复现命令（gcc，构建目录可放在任意可写位置）：

```bash
R=/home/xyt/map/src/MA_Nav; B=$R/.cache/tt_build
source /opt/ros/humble/setup.bash
cmake -S $R -B $B -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_C_COMPILER=gcc -DCMAKE_BUILD_TYPE=Release
ninja -C $B path_planning 3d_occ_map utils
libs="$(find $B -name libpath_planning.a) $(find $B -name lib3d_occ_map.a) $(find $B -name libutils.a)"
g++ -std=c++20 -O2 -DROOT_DIR=\"$R/\" -I$R/src/test/include -I$R/src/planner/path_planning/include \
    -I$R/src/map/include -I$R/src/utils/include -I/usr/include/eigen3 $(pkg-config --cflags opencv4) \
    $R/.cache/tt_test/test_target_tracking.cpp $libs $(pkg-config --libs opencv4) -lspdlog -lfmt -o $R/.cache/tt_test/tt
$R/.cache/tt_test/tt
```
