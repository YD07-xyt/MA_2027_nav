# MA_Nav 各模块设计、联动与实机部署问题审计

> 审计日期：2026-11-02。对象为当前工作区源码，而不只是 Git HEAD。基线 HEAD：`4b4bcb0d12578574a714878a5f6216b244bcfc33`。
> 本文是问题文档，不是已实施的改造方案，也不是实机安全认证。仅静态读码、配置解析与几何计算；未编译、未启动节点、未回放 rosbag、未连接机器人。
> 用户已有地图配置、A* 头文件、测试与图片改动保留不动。第三方目录不是全部生产链；本文不把第三方方案或历史模块当成当前已启用功能。
> 审计期间检测到外部版本漂移：HEAD变为`c9d1aa2a6af5479bf8840052990d94aa917b5126`，并新增轨迹优化测试及输出。该提交非本次审计执行；已读其变更范围并复核当前测试CMake。主导航/优化/控制源码不在该次外部提交变更列表中。新测试只做有限静态抽查，未运行，不据此判断性能或安全通过。

## 1. 总体结论与阅读约定

项目已有较完整的技术链：ROG 概率占据 → 持久地面/地形判定 → 2D 栅格与 ESDF → JPS 搜索与后处理 → 两阶段五次样条优化 → 轨迹投影跟踪与线性 MPC；并具备先验地图、隧道语义、折叠事件、反向刹车等比赛场景功能。

**主要风险不是缺少某个更先进的算法，而是这些模块尚未形成可证明一致的执行安全闭环。** 例如：搜索路径安全不等于最终样条安全；样条安全不等于真实底盘能执行；持续发指令不等于传感器仍新鲜；“导航成功”也不等于已经停车和解除旧轨迹。单独优化搜索耗时或 ESDF 权重不能消除这些风险。

### 1.1 证据等级

- **[事实]**：当前源码/配置直接支持。事实只说明代码行为，不代表实机已发生事故。
- **[推导]**：由代码、参数或几何模型推出的条件性后果，需要实验确认影响大小。
- **[待核实]**：仓库入口无法证明的上游、底盘、安装或外部系统契约；不能当成已有保障。

### 1.2 优先级

- **P0：实机运行前必须闭环**——失效后仍可能运动、碰撞/跌落、执行器未就绪仍通行、数据失真却无可靠降级。
- **P1：比赛稳定性关键**——易造成规划失败、轨迹跳变、错误任务反馈、地图失真或不可复现。
- **P2：维护与性能债务**——影响定位问题、可移植性、性能上限和后续功能集成。

P0 是条件性风险等级，不是声称所有场景必然出事故。本文中的“建议”均未执行；任何修改比赛主逻辑、参数、QoS、启动组合前仍需用户明确确认。

### 1.3 最需要先处理的六个闭环

| 闭环 | 当前缺口 | 对应问题 |
|---|---|---|
| 传感器失效 → 限速/停车 | 控制端没有 odom/cloud 新鲜度门控；建图时的 odom 超时不是控制超时 | ROS-01、ROS-02 |
| 新障碍 → 旧轨迹失效 → 安全停止 | 重规划失败返回，但控制仍持有旧轨迹；无独立执行轨迹碰撞屏障 | FSM-01、FSM-02 |
| 优化成功 → 轨迹验收 | `success` 主要代表样条初始化成功，未接入最终连续轨迹硬验收 | OPT-01 |
| 规划动力学 → 控制 → 底盘 | 10 ms 回调使用 50 ms 模型的下一步速度；刹车 6/10 与 MPC 3 不一致 | CTRL-01、CTRL-03 |
| 低矮通道 → 折叠完成 → 放行 | 事件时间/重规划游标存在问题，且没有执行器完成反馈 | ACT-01～ACT-03 |
| MID360 观测 → 未知空间 → 可通行 | 近场下视不足；2D 抹去未知；种子/遗忘可能将无观测转为可通行 | MAP-01、MAP-02、TER-01、HW-01 |

## 2. 当前真正运行的链路与模块边界

### 2.1 启动与参数

[启动文件](</home/xyt/map/src/MA_Nav/launch/run.launch.py#L9-L49>)只启动一个 `ma_nav` 可执行程序，并通过 ROS 参数传入 planner 配置路径。入口构造 `GlobalPlanner2d` 后调用执行器 `spin_some()`，见[入口](</home/xyt/map/src/MA_Nav/src/ma_nav.cpp#L5-L26>)。

- 默认[规划配置](</home/xyt/map/src/MA_Nav/config/planner.yaml#L1-L20>)为 `mapping_model: true`；这影响先验加载、动态障碍遗忘与 2D 持久化，但并没有禁止接收目标、规划或控制。
- LIO、MID360 驱动、定位初始化/重定位、底盘通信、电机安全控制、折叠执行器、裁判系统、行为树**均未由该启动文件启动**。
- `is_minco: false` 被读取，但当前主链仍使用 MA spline；不能据该配置认定优化器关闭。
- 目标跟踪源码被路径规划库收集编译，但当前 ROS 节点没有构造/调用 `TargetTracker`；它不是 `/goal_pose` 当前规划链的一环。
- 老的 spline、GCOPTER 类型、yaw 接口和大量第三方目录的存在不等于运行时采用了对应功能。本文重点是当前 MA spline + OMNI_XY + MPC 主链，不对所有第三方代码做逐行安全结论。

### 2.2 数据流与周期

```text
外部 LIO /lio/base_odom ──→ current_pose / current_XYTheta
                         └→ ROG robot_state（接收墙钟时间）
外部 LIO /lio/cloud_world → 点云转 PointXYZI
                         → 取最近一次收到的 odom 当射线位姿
                         → ROG updateProbMap → GroundLayer → TerrainAnalyzer
                         → 动态/持久占据与全局先验 max 合并
                         → 2D ESDF → 发布 GridMap 复制快照
/goal_pose ──→ goal_pose
33 ms 规划回调 → FsmReplan → JPS → 路径简化/时间分配
             → MA spline 两阶段优化 → 轨迹接口 + 折叠事件 + MPC 引用
10 ms 控制回调 → 投影游标/刹车时间 → MPC(N=20, dt=0.05)
              → 取预测状态[1]的世界速度 → yaw 旋转 → body Twist
10 ms 可视化回调（按配置限到约30 Hz）→ 地图/ESDF/ROG 查询与发布
```

关键证据：[ROS 构造与回调绑定](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L27-L130>)、[MaMap 更新](</home/xyt/map/src/MA_Nav/src/map/include/map/ma_map.hpp#L97-L190>)、[规划调用](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L13-L208>)。

| 输入/输出 | 当前约定 | 未闭合之处 |
|---|---|---|
| 点云 | SensorDataQoS；期望世界系、可用于真实射线的当前扫描端点 | 时间戳未用于同步；是否累积地图未核实 |
| odom | Reliable `QoS(10)`；速度在代码中按世界系解释 | 上游若 BestEffort 不兼容；速度语义与 child frame 不可只靠名称判断 |
| goal | Reliable `QoS(10)`；坐标按数值直接使用 | 不做 TF、不拒绝 frame 错误、无目标 ID/取消协议 |
| cmd_vel | Reliable 深度10、无时间戳 Twist；输出机体系 | 无有效期/仲裁/下游超时契约证据 |
| fold | Bool 深度10、单次事件 | 无完成/失败反馈、无命令 ID |
| nav_feedback | Int16 0/1/2/3 | 不携带目标 ID、失败原因或事件时间 |

## 3. ROS2 集成、时间与运行调度

### ROS-01 [P0，事实+待核实] 控制使用陈旧状态，无感知健康门控

**证据**：[odom 回调](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L447-L491>)只更新数值；[控制入口](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L131-L207>)只判断 optional 和轨迹有效，没有检查 odom/cloud 时间、定位质量、协方差或最新地图年龄。`MaMap::update_cloud` 的 0.2 s 检查只决定点云是否建图，见[接收过滤](</home/xyt/map/src/MA_Nav/src/map/include/map/ma_map.hpp#L174-L190>)。

**后果**：拔掉雷达/停止 LIO，已经存在的 pose 与轨迹仍可让控制器持续输出。下游即使有“未收指令超时停车”，这里仍持续发布时也不一定触发。新点云被丢弃不等于旧地图被禁止使用。

**约束/建议**：先明确外部急停和底盘 watchdog 的真实行为；把定位有效、odom 年龄、点云/地图年龄、轨迹年龄分别作为执行条件，不用一个“收到过数据”标志替代。失效应有确定的停止/降级状态与重新放行条件。

**验收**：分别冻结 odom、冻结 cloud、保留回调但让 header 时间戳不再增长、LIO 输出 NaN/跳变，记录最后健康数据到实际停止的最坏时延与制动距离。

### ROS-02 [P0，事实+推导] 点云与射线位姿没有时间同步

**证据**：[map_callback](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L483-L491>)没有使用点云时间戳；[update_cloud](</home/xyt/map/src/MA_Nav/src/map/include/map/ma_map.hpp#L174-L188>)将最近一次收到的 ROG pose 与当前云绑定；[ROG 状态](</home/xyt/map/src/MA_Nav/src/map/src/3d_occ_map/rog_map.cpp#L266-L272>)记录的是接收墙钟。

**后果**：点云已经在世界系并不代表能用任意时刻雷达原点做清空射线。3 m/s 下 0.1 s 错位是 0.3 m，接近 0.35 m 安全阈值；转动时杆下净空、地面高度和遮挡清除也会失真。排队的旧 odom 刚收到时会被当成“新鲜”。

**建议/验收**：确认点云 stamp 是扫描起/止/中时刻，上游是否逐点去畸变，缓存并按同一时基取位姿；模拟固定时差、乱序、DDS 堆积和急转，检查障碍边缘偏移、误清空以及控制响应。不得仅放宽 odom_timeout 宣称问题解决。

### ROS-03 [P1，事实] frame 不一致只告警、不转换、不拒绝

**证据**：[frame 检查与各回调](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L434-L539>)；空 frame 甚至不会触发告警，clicked point 也没有同类检查。控制器注释按特定 Super-LIO 的世界速度解释 odom，见[速度处理](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L175-L193>)。

**后果**：RViz Fixed Frame 为 map、目标却被当 world；或更换标准按 child frame 表达速度的 odom，系统不会自动正确工作。只改发布 frame 字符串不是坐标变换。

**建议/验收**：明确 world/map/odom/base/lidar 的唯一 TF 所有者、速度表达坐标系和定位重置行为；用 yaw=90° 的直线运动、非零 TF 目标和不同 frame 输入验证方向。未知 frame 应拒绝或经过有时效的真实 TF。

### ROS-04 [P0/P1，事实+版本条件] 8 线程配置没有证明回调并行，长任务可阻塞控制

**证据**：[入口](</home/xyt/map/src/MA_Nav/src/ma_nav.cpp#L16-L25>)构造 MultiThreadedExecutor 却循环调用 `spin_some()`。ROS 2 Humble 的 [Executor 实现](https://raw.githubusercontent.com/ros2/rclcpp/humble/rclcpp/src/rclcpp/executor.cpp)在 `spin_some_impl` 中直接执行回调；[MultiThreadedExecutor 实现](https://raw.githubusercontent.com/ros2/rclcpp/humble/rclcpp/src/rclcpp/executors/multi_threaded_executor.cpp)在 `spin()` 才创建工作线程。目标 ROS 发行版/补丁版本仍需现场核实。

**后果**：不能根据 `num_threads:8` 或不同 callback group 认定控制已经与地图/规划隔离。JPS 的[调用超时](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L112-L120>)为5000 ms；大地图查询、样条求解、地图保存均可能延迟控制。1000 Hz 外循环也不能保证100 Hz控制。

**约束/建议**：不要直接换成 `spin()` 就宣布修复；先核实实际线程 ID 和调度，再处理下条共享状态问题。需要闭合“最大阻塞时间小于底盘安全预算”的契约，而非只报平均耗时。

**验收**：在目标平台记录 map/planner/control/viz 回调线程、开始结束时间、控制间隔 p99/最大值；无路搜索、RViz订阅未知云、保存地图时重复测量。实测前不得宣称具备实时隔离。

### ROS-05 [P1，条件性事实] 真正启用多线程后，共享状态尚无完整同步

**证据**：[共享成员](</home/xyt/map/src/MA_Nav/src/ros2/include/ros2/ros2_node.h#L68-L160>)与[读写回调](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L270-L410>)。规划组写 `ma_traj_interface_`、MPC轨迹引用、fold数组、游标、急停optional；控制组同时读写；地图组写 pose、mapInitialized。局部 map_mutex 并没有包住这些状态。

**后果**：当前串行入口下不应写成“数据竞争已复现”；但若更换执行器驱动方式，会存在 shared_ptr变量本身、optional和vector并发读写的未定义行为风险。2D地图快照只解决地图内容，不解决整机状态同步。

**建议/验收**：先确定状态所有者和交接点；pose快照、轨迹及其事件/版本应一致发布。经授权采用真实并发后做并发静态审查与 sanitizer/压力实验，测试高频目标切换与地图更新；不扩大为无边界重构。

### ROS-06 [P1，事实+推导] QoS、时钟和消息新鲜度没有统一契约

**证据**：[订阅配置](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L51-L102>)；规划/控制采用steady_clock，ROG占据遗忘和odom接收采用系统时间；ROS header时间基本未进入有效性判断。

**后果**：可靠odom订阅可能无法匹配BestEffort发布；队列深度10在拥塞时增加状态年龄；rosbag暂停或倍率改变时墙钟遗忘仍推进，离线结果不等价于实时。时间回拨对遗忘也有影响。

**建议/验收**：记录各消息期望QoS、真实端点QoS、stamp含义、运行/回放时基；通过端点信息和固定倍率回放确认。不能把本次静态解析当作QoS连通性验证。

## 4. ROG、2D地图与ESDF

### MAP-01 [P0，事实] 3D未知在2D层被压成自由，先验缺失时尤其危险

**证据**：[MaMap 初始化/合并](</home/xyt/map/src/MA_Nav/src/map/include/map/ma_map.hpp#L70-L168>)中占据矩阵初始为0，导航动态层每帧setZero，仅把terrain障碍写1；没有把未知/未观测/不可达作为独立2D状态。地图加载失败只返回，见[加载入口与失败处理](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L612-L723>)。

**后果**：地形“不给空闲证据”并不等于规划器“不许走”；对初始化就是0、导航模式每帧清0的格，未观测仍可通行。`reach`主要约束空闲证据的生成，不是规划域硬限制。先验路径无效时可能以无先验的乐观地图继续运行。

**建议/验收**：明确比赛是否允许未知探索；分清先验自由、实时已观测自由、未知、不可达。导航缺少必需先验/覆盖范围不足时应有明确放行门槛。对被屏风遮住区域、窗口外和下坡无回波区域检查规划结果。

### MAP-02 [P0，事实+推导] 导航遗忘会把“未再次看到”转成可通行

**证据**：[配置](</home/xyt/map/src/MA_Nav/config/map.yaml#L48-L52>)导航遗忘2 s、保护半径1.5 m；[遗忘实现](</home/xyt/map/src/MA_Nav/src/map/src/3d_occ_map/prob_map.cpp#L829-L856>)按占据最后命中时间resetCell，变为UNKNOWN；MAP-01说明UNKNOWN在2D未被阻挡。遗忘每5次更新执行，非独立严格周期。

**后果**：盲区外被遮挡但没移动的新障碍，2 s后也可能消失；“先验兜底”仅适用于先验已有且配准正确的静态障碍。保护圈内移动障碍反而可能长期残影；点云完全中断时遗忘也不会独立运行，不能把2 s当硬实时清除保证。

**建议/验收**：把有射线确认的清除与时间遗忘区分；遗忘到未知不应自动获得与实测自由相同的执行权限。做“新放障碍→遮挡→等待→接近”与“近场障碍离开→无清空射线”两组对照，评价漏障与残影两端风险。

### MAP-03 [P0，事实] 地图外无碰撞，搜索边界、优化边界与执行边界不一致

**证据**：[GridMap距离/碰撞](</home/xyt/map/src/MA_Nav/src/map/src/grid_map.cpp#L121-L145>)地图外返回max距离/false；[优化代价](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp#L78-L144>)地图外不加障碍代价；[JPS入口](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/search/jps.cpp#L171-L219>)地图外起终点不拒绝，索引clamp到边界，最终又[把末点替换为精确goal](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/search/jps.cpp#L289-L295>)。重规划碰撞检查也跳过地图外。

**后果**：搜索内部越界格blocked，不代表最终世界路径仍在地图内；样条有绕出边界以避免障碍代价的激励，终点也可越界。固定2D边界与滑动3D窗口不是同一边界。

**建议/验收**：显式区分“未知世界外”和“物理禁行边界”；在线执行域原则上不能靠跳过碰撞查询放行。测试边界外goal、四条边、四个角、样条向外过冲和定位越界。

### MAP-04 [P1，事实+推导] ESDF初始化、边界索引与插值不具备保守距离保证

**证据**：[初始化与setMap](</home/xyt/map/src/MA_Nav/src/map/src/grid_map.cpp#L3-L51>)距离初始全0，只在占据改变时重算：第一帧全自由地图仍保持全0，可能所有位置都小于0.35而规划失败；[边界/坐标索引](</home/xyt/map/src/MA_Nav/src/map/include/map/grid_map.hpp#L53-L95>)闭区间上边界可能映射到N，负向刚出界采用int截断也可能落第0格；[3×3二次插值](</home/xyt/map/src/MA_Nav/src/map/src/grid_map.cpp#L54-L118>)无距离保守夹紧。

**后果**：空地图/最后障碍移除、边缘查询和梯度零点可能有不同语义；二次插值可过冲，格心ESDF也不是自动等于机器人到体素实体边缘的真实净空。不能仅凭连续可导就视为安全距离下界。

**建议/验收**：建立全空、全占据、单点、单格薄墙、窄缝、贴边、正负坐标等数值金标准；检查有限性、符号、梯度差分与插值误差上界，再决定安全阈值补偿。这里不声称所有插值位置都已测出过冲。

### MAP-05 [P1，事实] 先验地图只能加障碍，无法在线纠正旧地图中的假障碍

**证据**：[占据合并](</home/xyt/map/src/MA_Nav/src/map/include/map/ma_map.hpp#L135-L168>)为dynamic与global的逐格max。先验格为1时，实时自由和遗忘不能将它清除。

**后果**：建图时把人/机器人或假地面存成障碍、场地改动、先验定位有偏移，均可能持续无路；反过来先验自由也不能证明当前无新障碍。长期层、动态层与地形记忆的更新/失效责任不同。

**建议/验收**：先建立地图采集、清场、人工校验、版本与定位一致性要求；任何允许清除静态先验的策略必须单独确认，不可直接删除兜底。用旧障碍移除/新增场景分别验证。

### MAP-06 [P1/P2，事实] 射线数据有效性与虚拟平面求交存在边界问题

**证据**：[raycastProcess](</home/xyt/map/src/MA_Nav/src/map/src/3d_occ_map/prob_map.cpp#L678-L754>)在坐标转格前没有统一allFinite过滤；越过虚拟顶/底时使用 `origin + normalized(delta) * pc/dz`。

**推导**：若 `delta=p-origin`，与水平平面正确交点应为 `origin + delta*(pc/delta.z)`；现式多除一个delta长度，长度不是1时交点通常不在目标平面。仅对实际落到虚拟顶/底之外的输入触发，不能泛化为所有射线错误。NaN坐标参与索引转换的结果也不应依赖平台行为。

**建议/验收**：独立几何单元样例验证交点z与射线方向，覆盖水平近退化、NaN/Inf、越界与零距离点；不需要先跑机器人才能确认数学不一致。

### MAP-07 [P2，事实+推导] 窗口、分辨率与多次拷贝有明显资源成本

**证据**：[配置](</home/xyt/map/src/MA_Nav/config/map.yaml#L1-L40>)3D nominal 10×10×4 m、5 cm；[2D范围](</home/xyt/map/src/MA_Nav/config/map.yaml#L105-L125>)28×20 m、5 cm；[快照复制](</home/xyt/map/src/MA_Nav/src/map/include/map/ma_map.hpp#L280-L283>)每发布复制GridMap；[PathPlanning::set_map](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/path_planning.cpp#L7-L10>)又分别复制给A*/JPS/后处理。

**量级**：名义3D体素约320万，实际受奇数格和膨胀对齐影响；2D约224000格。GridMap的char占据、double距离、uint8语义基础数据约2.24 MB/份，不含分配与其他矩阵。2D障碍有变化时整图ESDF重算；terrain对4万列分配高度容器，地面层还遍历每条射线。

**后果/建议**：CPU/内存带宽及分配抖动可能比搜索堆优化更重要；配置ray_range=30 m不会突破局部约5 m半边长。先实测整帧预算、复制峰值和内存，按问题证据做局部优化，不建议未经验证替换整套地图。

## 5. 持久地面层与地形分析

### TER-01 [P0，事实+推导] 启动种子与走过足迹把机械假设写进了地形事实

**证据**：[seedDisk](</home/xyt/map/src/MA_Nav/src/map/src/ground_layer.cpp#L41-L104>)首帧按1.5 m圆盘估计平面；无邻域观测时用base z+offset，当前offset=0且注释未核实。footprint标记TRAV；[可达生长](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp#L329-L356>)清除TRAV的ground_obs，SEED参与生长时可跳过边高差检查。

**后果**：机器人从高台边缘/坡面启动，盲区seed可能跨过真实落差；身体中心经过不代表以0.2 m圆盘覆盖的每个格都被足端/轮胎验证，尤其四足支撑跨缝、悬空或打滑时。“低置信”方差存在，但下游没有据方差收紧通行。

**建议/验收**：实测base到接地面的定义；启动种子仅是先验假设，应限制放行权重/范围，明确起步区必须人工确认条件。测试台边启动、跨坑、侧滑和被搬运后重启，检查TRAV是否错误消除边缘风险。

### TER-02 [P0，事实] 未见地面/下降边缘不能被等同于没有危险

**证据**：[配置](</home/xyt/map/src/MA_Nav/config/map.yaml#L96-L103>)`drop_edge_enable:false`；[Terrain最终判定](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp#L420-L449>)无参考地面/无占据不下结论；MAP-01说明2D仍可能自由。

**后果**：当前算法擅长已有上下表面证据的台阶/坡度，不代表能发现所有负障碍。MID360低头角不足和平台边沿遮挡恰好使下方地面无观测，可能没有占据物可供“障碍”生成。

**约束/建议**：不能简单打开drop_edge就宣称解决，它可能误挡合法下坡。需要已知场地禁行层/可信地面覆盖、近场下视传感器或不确定区低速停车策略组合，按真实场景授权取舍。

**验收**：平地坑洞、台边向外、合法下坡入口、亮/黑地面无回波分别测试，报告漏检和误挡；无实际跌落保护时不得高速验证。

### TER-03 [P0，事实+推导] 悬空判据放行门槛可低于robot_height

**证据**：[floating判定](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp#L111-L146>)按`min_overhang_clearance`及下方free证据识别；[floating直接放行](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp#L412-L418>)未再次要求robot_height；[另一放行分支](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp#L452-L463>)在净空小于robot_height但≥min_overhang_clearance时也可放行。当前阈值0.225与0.24 m。

**后果**：射线证实有空气只能证明无遮挡，不证明空气高度够机器人通过。代码里有意识用该分支补偿量化/拖点，但它实质把最小净空下调。若物理高度确实0.24 m，真实0.23 m间隙不能因为有空闲体素就安全。

**建议/验收**：分清“真实净空下界”“体素中心偏差补偿”和“机器人包络”；对已测量0.22/0.225/0.23/0.24/0.25 m通道按不同姿态与速度标注，不仅检查与旧结果是否一致。

### TER-04 [P1，事实+推导] 单层高度与最低点融合不足以表达多层和不确定性

**证据**：[GroundLayer量测](</home/xyt/map/src/MA_Nav/src/map/src/ground_layer.cpp#L128-L239>)每帧每格取最低点，方差加权，升降按票；[地形引用](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp#L151-L167>)持久地面只往低替换。Cell虽有var、last_hit，但地形通行大多只读h/flags，见[可达层读取](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp#L234-L246>)。

**后果**：桥面/桥下、台面/低地、透明/反光异常低点、动态物体最低表面和旧地面冲突时，单个h不能表达多假设；融合方差逐步收缩而缺少明确的时间老化，可对历史误判过度自信。票数也未等同严格连续有效帧：格中间未观测时up/down不必清0。

**建议/验收**：先界定比赛场景是否禁止多层；用带标注的原始数据核查最低点异常、地面重访和观测中断。方差、年龄、SEED/OBS不能在可通行消费端被无差别对待，不必立刻引入完整复杂概率框架。

### TER-05 [P1，事实+推导] 姿态跳变门控、坡度判据与帧率耦合

**证据**：[姿态门控](</home/xyt/map/src/MA_Nav/src/map/src/ground_layer.cpp#L107-L125>)帧间角度>2°后保持4帧（配置）；它只跳过地面层量测/上界，ROG与地形更新仍继续。[坡度/台阶门槛](</home/xyt/map/src/MA_Nav/src/map/src/terrain_analysis.cpp#L249-L307>)为tan(25°)×格间距离+0.03 m，5×5拟合需12点。

**后果**：同一运动在10/20/50 Hz下触发率和冻结时长不同；持续振动可反复冻结地面，旧地面与新ROG混用。在5 cm正交格下边高差门槛约5.33 cm，对角约6.30 cm，不是注释中的统一5 cm台阶阈值。稀疏点不足时陡坡拟合不触发。

**建议/验收**：记录真正传入帧率、门控占比、冻结时长、坡度误差；分辨率改变时重新评估物理门槛。不能通过无限调大票数/保持帧数替代点云-位姿同步。

### TER-06 [P1，事实] 24 cm机身与25 cm限高的安全裕量不足以从当前表示中证明

**证据**：[高度配置与注释](</home/xyt/map/src/MA_Nav/config/map.yaml#L54-L74>)将robot_height设为0.24以通过约0.25净空；体素5 cm；MID360官方距离精度见第10章。

**后果**：名义剩余1 cm小于体素边长，且还有地面估计、雷达外参、安装、pitch/roll、步态起伏、结构挠曲与执行器未完全收拢误差。不能用“阈值小于杆高”当作实机几何证明；也不能把多个误差简单线性或独立高斯假设混算。

**建议/验收**：按展开/折叠、姿态和步态实测最大动态高度包络；给净空上下界与允许速度。净空不足以覆盖最坏包络的场景应列为受限工况，而不是继续减小robot_height。

## 6. JPS/A* 与路径后处理

### SEARCH-01 [P1，事实+推导] 点安全不等于边安全，角穿越与精确端点替换需单测

**证据**：[A*邻居](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/search/astar.cpp#L122-L147>)对角扩展主要判目标格，不检查两侧格或整段连续净空；[JPS射线](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/search/jps.cpp#L88-L120>)也主要查询走过格心，终点格可在blocked前返回；[后处理](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/post_processing.cpp#L4-L86>)用Bresenham，不是覆盖所有相交格的supercover，也不会因为原路径某段不安全就必然拒绝输出。

**后果**：0.35 m安全阈值会缓解很多正常细格角点情形，但不能作为所有阈值、插值和端点情况的证明。特别是参数收紧、栅格精确goal安全而中心不安全、细薄障碍和斜向贴边情况。

**建议/验收**：统一顶点/边的碰撞语义，保留有效比赛行为前提下补充金标准样例；对角两个占据角、端点半格偏移、同格起终点、Bresenham漏侧格、A*/JPS成功率与最短性逐项对照。不把“JPS理论最优”直接套到当前变体。

### SEARCH-02 [P0/P1，事实] 起点安全推移与JPS吸附缺少机器人到新起点的可执行连接

**证据**：[FSM安全推移](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L65-L72>)和[get_safe_pos](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L335-L369>)最大推1.5 m；[JPS吸附](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/search/jps.cpp#L221-L239>)也可另找根；[重建路径](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/search/jps.cpp#L289-L295>)吸附后不恢复真实起点。

**后果**：新起点安全不证明机器人能安全到达；可能跨薄墙或跳到不同连通域。两层各自修正也缺少统一偏移预算。优化器固定的是新端点，控制器会从实际机器人位置追过去，补接段未被验证。goal被推移时还可能超过0.3到达半径，任务永远不达。

**建议/验收**：修正必须返回原点、新点、原因、连接轨迹和是否满足原任务；不能靠静默挪点把搜索失败转成功。用窄凹槽、墙内定位、障碍另一侧自由格和目标贴墙验证。

### SEARCH-03 [P1，事实+推导] 入口前低速只存在于初始时间分配，优化后可被改掉

**证据**：[fold速度限制](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/post_processing.cpp#L395-L423>)入口前设vmax=0.05；[样条变量掩码](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h#L226-L249>)允许内部航点和时间变化，优化代价没有对应位置相关fold限速或等待执行器完成约束。

**后果**：前端保留1秒折叠时间，不代表最终轨迹仍保留；入口接近起点时prep区甚至不存在。最少5最多16段的均匀重采样还可能稀释很短的关键转角和准备段。

**建议/验收**：对最终轨迹重新验收折叠准备距离/持续时间，而非只验初始路径；必要时用明确停等联锁。记录样条优化前后在隧道入口的速度和折叠完成余量。

## 7. MA Spline 轨迹优化

### OPT-01 [P0，事实] 优化器“成功”与碰撞/动力学可行性没有硬绑定

**证据**：[optimize_xy](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h#L237-L264>)第二阶段失败保留第一阶段，最终success由isInitialized决定；[碰撞检查函数](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h#L148-L165>)有定义但全仓当前调用链没有调用；[主调用](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L193-L208>)仅检查success。避障、速度、加速度均为软代价。

**后果**：收敛/初始化成功不是硬安全证明；Stage1较低障碍权重下的回退也直接被接受。离散积分的采样点不覆盖连续曲线所有位置/导数极值。原折线安全也不能阻止自由内部航点和五次段切角。

**约束/建议**：独立验收最终“刹车段+样条+切换连接”的执行轨迹，覆盖界内、净空、速度/加速度/必要jerk与数值有效性；验收失败不发布新轨迹。简单启用现有0.05 s采样仍不是完整证明：3 m/s时采样间距约15 cm，需速度/曲率自适应或分段包络/连续极值保障。

**验收**：薄墙、S弯、窄门、过短段、很高初速、Stage2失败、边界外过冲等场景，记录最终最小净空/最大动力学量，不以优化cost大小代替。

### OPT-02 [P1，事实+数学判断] Stage2返回的避障梯度不再是返回代价的解析梯度

**证据**：[Stage2代价与梯度](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp#L94-L142>)cost继续使用真实safe-d，gp却去掉速度切向分量并做归一化/增强；返回gv没有加入该人为方向依赖的相应导数。

**后果**：这可作为经验搜索方向，但不能再视作原标量目标的严格梯度。L-BFGS曲率更新和线搜索依赖的一致性被削弱，可能使Stage2失败、震荡或终止判据意义改变；不能因为避障代价连续就声称优化方法仍有相同收敛解释。

**建议/验收**：固定样条变量做中心差分梯度校验，分Stage1/Stage2分别报告；记录Stage2失败比例与最小净空。若保留经验方向，应明确它是启发式并依赖独立可行性验收，而不是再叠补丁隐藏梯度不一致。

### OPT-03 [P1，事实+推导] 规划耗时与有效期缺少可取消、可过期的发布契约

**证据**：[LBFGS配置/调用](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h#L311-L331>)按迭代数而非明确墙钟截止；FSM整个调用使用起始pose和map快照，[结果发布](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L298-L340>)未核对结果是否已过期/目标版本是否变化。

**后果**：串行时求解阻塞新数据；真正并行时求解期间机器人/地图/goal可能变化。最近点投影能修正游标，却不能证明新轨迹在当前状态仍可连接和安全。

**建议/验收**：给规划请求/轨迹明确版本、观测时刻、发布时有效域；先明确过期拒收/切换条件，再考虑取消。测最坏求解时间与出结果时机器人移动距离。

## 8. 重规划状态机与任务生命周期

### FSM-01 [P0，事实] 新规划失败不撤销旧轨迹，碰撞后仍可能继续执行

**证据**：[FSM触发碰撞后求解失败](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L41-L46>)、[失败返回](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L115-L120>)；[ROS失败分支](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L299-L321>)直接return，不清`ma_traj_interface_`，控制端不读取nav_state作为使能。

**后果**：已经判出原路径碰撞，但新路求不出时，旧轨迹仍有效，控制继续追踪。20 s后设FAILED也没有自动停止；这是比“规划成功率低”更高优先级的状态所有权问题。

**建议/验收**：明确旧轨迹可继续执行的前提只能是当前执行前缀仍安全且能安全刹停，不是“上一次成功过”；碰撞/无路/优化失败分别有失效动作。动态障碍封路且无可替代路线时，必须验证实际停在碰撞前。

### FSM-02 [P0，事实] 在线重检的是前端折线，不是控制正在执行的样条

**证据**：[check_collision](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L323-L333>)只检查planning_traj的optimized_path和raw_path，阈值[硬编码计算](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L32-L42>)为max(0.05,0.2×safe)，当前0.07 m；没有相同逻辑校验最终样条/刹车段。控制MPC也无障碍约束。

**后果**：最终样条向折线侧边偏移后，新障碍正好落在样条上而不靠近折线，可能不触发重规划。0.35 m的搜索/优化安全距离与0.07 m在线触发距离不在同一安全口径；不能只把0.07叫“硬净空”而忽略机身半径。

**建议/验收**：以当前执行轨迹剩余前缀和实际足迹作为首要监控对象，覆盖停止距离并明确跟踪误差预算。把障碍放在样条偏离折线处与刹车段上，检查触发时刻和停车余量。

### FSM-03 [P1，事实] 热启动忽略真实速度，已配置的有效期/连续性没有生效

**证据**：[热启动](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L76-L109>)扫描旧参考最近点取计划v/a，无旧轨迹时静止起步；当前[参数读取](</home/xyt/map/src/MA_Nav/src/ros2/include/ros2/config.hpp#L211-L223>)的`minco_traj_validity_duration`和`minco_traj_continuity_threshold`未在该主链消费。`min_replan_interval`在YAML有值但没有对应读取/执行。

**后果**：机器人打滑、被撞、堵转、LIO跳变、暂停后恢复或新目标反向时，计划速度与真实速度可能显著不同；旧参考距离再远、年龄再长也可能参与热启动。1秒前路径参考本身连续不代表机器人状态连续。

**建议/验收**：区分抗噪状态估计与用参考替代反馈；给热启动设年龄/距离/速度一致性与版本条件。测试轨迹旁移>0.3 m、冻结2 s以上、被外推移动和中途反向；不能只调小projection分辨率。

### FSM-04 [P1，事实] 失败时内部结果可能混合新折线与旧样条

**证据**：[更新顺序](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L167-L207>)先覆盖`result_.planning_traj`，后求样条；优化失败时旧`ma_spline_traj`仍在，old_goal与时间只在后续成功时更新。

**后果**：下次碰撞/偏差判定、热启动以及外部仍执行的样条不再共享一个版本。“结果对象存在”不是事务性成功；日志看到的新折线也不一定是当前执行路径。

**建议/验收**：候选结果应在全链验收后一次性交接，保留最后已接受轨迹与失败候选分离。注入优化失败，核对折线、样条、地图版本、goal和轨迹ID的一致性。

### FSM-05 [P1，事实] 到达、失败、新目标与反馈生命周期不完整

**证据**：[到达分支](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L282-L306>)按位置距离<0.3即SUCCESSED、只reset goal；不清轨迹、不检查速度/yaw，也不发停车。成功规划后plan_start_time每次回调[重置](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L324-L340>)；失败分支设置FAILED却直接return，[实际反馈发布](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L408-L410>)通常不执行。目标[精确去重](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L525-L539>)使用old_goal；同位置新朝向或任务重发被丢弃。

**后果**：到达半径内仍运动就提前报成功，旧轨迹仍可能向原终点走；同点重试不能重新发起；卡住但参考路径仍可用时不会因为总任务超20 s失败。规划层SUCCESSED是“路径可用”，节点层SUCCESSED是“到达”，两个语义容易误用。`plan_start_time_.value()`目前通常由目标回调初始化，不应无证据声称默认就会抛异常，但防御契约确实脆弱。

**建议/验收**：明确请求接收、执行、到达且停止、失败/取消与重试生命周期；反馈应关联目标ID并可达所有终态。测试同点重发、只改yaw、到达时残速、20 s堵转与持续规划失败。

### FSM-06 [P1，事实] 辅助入口和初始化语义与主链不一致

**证据**：[FsmReplan带参构造](</home/xyt/map/src/MA_Nav/src/planner/replan/include/replan/fsm_replanner.h#L45-L55>)未像set_param那样给ma_opt设配置；[one_plan](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L212-L253>)与主plan在速度、model、结果状态检查上不同。[MaSplineInput](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h#L65-L78>)的model无默认值，[转换函数](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h#L147-L191>)也未设置它；主链主动赋OMNI_XY，one_plan则未赋。`old_goal_pose_`持有的RobotState位置在首次读取前没有显式初始化，见[成员](</home/xyt/map/src/MA_Nav/src/planner/replan/include/replan/fsm_replanner.h#L65-L76>)与[首次比较](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L35-L39>)。

**后果/建议**：主入口当前可避开部分问题，不能把未调用入口的问题误写为当前每帧触发；但测试/复用新入口时可能读取未初始化字段或用错优化配置。分别定义构造/方法前置条件，覆盖首次规划和所有公开入口。

## 9. MPC、跟踪游标和折叠执行

### CTRL-01 [P0，事实+推导] 10 ms循环反复执行50 ms预测增量，物理限幅不再按预期成立

**证据**：[控制timer](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L80-L89>)10 ms；[配置](</home/xyt/map/src/MA_Nav/config/planner.yaml#L115-L137>)dt=0.05；[模型](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/mpc.h#L120-L133>)v_next=v+u×dt；[状态与输出](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L175-L249>)下一轮将上轮预测v_next当初始速度，再输出新的v_next。

**推导**：若连续每10 ms都取u=3，则命令每次增加0.15 m/s，命令斜率约15 m/s²，而不是3。实际底盘是否跟得上取决于执行器限幅/动力学，不能把该命令斜率当已测实际加速度。调度不稳定时比例也变化。

**建议/验收**：预测步长可以与控制频率不同，但执行/状态传播必须使用正确的实际时间或明确控制保持策略。记录真实dt与命令差分加速度，比较模型预测、里程计与底盘反馈。

### CTRL-02 [P0/P1，事实+推导] 速度状态长期使用上次指令，真实闭环能力被削弱

**证据**：[速度初值选择](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L175-L198>)只在第一次无last_cmd时使用限幅的odom；后续使用last world cmd，且轨迹切换没有明确重置has_last_mpc_cmd。位置仍有反馈，不应称整个控制器完全开环。

**后果**：堵转时模型认为已加速，释放后突进；滑移/外力下真实动量不被状态速度体现。yaw仍跟随odom旋转输出，快速旋转与时延下世界/机体系执行误差扩大。

**建议/验收**：建立可信速度估计/滤波和执行器反馈，不把规避QP infeasible当作状态真实性保障。测试低摩擦、抵墙、架空、不同yaw高速横移，并测恢复时最大速度误差。

### CTRL-03 [P0，事实] 反向刹车参考要求超出MPC，急停按理论时间而非真实停车完成

**证据**：[刹车配置](</home/xyt/map/src/MA_Nav/config/planner.yaml#L107-L109>)6/10 m/s²，而MPC轴向限幅3；[刹车构造](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L376-L421>)按计划速度求停止点/时长；[急停处理](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L141-L165>)期限内发0，期限结束按最近样条点恢复，不检查真实速度是否已近0。[刹停点再次推移](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L140-L150>)还可能令刹车终点与样条起点不连续。

**后果**：规划声称0.75 m可从3 m/s刹停，是a=6的理想值；a=3需1.5 m，未计延迟。发零速度不是瞬时刹停。若CTRL-01让命令跳变更快，那是另一模型不一致，而非证明底盘有6/10制动能力。

**建议/验收**：按底盘、地面摩擦、坡度、负载实测可保证最小减速度建立停车包络；急停后用真实状态与安全连接条件重新放行。覆盖横移、斜向、下坡、快速反向、障碍近到不足停止距离等受保护工况。

### CTRL-04 [P0，事实+待核实] MPC失败/异常返回没有确定停车输出

**证据**：[solve失败](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L204-L219>)、[输出异常](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L253-L267>)与轨迹无效分支均直接return。ROS退出也没有显式停车握手。

**后果**：下游若保持最后一条速度会持续运动；若有watchdog，停止时延取决于它的阈值且未在本链验证。仅warn不是安全动作。

**建议/验收**：明确定义“无指令/解算失败/线程卡死/进程退出”的底盘行为；受控停止和硬件急停分层。故意造成QP不可解、数值无效、kill进程、网络断开，记录实际停止而不只看日志。

### CTRL-05 [P1，事实] 约束形状、代价矩阵与控制增量约定存在语义落差

**证据**：[MPC硬约束](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/mpc.h#L250-L272>)分别约束vx/vy和ax/ay；[优化代价](</home/xyt/map/src/MA_Nav/src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp#L146-L157>)按二维范数限3；[H构造](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/mpc.h#L179-L208>)只用Q/R对角，g却用完整矩阵；[u_prev](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/mpc.h#L524-L525>)保存但未参与首个控制增量惩罚。

**后果**：MPC允许对角合速度/加速度到约4.24，与上游范数3不一致；当前配置矩阵是对角所以非对角问题未触发，但加载器接受非对角矩阵会制造错误预期。Rd平滑了预测序列内部，不等于平滑了本次u0与上次实际控制。

**建议/验收**：统一norm/axis/body/world动力学约束含义，明确矩阵能力；测斜向最大速度与切轨迹时命令跳变量。模型前向Euler缺少位置0.5a·dt²项，也应纳入预测误差评估，不能孤立视为必须立即替换的bug。

### CTRL-06 [P1，事实+推导] 最近点游标不等于真实时间进度，容易跳段和放大折叠偏差

**证据**：[轨迹接口](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/traj_interface.hpp#L94-L137>)初始全局搜索每0.1 s，后续搜索hint−0.3到hint+3，每0.02 s；仅按位置距离，无速度方向/段ID/单调约束。

**后果**：回头线、自交、相邻并行路径和定位跳变时可能投影到未来/过去分支；游标每次最多前搜3秒不代表真实运行了3秒。它会同时影响MPC参考和fold事件，机械动作提前/滞后可能比位置误差更危险。

**建议/验收**：区分沿路径几何进度与机械动作的真实持续时间；给轨迹切换和自交段明确投影约束。用U形回头、8字、原路反向和定位瞬移测游标跳变量。

### ACT-01 [P0，事实] 新轨迹折叠事件按旧游标跳过，状态补发并未实际发布

**证据**：[切换代码](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L338-L371>)生成新fold_events后，先用旧`t_track_`跳过事件，再计算新轨迹nearest_time；补发处`publish_fold_cmd`被注释，仅修改`current_fold_state_`。控制回调真正发布则发生在[next事件触发](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L222-L240>)。

**后果**：旧轨迹走到5 s，新轨迹前1 s需折叠时，事件可能被跳过，内部记为folded但执行器从未收到。当前fold_state是“记录的命令状态”，不是物理实际状态。

**建议/验收**：以新轨迹有效游标重建事件状态，保证必要状态命令确实发布，并交由物理反馈确认。复现“旧游标大、新轨迹短且入口近”并检查底盘仍未进入前已经真实折叠。

### ACT-02 [P0，事实+推导] 折叠/展开没有完成反馈，轨迹时间不保证动作完成

**证据**：[输出接口](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L99-L102>)只有Bool publisher；[事件时刻](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L1004-L1035>)按样条入场时间提前fold_time+margin。无相应状态订阅/确认。若入口时间不足，fold时刻clamp0，不意味着会等待1秒；刹车段自身也没参与隧道检测。

**后果**：电机卡住、动作受负载变慢、接收端重启丢失单次消息、游标跳跃/快速推进时，车体可能还未收拢就进入；重规划不是机械时间重置。只加时间裕量不能处理卡死。

**建议/验收**：明确折叠状态/故障反馈、等待完成后放行和超时停车；命令应可关联目标、可幂等重发。受保护条件下测试执行器延迟、丢消息和故障；不能只验证Bool已发送。

### ACT-03 [P0/P1，事实+推导] 2D点语义不代表整机已出隧道，终点可能使展开事件永不触发

**证据**：[隧道检测](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L933-L959>)只查轨迹中心点；[展开时刻](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L1007-L1015>)为exit_time+margin；[游标上界](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/traj_interface.hpp#L121-L137>)不超过总轨迹duration。

**后果**：车中心离开不代表后缘/云台扫掠体已离开，倒车/横移更不能只按中心计算；隧道贴终点或终点在隧道内时展开事件可能晚于轨迹结束，永远不会执行。终点在隧道内保持折叠可能正确，但当前没有显式终端状态规则。

**建议/验收**：用不同形态的footprint/swept volume扩展入口出口区域，定义终点在洞内、紧贴出口、取消、倒退退出时的机械状态。展开必须以物理包络已离开为前提，不能只靠增加固定margin。

## 10. MID360 与实际部署约束

### 10.1 官方规格与计算前提

来源：[Livox MID-360 官方规格](https://www.livoxtech.com/mid-360/specs)，本次已读取正文：

- 水平FOV 360°；垂直FOV −7°～52°；近距盲区0.1 m。
- 0.1～0.2 m虽可记录点，但精度不保证；0.1～1 m低反射率或细小物体的检测效果不保证。
- 典型10 Hz、200000点/s（first return）；100BASE-TX以太网；支持PTPv2/GPS同步。
- 官方精度条件为：10 m处≤2 cm（1σ，80%反射率/25℃等条件）；0.2 m处≤3 cm（1σ）。不是所有距离、材质、姿态下的最坏误差保证。
- 测距40 m@10%反射率/70 m@80%反射率（100 klx条件）；不等于本项目规划可观测距离。
- 供电9～27 V、平均6.5 W；低温自热峰值可14 W；−20～55℃工作环境，强振动/雾/极端温度可影响性能，过温会停止。

下面几何推导假设：雷达水平正装，无遮挡，平地，雷达扫描参考水平面与地面高差h；FOV为标称范围，不保证任意帧任意方向均有稠密回波。**配置sensor_offset.z=0.194只是base到雷达的平移，不自动等于实机离地高度。** 下列h=0.194示例还需要base确实处在地面/足底坐标。

### HW-01 [P0，物理推导] 0.1 m测距盲区与约1.58 m地面盲环是两回事

最低俯角仅7°，平地首次可能相交的水平距离：

`r_ground = h / tan(7°)`。

| 雷达实际离地h | 水平正装理论地面可视近界 |
|---|---:|
| 0.194 m | 1.580 m |
| 0.300 m | 2.443 m |
| 0.500 m | 4.072 m |

可见，提高安装高度虽然减少车体遮挡，却扩大近场地面不可见区；360°只是水平方位覆盖，不是向下半球覆盖。项目seed_radius/forget_keep_radius均1.5 m，只有在某些安装高度与姿态下才与实际盲区近似，不能当通用安全半径。

对于高度为z<h的障碍顶缘，简化近界为`r_min=(h-z)/tan(7°)`：h=0.194时，5 cm障碍约1.173 m外、10 cm障碍约0.766 m外才有落入下视边界的可能。障碍从远处进入后可保留历史证据，但启动就在近场、从侧方遮挡区进入、被临时放入的障碍没有此前观测保证。

**部署建议/验收**：用实际安装CAD/照片和带角度标定的点云，绘制各方位、俯仰/横滚、折叠/展开状态的观测区域；近场低障碍和坑洞需要互补传感/碰撞保护/受限运动策略。单纯加大射线max_range不能补下视盲区。

### HW-02 [P0/P1，物理推导+待核实] 倾斜、倒装、遮挡与姿态会重分配视野

单方向下倾β，前向最低角约`−(7°+β)`，后向最低角约`β−7°`；例如下倾10°后前向看地更近，但后向最低射线可能向上，倒车/侧移安全会变差。坡上抬头与步态pitch也有相同局部作用，必须按完整三维安装旋转评估。

理想倒装会把垂直角范围相对世界水平翻到约−52°～7°，看地更近但高处/顶部覆盖变差；是否允许这样安装、具体FOV细节与机械可靠性应查目标设备手册并实测，本文不作安装许可建议。

云台、立柱、线束、装甲、外壳檐边和雷达防护罩可能遮挡扫描；雷达若装在折叠件上，折叠时外参不是固定sensor_offset，LIO外参也可能同时失效。雷达若固定在底盘，折叠件运动仍可能形成新的遮挡。

**验收**：在每种机械形态、各yaw与最大pitch/roll下放低矮黑块/细杆/高杆，记录首次可靠检测距离、持续漏帧和盲扇区。不能只拍一张静态360°点云截图。

### HW-03 [P0，物理推导] 速度上限必须来自可观测停车距离，不只来自算法参数

至少满足：

`D_required >= v*T_total + v²/(2*a_guaranteed) + D_geometry + D_uncertainty`。

- T_total应含扫描积累/传输、LIO、排队、地图、重规划、控制、底盘执行及故障检测；不能把10 ms控制timer当整链延迟。
- a_guaranteed取当前方向/坡度/摩擦/负载下可以保证的实际制动能力，不是brake_acc_max。
- D_geometry覆盖车体前缘/横向包络与传感器相对base偏置；D_uncertainty覆盖定位、感知与执行误差。

例如仅作量级演示：v=3 m/s、T_total=0.2 s、a=3 m/s²，尚未加几何与误差就需2.1 m；若a=1则需5.1 m。局部窗口轴向半边约5 m，且遮挡、下视不足可能使真正可靠观测距离远小于它；第二个例子已超名义轴向半窗口。不能因为雷达可测40 m就认定3 m/s安全。

**验收**：测每条链的端到端延迟分布、最坏连续漏检和刹车距离，按可见距离/方向设置受限工况。新障碍突然出现在物理停车距离内时，算法无法保证“必不碰撞”，应明确残余风险与硬件急停/防撞策略。

### HW-04 [P1，事实+待核实] 射线清空要求输入是当前真实扫描，不能随意换成累积地图

**证据**：ROS层直接订阅`/lio/cloud_world`并对所有端点从当前雷达原点投射，见[点云回调](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L483-L489>)与[raycastProcess](</home/xyt/map/src/MA_Nav/src/map/src/3d_occ_map/prob_map.cpp#L660-L770>)。PointXYZI不保存逐点时间信息。

**风险**：topic名不能证明消息语义；如果上游输出的是多帧拼接地图，把历史点从当前原点连线会穿过当前遮挡物，制造不存在的free证据；若是去畸变当前帧，也需核实原点时间近似误差。地面层/悬空判断对free证据非常敏感。

**验收**：核查外部LIO源码与输出说明，记录单帧点数、覆盖持续时间、是否保留运动目标、降采样和过滤规则。更换LIO时将此作为接口验收，不是简单重映射topic。

### HW-05 [P0/P1，部署约束] 足迹、姿态和yaw控制权必须统一

**证据**：当前主优化固定[OMNI_XY](</home/xyt/map/src/MA_Nav/src/planner/replan/src/fsm_replanner.cpp#L193-L196>)，输出[angular.z=0](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L247-L251>)；[yaw接口](</home/xyt/map/src/MA_Nav/src/planner/controller/include/controller/yaw_plan.hpp>)仍是声明式骨架。搜索和避障主要是点位置+标量安全距离。

**风险**：若底盘不是可独立XY平移的理想全向平台，参考可能不可执行；若外部在小陀螺/自瞄控制yaw，矩形/非对称机身、折叠件扫掠包络随yaw改变，0.35 m中心净空是否足够无法由代码证明。`angular.z=0`在下游可能表示“停止旋转”而非“我不拥有yaw”，应核实协议。

**验收**：给出展开/折叠的长宽高、旋转中心、可保证v/a/yaw能力及控制仲裁；在窄门横移、倒车、旋转、坡面下测试完整包络。目标朝向当前不能作为已实现到达条件。

### HW-06 [P1，部署约束] 电源、网络、温度、时间同步与机械标定会直接变成算法故障

- 供电应覆盖冷启动自热、底盘电机瞬态压降与线束损耗；雷达重启后LIO/地图/轨迹如何清理必须明确。
- PTP/GPS“支持”不等于现场已经同步；记录同步源、offset、时间跳变与恢复机制，区分传感器时间和主机ROS时间。
- 100BASE-TX链路应测丢包、NIC/交换机负载与DDS大点云链路；不要把所有图像/点云可视化挤进比赛控制通道。
- CPU热降频、雷达过温、振动松动、罩面污染、强光/黑色/反光/细线都应纳入故障工况。算法无法用概率记忆永久补偿传感器停止。
- [外参配置注释](</home/xyt/map/src/MA_Nav/config/map.yaml#L121-L125>)已记录0.194与TF中0.15不一致的待核实项；4.4 cm差异接近一个5 cm体素，尤其影响杆下连续free证据与地面高度。必须确定base、足底、IMU、雷达的真正变换，而非选择“看起来图更好”的数。

**验收**：长时满负载、开RViz/关RViz、低温上电、急停制动、网络抖动、反光材质和不同机械姿态下，保留带版本的bag与实测地面真值。

## 11. 目标跟踪、工具层与工程部署

### TRACK-01 [P1，事实；当前未接入] 库级追踪不等于端到端追踪能力

**证据**：[路径规划CMake](</home/xyt/map/src/MA_Nav/src/planner/path_planning/CMakeLists.txt#L5-L9>)收集追踪源码；[TargetTracker](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/target_tracking/target_tracker.cpp#L51-L136>)输出goal/gimbal/hold，但ROS节点无对应测量订阅、状态调用和输出仲裁。

**设计问题**：hold或找不到viewpoint时只返回空goal，必须由调用者决定停车还是旧轨迹；当前位置安全且可见不保证从机器人到该点连通可达；LOS采用通行2D占据，通行地图删除高杆不代表视觉不遮挡。对目标周边ignore圆还会一并忽略邻近墙等真实遮挡，且LOS地图外不遮挡。

**算法限制**：[匀速预测与候选选择](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/target_tracking/viewpoint_selector.cpp#L103-L273>)没有目标绕障/主动搜索和自车到候选的可达时间代价；[KF](</home/xyt/map/src/MA_Nav/src/planner/path_planning/src/target_tracking/target_estimator.cpp#L33-L77>)没有统一非有限数过滤，dt=0可重复更新，目标身份变化主要靠门控/长间隔重置，且下游未利用完整预测不确定性。

**约束/验收**：这些是待接入模块问题，不声称当前定点导航已执行这些危险分支。未来接入前应定义目标ID、时间戳、失联、hold、无解、yaw与fold冲突的执行合同；把视觉遮挡与底盘通行地图分开验证。

### ENG-01 [P1，事实] 安装空间与运行目录未解耦，默认资产路径不一致

**证据**：[launch默认路径](</home/xyt/map/src/MA_Nav/launch/run.launch.py#L11-L20>)来自包share；但[Config](</home/xyt/map/src/MA_Nav/src/ros2/include/ros2/config.hpp#L319-L348>)对内部map_params_path直接LoadFile，不按planner配置目录解析。[默认路径](</home/xyt/map/src/MA_Nav/config/planner.yaml#L10-L17>)混用`config/map.yaml`、`src/nav/MA_Nav/map`和`src/nav/MA_Nav/maps`；顶层安装[只列launch/config](</home/xyt/map/src/MA_Nav/CMakeLists.txt#L99-L104>)。

**后果**：从本仓库根目录运行可找到config不代表从工作区根/systemd/安装包可找到；保存默认`global_map.pgm`但加载默认`global_maps.pgm`，同目录存在两个名字也不能保证版本对应。tunnel_regions路径复数maps与实际map不同。导航启动[重复加载先验](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L122-L129>)两次，增加I/O并制造误导。

**建议/验收**：固定资产路径解析规则、只读资源与可写数据目录，启动前校验全部解析后的绝对路径、地图/语义版本与模式；从任意CWD和干净安装环境启动验证。不得靠部署人员记住特定cd位置。

### ENG-02 [P1，事实] 保存成功反馈不可信，地图与语义非原子快照

**证据**：[服务](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L104-L120>)调用后无条件success=true；[地图保存](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L773-L798>)不检查文件open/write结果/创建目录；[语义保存](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L800-L833>)内部失败return也不会传回服务。地图和YAML分开写，无临时文件+整体版本切换。

**后果**：目录不存在、磁盘满或只读时仍报成功；掉电留下半文件；先保存地图再保存区域期间坐标/参数改变会错配。保存的是occupancy合并结果，不是必然经人工审定的静态永久层。

**建议/验收**：服务结果应表达真实I/O结果，并给地图、语义、分辨率、frame、采集模式和校验和统一版本。只读目录、磁盘满、进程中断与保存后重新加载对照。

### ENG-03 [P1，事实] PGM/YAML为内部坐标约定，不能直接当标准ROS地图互通

**证据**：[读取投影](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L725-L754>)规定PGM列=world Y、行反向world X，保存与之匹配；这不是常见ROS地图image列=x、行=y。loader只读origin前2维，忽略yaw，像素<128当占据；不消费negate/阈值/mode，图外保持自由。离线回放[PGM输出](</home/xyt/map/src/MA_Nav/src/test/src/test_terrain_replay.cpp#L129-L135>)又使用另一种标准朝向。

**后果**：内部往返可一致，但用外部map_server、地图编辑器、离线回放PGM替换时可能旋转/镜像/平移；未知灰色按固定阈值可能变自由。必须区分“某一组文件内部可读”与“兼容ROS地图格式”。

**建议/验收**：用非对称L形障碍和三个已测地标做保存→本loader→标准工具的三方比对；覆盖非零origin yaw、未知像素与不同分辨率重投影，不先凭地图图像看起来相似就通过。

### ENG-04 [P1/P2，事实] 参数可读不等于生效，缺省与非法值缺少系统校验

**证据**：[参数加载器](</home/xyt/map/src/MA_Nav/src/ros2/include/ros2/config.hpp#L17-L22>)错误维度可静默给0；ReplanParam/PathPostProcessingParams部分成员无默认值，而加载函数缺键直接保留；[通用YamlLoader](</home/xyt/map/src/MA_Nav/src/utils/include/utils/yaml_loader.hpp#L48-L104>)不同于planner加载策略，错误类型可回落default且很多返回值未被上层用于阻止运行。

| 配置/字段 | 当前实际状态 | 误导风险 |
|---|---|---|
| min_replan_interval | YAML有，Config未读取，FSM不执行 | 误以为每0.5 s最多重规划一次 |
| minco_traj_validity_duration / continuity_threshold | 读取但主链未应用 | 误以为旧参考有年龄/距离门控 |
| is_minco | 读取但主链不分支 | 误以为false关闭MA优化 |
| terrain.max_step_height | 传入配置，当前活跃地形判定未消费；出现于旧注释分支 | 调参没有预期效果 |
| distance_weight / yaw_weight | 读取，当前时间分配实际用rotation_penalty_weight | 调错权重 |
| traj_cut_length / if_cut | 仅计算标志，主链未执行轨迹截断 | 长轨迹被误认为局部短轨迹 |
| visualization.range | 同时影响MaMap取3D障碍进行terrain分析的boxSearch | 可视化参数可能影响算法输入 |

**建议/验收**：先做配置键→加载→消费者→实际单位的清单；对N≤0、dt≤0、resolution≤0、min>max、负票数/过大票数、非有限限幅/权重、概率顺序等拒绝非法配置。不要擅自删除旧字段或改变比赛默认值；文档先标清活跃/历史/待废弃。

### ENG-05 [P1/P2，事实+条件性风险] 可视化不是无成本读者，也未完全遵循地图快照隔离

**证据**：[pub_callback](</home/xyt/map/src/MA_Nav/src/ros2/src/ros2_node.cpp#L493-L513>)与control同组；[ROG可视化](</home/xyt/map/src/MA_Nav/src/ros2/include/ros2/misc/visualizer.hpp#L1312-L1360>)直接读robot_state和3D占据，不是2D快照，unknown云有订阅时才遍历。保存occupancy的getter也直接拷贝可变成员。

**后果**：现串行入口下首先是遍历/序列化阻塞；未来真正并发时又是3D读写一致性问题。`visualization.enable`没有在此入口统一门控所有工作，禁用不一定让所有开销消失。RViz订阅未知体素可显著改变运行负载。

**建议/验收**：算法用快照、可视化用快照/降频且不干扰控制的责任要明确；不默认加高频统计。测开关RViz、unknown订阅、保存服务与正常比赛模式下的最坏控制间隔。

### ENG-06 [P2，事实] 构建、依赖声明、测试与发布元数据不利于可复现交付

**证据**：[顶层CMake](</home/xyt/map/src/MA_Nav/CMakeLists.txt#L8-L27>)强制Release/LTO且在检查IPO前已加flto；[测试CMake](</home/xyt/map/src/MA_Nav/src/test/CMakeLists.txt#L5-L67>)默认进入OpenCV/FetchContent Google Benchmark网络依赖，无独立测试开关与CTest注册；[package元数据](</home/xyt/map/src/MA_Nav/package.xml>)仍是gcopter描述/作者、BSD声明，与不同来源源码许可需要一致性审查。CMake依赖远多于manifest显式声明。

**后果**：离线比赛机/新架构机器可能因测试依赖或LTO失败；rosdep不能仅凭当前manifest完整还原环境；多个头文件样条实现与大型third-party集合使实际使用版本难追溯。性能图和平均耗时不覆盖闭环安全回归。本文没有构建，不能声称当前构建必定失败。

**建议/验收**：记录OS/ROS/RMW/编译器/solver版本和目标硬件，隔离生产依赖与可选性能工具；经授权做干净离线构建/安装验证。为本文P0项建立可失败的自动验收，不只输出图片或printf。许可和来源应以真实归属审查，不能直接替换版权声明。

## 12. 跨模块耦合故障链：为什么单模块“通过”仍不够

### CHAIN-A：动态障碍封路 → 搜索失败 → 仍沿旧轨迹运动

1. 地图写入障碍；FSM在前端折线上检测碰撞。
2. 重新JPS无路或优化失败，ROS回调return。
3. 旧ma_traj_interface仍有效；controller不看FAILED，也无独立障碍约束。
4. 若传感器正常，仍持续发速度；底盘收包watchdog也可能不会触发。

**需闭合**：候选失败与旧轨迹可执行权限分离，执行前缀碰撞检查与停车包络优先；验证对象是实际停止位置而非“报了no path”。

### CHAIN-B：MID360看不见近地 → 种子填平 → 未知自由 → 靠近台边

1. 水平正装导致近场地面无回波，或坡面/遮挡让下方地面消失。
2. GroundLayer用base平面seed；reach跨SEED放宽高差，或地形无证据不下结论。
3. 2D未知初始0/动态层清0；drop_edge默认关闭。
4. 搜索认为可通行；MPC只看XY轨迹，无接地/负障碍约束。

**需闭合**：盲区假设不能升级成观测事实；人工确认起步区、地面覆盖/禁行先验或互补感知，与低速安全动作共同验收。

### CHAIN-C：初始路径为折叠减速 → 优化压缩时间 → 重规划跳事件 → 机械未收拢入洞

1. 后处理前端压低入口前速度。
2. 样条优化修改时间和航点，没有fold位置限速硬约束。
3. 新fold事件用旧游标跳过，内部状态补发处没有publish。
4. 即使发出Bool，也没有反馈确认完成；游标按位置可跳跃。
5. 地形按0.225/0.24 m放行，实际展开包络不受地图状态约束。

**需闭合**：最终轨迹准备时间、命令送达、物理完成、足迹入/出洞四个条件共同成立；不是增加fold_margin就足够。

### CHAIN-D：计划参考替代反馈 → 反向刹车可行性失真 → 定时解除急停

1. 热启动取旧参考v/a，不校验真实速度/年龄/距离。
2. 刹车按6/10减速度，MPC按3硬限，控制10ms执行50ms增量。
3. 发0速度后按理论T解除急停，不检查是否停稳。
4. 刹停点再推移、最近点投影跳到样条，实际连接段无验收。

**需闭合**：同一状态估计、同一时间基准与实测停车包络；参考连续性不能替代动力学可执行性。

### CHAIN-E：为了修实时性开启多线程 → 暴露共享状态竞态

1. 当前spin_some使长任务阻塞控制。
2. 直接改spin以启用多线程。
3. 2D快照虽稳定，pose/轨迹/事件/游标/3D ROG仍跨组读写。
4. 出现时序相关的错轨迹、漏折叠、越界或未定义行为。

**需闭合**：调度调整与同步审查必须作为同一变更边界；不能单改执行器就以“8线程”作为验收结果。

### CHAIN-F：先验加载失败/重定位 → 图仍可用 → 全链坐标自洽但物理错误

1. 默认相对路径失效，加载失败仅日志；或者外部LIO重置world原点。
2. 静态2D地图、地面层、隧道语义与旧轨迹没有统一坐标版本重置。
3. 发布frame字符串仍叫world，所有节点看起来“frame相同”。
4. 机器人在错误位置绕障/折叠，MPC数值正常并不能发现地图物理错位。

**需闭合**：定位生命周期与地图/语义/轨迹的版本一致性，重定位后先失能再恢复；至少实测三个非共线地标和机械区域。

## 13. 实机放行条件与最小验收证据

以下是问题关闭所需证据，不是本次已执行的测试，也不授权直接上高速或改参数。

| 验收项 | 场景/注入 | 必需结果证据 | 关联 |
|---|---|---|---|
| 输入健康 | odom/cloud分别断流、旧stamp持续接收、错frame、NaN、重定位 | 有明确状态与有限停车时延；恢复需重新满足条件 | ROS-01～03 |
| 失效停止 | 无路、优化失败、QP失败、进程卡死/退出 | 实际轮/足端停止距离，而非只有零指令/日志 | FSM-01、CTRL-04 |
| 轨迹验收 | 薄墙、S弯、窄缝、越界、Stage2失败 | 最终执行轨迹含刹车/连接的净空与动力学上界 | OPT-01、FSM-02 |
| 正确时间 | 高负载、开RViz、保存、连续目标切换 | 各回调线程、控制dt、数据年龄和命令加速度最坏值 | ROS-04、CTRL-01 |
| 感知几何 | 各高度黑块/细杆、盲扇区、pitch/roll、不同安装形态 | 首次可靠检测距离、漏检持续时长、外参误差范围 | HW-01～06 |
| 地面安全 | 台边起步、坡顶下坡、坑洞、跨缝、姿态门控 | 实测地形真值与可通行判定一致；未知不静默放行 | TER-01～05 |
| 动态障碍记忆 | 遮挡2s、离开视野、近场残影、先验内外新障碍 | 无“静止未见即自由”漏洞，且恢复通行有证据 | MAP-01～05 |
| 折叠联锁 | 延迟、卡死、接收端重启、近入口重规划、终点在洞内 | 物理完成前不进入，整机出洞后才展开 | ACT-01～03 |
| 地图交付 | 非对称标志地图、换CWD、只读目录、损坏/缺失文件 | 正确方向/原点/分辨率、真实I/O反馈、版本对应 | ENG-01～03 |
| 可复现回归 | 同一bag不同倍率、不同可视化负载与目标频率 | 明确哪些行为依墙钟变化，保存配置/版本/数据标注 | ROS-06、ENG-06 |

### 13.1 必须补齐的现场事实

1. 机器人真实类型、底盘控制接口与`angular.z=0`的含义；谁拥有XY/yaw/急停最终仲裁权。
2. 展开/折叠完整包络、动态最高点、旋转中心、足底/base/IMU/lidar外参和姿态范围。
3. 雷达实际安装高度/朝向、遮挡CAD、是否随云台/折叠件移动。
4. 外部LIO的cloud/odom时间、速度frame、去畸变、是否累积地图、定位质量与重置通知。
5. 全链端到端延迟、实际扫描帧率、CPU最坏负载、底盘watchdog和各方向实测制动能力。
6. 先验地图的采集条件、world原点对齐、版本、场地变化和禁行区域；洞口净空真实测量及误差。
7. 雷达供电/温度/网络/PTP配置及故障恢复行为；外部安全控制器是否真正独立。

没有这些数据，不应声称3 m/s、0.24 m高度或0.35 m净空已被实机验证；本文也不为它们给出无条件推荐值。

## 14. 修复边界与问题关闭标准

建议按风险依赖关系讨论，不在本次直接更改代码：

- **先安全使能与失效动作**：感知新鲜度、结果失效、执行前缀碰撞、底盘停止合同、折叠完成联锁。
- **再统一接口物理含义**：时钟/位姿、frame/外参、状态反馈、动力学约束、足迹与可观测域。
- **再处理候选规划质量**：连续轨迹验收、投影/热启动条件、搜索与插值边界、地形置信度。
- **最后局部优化工程成本**：复制、可视化、构建依赖、文档与历史参数，不为理论优雅重写比赛主链。

每个问题关闭至少包含：复现输入/场景、允许改变的行为边界、最小改动、静态检查、经授权的构建与回放/实机证据、残余风险。问题无法仅以“参数改了”“仿真图更好”“没有报错”关闭。

## 15. 本次检查结果与局限

- 已读取主入口、launch、配置加载、ROS主回调、地图包装/2D ESDF、ROG更新与射线/遗忘、GroundLayer、TerrainAnalyzer、JPS/A*、后处理、MA优化/代价、重规划、轨迹接口/MPC、目标跟踪核心和构建/包元数据。
- 用Python静态解析两份主配置YAML、launch AST和package XML，均通过；计算MID360平地近界、名义地图格数，不触发编译或代码生成。
- 使用Livox官方规格和ROS2 Humble上游执行器源码交叉核对硬件/框架事实；ROS发行版和部署二进制与当前源码的一致性未验证。
- 未全面审计所有第三方目录、旧优化器实现、传感器固件/驱动/LIO/底盘/决策工程；未证明当前install中的二进制包含当前工作区改动；未读取rosbag图像/点云内容并作环境真值判断。
- 初始工作区有用户未提交改动；本文按现场工作区审计，不能自动推广到HEAD、其他分支或比赛部署版本。
- **未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。** 未声称任何运行测试通过。
- 文档交付与静态证据核对可单独通过；**系统部署安全结论为 NEEDS_FIX / 需实机证据，不是PASS。**
