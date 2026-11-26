#pragma once
#include "map/core/grid_map.hpp"
#include "traj_optimize/ma_spline_opt/optimizer_config.h"

#include "path_planning/post_processing.h"
#include "utils/clock.hpp"
#include "utils/expected.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <vector>
namespace replan {
class FsmReplan {
public:
    //重规划触发参数
    struct ReplanParam {
        Eigen::Vector3d goal_deviation;
        //double replan_interval; // 路径最大年龄（s），超过则强制重规划
        double replan_lateral_dev; // 横向偏差阈值（m），机器人偏离参考路径超过则重规划
        double goal_reached_radius; // 起点≈终点判定半径（m）：机器人距目标小于该值时直接退出不规划，
        // 避免退化路径（A* 单点）与无意义的周期重规划
        // 后端执行轨迹（刹车段+样条）提交验收与监控态前向碰撞检测的统一硬净空（m），须 > 0（运行期夹紧至 <= safe_threshold）
        double hard_clearance = 0.20;
        double projection_search_resolution; ///< 投影点搜索时间分辨率 (s)，越小越精确但计算量越大
        // 反向重规划刹车段：常规减速度；空间不够时最多提到 brake_acc_max，仍不够则急停（发零速）
        double brake_acc = 6.0; ///< 常规刹车减速度 (m/s²)
        double brake_acc_max = 10.0; ///< 刹车减速度上限 (m/s²)，应 ≤ 底盘真实能力且 ≤ MPC u_max
        double start_oob_recover_dist = 0.3; ///< 导航模式下允许轻微越界起点沿虚拟墙向内恢复的最大距离 (m)
        // 多点途经导航参数（单点导航不生效）
        double via_pass_radius = 0.5; ///< 中间途径点通过判定半径 (m)，须 >= 0
        double via_opt_radius = 0.35; ///< 后端软球优化死区半径 (m)，运行期夹紧至 <= via_pass_radius
        double via_time_pass_hysteresis = 0.15; ///< 轨迹时间锚点越过判据的滞回时间 (s)，须 >= 0
        double via_min_spacing = 0.25; ///< 相邻冗余中间途径点自动合并间距 (m)，须 >= 0
        double horizon_length = 12.0; ///< 滑动规划视野长度 (m)；<= 0 表示不截断全量规划
        double horizon_replan_dist = 6.0; ///< 有后续未纳入点时触发滚动延伸的剩余弧长阈值 (m)
        int max_waypoints = 64; ///< 单次多点任务允许的最大航点数上限，须 >= 1
    };
    struct PlannerConfig {
        ReplanParam replan_params;
        ma_spline_opt::MaSplineOptimizerConfig ma_spline_opt_params;
        path_planning::PathPostProcessing::PathPostProcessingParams path_planning_params;
    };
    //init 1
    FsmReplan();
    ~FsmReplan();
    FsmReplan(FsmReplan&&) noexcept;
    FsmReplan& operator=(FsmReplan&&) noexcept;
    FsmReplan(const FsmReplan&) = delete;
    FsmReplan& operator=(const FsmReplan&) = delete;
    // 构造或 set_param 后方可调用 plan；重复设置会更新规划器配置。
    auto set_param(const PlannerConfig& planner_config) -> void;
    // 注入时钟源（仿真/回放跟随 /clock，离线测试可注入 ManualClock；默认 steady_clock）。
    auto set_clock(utils::Clock clock) -> void;

    //init 2
    explicit FsmReplan(const PlannerConfig& planner_config);

public:
    // 对外导航/规划结果状态（供 ROS2 编排层判断交接与发布 nav_feedback）
    enum PathState {
        SUCCESSED, // 规划优化成功并产出可用轨迹，或机器人已进入 goal_reached_radius 到达目标
        FAILED, // 导航任务失败（当前主要由 ROS2 外层 20s 任务超时置位）
        RUNNING, // 导航执行/重试中（收到目标后或单次规划失败等待下一帧重试时由 ROS2 层置位）
        IDLE, // 初始空闲态，尚未收到目标或尚未生成有效轨迹
    };

private:
    struct ResultPath {
        path_planning::PathPostProcessing::Trajectory planning_traj;
        ma_spline_opt::MAsplineOutput ma_spline_traj;
        PathState path_state = PathState::IDLE;
        // true 表示这次返回的是新规划的轨迹
        bool is_new_trajectory = false;
        // 多点任务进度：当前待通过工作点索引，以及含最终点在内的剩余目标数（到达最终点时为 0）
        size_t active_goal_index = 0;
        size_t remaining_goals = 0;
    };

public:
    // 单次 plan() 失败的错误原因（失败不覆盖已提交轨迹，内部转入 RETRY_PENDING）
    enum PathError {
        PLANNING_FAILED, // 地图快照为空，或前端路径搜索失败（含反向刹车后从刹停点二次搜索失败）
        MINCO_OPT_FIALED, // 后端样条轨迹优化失败（保留历史枚举拼写）
    };
    using path = tl::expected<ResultPath, PathError>;
    // 接收不可变地图快照；grid_map 为空时返回 PLANNING_FAILED，不改变内部状态与已提交轨迹。
    // 失败只返回单次错误，已提交的轨迹保持不变。
    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        grid_map::GridMap::ConstPtr grid_map
    ) -> path;

    // 多点途经规划：按序高速通过中间途径点（不减速），仅在最终点 goals.back() 刹停。
    // 中间途径点通过仅推进内部进度，不触发重规划；goals 为空、超过 max_waypoints、含非有限坐标
    // 或在导航模式越界时返回 PLANNING_FAILED，且不改变已提交轨迹与旧任务进度。
    auto plan(
        const std::vector<utils::RobotState>& goals,
        const utils::RobotState& current_pose,
        grid_map::GridMap::ConstPtr grid_map
    ) -> path;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace replan
