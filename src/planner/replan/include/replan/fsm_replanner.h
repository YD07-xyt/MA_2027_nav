#pragma once
#include "map/grid_map.hpp"
#include "traj_optimize/ma_spline_opt/optimizer_config.h"

#include "path_planning/post_processing.h"
#include "utils/expected.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <memory>
namespace replan {
class FsmReplan {
public:
    //重规划触发参数
    struct ReplanParam {
        Eigen::Vector3d goal_deviation;
        //double replan_interval; // 路径最大年龄（s），超过则强制重规划
        double replan_lateral_dev; // 横向偏差阈值（m），机器人偏离参考路径超过则重规划
        double minco_traj_validity_duration; // minco轨迹有效期 (s)，超时则视为无效
        double goal_reached_radius; // 起点≈终点判定半径（m）：机器人距目标小于该值时直接退出不规划，
        // 避免退化路径（A* 单点）与无意义的周期重规划
        //double hard_clearance; // 优化结果硬净空（m）：轨迹采样点净空低于该值直接拒收。
        // 应 ≤ 通道允许的最小净空（≈通道半宽）；太小失去安全网，太大窄缝永远过不去
        double minco_traj_continuity_threshold; ///< 轨迹连续性距离阈值
            ///< (m)，机器人距轨迹投影点小于此值时使用投影点边界
        double projection_search_resolution; ///< 投影点搜索时间分辨率 (s)，越小越精确但计算量越大
        // 反向重规划刹车段：常规减速度；空间不够时最多提到 brake_acc_max，仍不够则急停（发零速）
        double brake_acc = 6.0; ///< 常规刹车减速度 (m/s²)
        double brake_acc_max = 10.0; ///< 刹车减速度上限 (m/s²)，应 ≤ 底盘真实能力且 ≤ MPC u_max
    } replan_param;
    struct PlannerConfig {
        ReplanParam replan_params;
        ma_spline_opt::MaSplineOptimizerConfig ma_spline_opt_params;
        path_planning::PathPostProcessing::PathPostProcessingParams path_planning_params;
    } planner_config_;
    //init 1
    FsmReplan();
    ~FsmReplan();
    FsmReplan(FsmReplan&&) noexcept;
    FsmReplan& operator=(FsmReplan&&) noexcept;
    FsmReplan(const FsmReplan&) = delete;
    FsmReplan& operator=(const FsmReplan&) = delete;
    auto set_param(const PlannerConfig& planner_config) -> void;

    //init 2
    explicit FsmReplan(const PlannerConfig& planner_config);

public:
    enum PathState {
        SUCCESSED,
        FAILED,
        RUNNING,
        IDLE,
    } ;
private:
    struct ResultPath {
        path_planning::PathPostProcessing::Trajectory planning_traj;
        ma_spline_opt::MAsplineOutput ma_spline_traj;
        PathState path_state;
        // true 表示这次返回的是新规划的轨迹
        bool is_new_trajectory = false;
    };

public:
    enum PathError {
        PLANNING_FAILED,
        MINCO_OPT_FIALED,
        MAX_RETRIES,
    };
    using path = tl::expected<ResultPath, PathError>;
    //ma opt
    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path;

public:
    auto one_plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        std::shared_ptr<grid_map::GridMap> grid_map
    ) -> path;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace replan
