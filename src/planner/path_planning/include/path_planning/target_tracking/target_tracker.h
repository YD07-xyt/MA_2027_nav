#pragma once
/**
 * 目标跟踪（追击）对外接口。
 *
 * 用法（与 ROS 解耦，调用方负责线程同步）：
 *   TargetTracker tracker; tracker.set_param(params);
 *   // 目标回调：
 *   tracker.on_target_measurement(target_xy_world, stamp);
 *   // 规划定时器：
 *   auto out = tracker.update(*grid_map, robot_state, now);
 *   if (out.hold)                       → 停车保持
 *   else if (out.goal && out.goal_updated) → 把 out.goal 作为 goal_pose 交给 FsmReplan::plan
 *   out.gimbal                          → 云台：release=true 释放给自瞄，否则按 out.gimbal.yaw 控制
 *
 * 暂时处理：第一阶段只做选点，不改优化器。可见性仅在观测点处保证，从当前位置到观测点的途中不保证可见。
 *
 * 文档：src/planner/path_planning/src/target_tracking/README.md（流程、接入、调参、已知问题）
 * 参数说明与调参：tracking_types.h 中 TargetTrackingParams 的注释
 */
#include "map/core/grid_map.hpp"
#include "path_planning/target_tracking/target_estimator.h"
#include "path_planning/target_tracking/tracking_types.h"
#include "path_planning/target_tracking/viewpoint_selector.h"
#include <optional>

namespace path_planning::target_tracking {

class TargetTracker {
public:
    auto set_param(const TargetTrackingParams& params) -> void;
    auto params() const -> const TargetTrackingParams& { return params_; }
    auto reset() -> void;

    /// 输入目标位置观测（世界系）。stamp 与 update 的 now 必须是同一时钟 (s)
    auto on_target_measurement(const Eigen::Vector2d& target_pos, double stamp) -> bool;

    /// 每个规划周期调用一次，计算跟踪目标点与云台指令
    auto update(const grid_map::GridMap& map, const RobotState2D& robot, double now) -> TrackingOutput;

    auto state() const -> TrackingState { return state_; }
    auto estimator() const -> const TargetEstimator& { return estimator_; }

private:
    auto compute_gimbal(
        const RobotState2D& robot,
        const Eigen::Vector2d& target_pos,
        const Eigen::Vector2d& target_vel,
        bool visible,
        double obs_age
    ) -> GimbalCommand;

    TargetTrackingParams params_;
    TargetEstimator estimator_;
    ViewpointSelector selector_;
    TrackingState state_ = TrackingState::IDLE;
    std::optional<TrackingGoal> last_goal_;
    double last_gimbal_yaw_ = 0.0;
    bool has_gimbal_yaw_ = false;
};

} // namespace path_planning::target_tracking
