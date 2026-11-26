#pragma once

#include "path_planning/post_processing.h"
#include "traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp"
#include "utils/logger.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <string>
#include <vector>

namespace ma_spline_opt {

struct StageOptimizerConfig {
    // 平滑
    double rho_energy = 5.0;

    // 时间
    double weight_time = 20.0;
    double weight_mean_time = 40.0;
    double mean_lower = 0.9;
    double mean_upper = 1.1;
    double min_time = 0.1;
    double weight_min_time = 100.0;

    // ESDF
    double weight_obstacle = 8000.0;

    // 动力学
    double weight_vel = 40.0;
    double weight_acc = 40.0;

    // L-BFGS
    int max_iterations = 200;
    double g_epsilon = 1e-4;
    int lbfgs_mem_size = 64;
    double lbfgs_delta = 5e-3;
};

struct MaSplineOptimizerConfig {
    // 两阶段公共约束
    double safe_distance = 0.3;
    double v_max = 3.0;
    double a_max = 5.0;

    // yaw 动力学
    double weight_yaw_vel = 10.0;
    double weight_yaw_acc = 5.0;
    double yaw_rate_max = 1.0;
    double yaw_acc_max = 2.0;

    // 积分采样
    int integral_num_steps = 8;

    // 中间途径点软球拉回权重（须 << stage2.weight_obstacle，保证安全避障优先）
    double via_point_weight = 600.0;

    // ========== 两次优化参数分开 ==========
    StageOptimizerConfig stage1;
    StageOptimizerConfig stage2;
};

struct TimedReferencePoint {
    double t = 0.0;
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double yaw = 0.0;
};
// 中间途径点软球约束锚点（容忍半径内零惩罚，允许平滑高速通过不减速）
struct ViaPointAnchor {
    int knot_index = -1; // 对应 timed_trajectory / 样条内部节点索引 [1, N-1]
    Eigen::Vector2d target = Eigen::Vector2d::Zero();
    double pass_radius = 0.35; // 死区容忍半径 r_via (m)，>= 0
    double weight = 600.0; // 拉回权重（须 << stage2.weight_obstacle）
};
enum OptModel { OMNI_XY, OMNI_XY_YAW_JOINT };
struct MaSplineInput {
    std::vector<TimedReferencePoint> timed_trajectory;
    std::vector<ViaPointAnchor> via_anchors;
    OptModel model;
    Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_acc = Eigen::Vector2d::Zero();

    double start_yaw_rate = 0.0;
    double start_yaw_acc = 0.0;
    double end_yaw_rate = 0.0;
    double end_yaw_acc = 0.0;
};
// 反向重规划时拼在样条前面的匀减速刹车段（局部时间 [0, duration)）
struct BrakeSegment {
    double duration = 0.0; // 0 表示没有刹车段
    Eigen::Vector2d p0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d v0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d acc = Eigen::Vector2d::Zero(); // 恒定减速度，方向与 v0 相反
    bool emergency = false; // 刹车距离不够（需要的减速度 > brake_acc_max）：控制器在刹车段时长内直接发零速
};
struct MAsplineOutput {
    std::vector<double> time_segments;
    double start_time = 0.0;

    //SplineTrajectory::QuinticSplineND<2> xy_spline;
    SplineTrajectory::QuinticSplineND<3> trajectory;//xy_yaw_spline;

    BrakeSegment brake; // 刹车段在前，样条从刹停点开始
    // 各中间途径点在执行轨迹局部时间轴上的时刻 (s)；优化器输出相对样条起点，拼入刹车段后由调用方偏移 brake.duration
    std::vector<double> via_anchor_times;

    double cost = 0.0;
    bool success = false;
};

// 刹车段 + 样条的总时长
inline double total_duration(const MAsplineOutput& out) {
    const double spline_dur = out.trajectory.isInitialized() ? out.trajectory.getDuration() : 0.0;
    return out.brake.duration + spline_dur;
}

// 按局部时间 t 采样 xy 的位置/速度/加速度：刹车段用匀减速解析式，之后采样样条
inline void sample_xy(
    const MAsplineOutput& out,
    double t,
    Eigen::Vector2d& p,
    Eigen::Vector2d& v,
    Eigen::Vector2d& a
) {
    const auto& b = out.brake;
    t = std::max(0.0, t);
    if (t < b.duration) {
        p = b.p0 + b.v0 * t + 0.5 * b.acc * t * t;
        v = b.v0 + b.acc * t;
        a = b.acc;
        return;
    }
    const auto& traj = out.trajectory.getTrajectory();
    const double ts = out.trajectory.getStartTime() + std::clamp(t - b.duration, 0.0, out.trajectory.getDuration());
    p = traj.evaluate(ts, 0).head<2>();
    v = traj.evaluate(ts, 1).head<2>();
    a = traj.evaluate(ts, 2).head<2>();
}

static double unwrap_yaw(double previous, double current) {
    return previous + std::remainder(current - previous, 2.0 * M_PI);
}

static std::vector<double> build_unwrapped_yaw(const std::vector<TimedReferencePoint>& timed) {
    std::vector<double> yaw(timed.size());

    if (timed.empty()) return yaw;

    yaw[0] = timed[0].yaw;

    for (size_t i = 1; i < timed.size(); ++i) {
        yaw[i] = unwrap_yaw(yaw[i - 1], timed[i].yaw);
    }

    return yaw;
}

inline auto from_path_planning_trajectory(const path_planning::PathPostProcessing::Trajectory& traj) -> MaSplineInput {
    MaSplineInput input;

    std::vector<TimedReferencePoint> pts;
    pts.reserve(traj.timed_trajectory.size() + 2);

    // 起点
    if (traj.timed_trajectory.empty() || traj.timed_trajectory.front().time > 1e-6) {
        pts.push_back({0.0, traj.start_state_XYTheta.head<2>(), traj.start_state_XYTheta.z()});
    }

    // 中间点
    for (const auto& p: traj.timed_trajectory) {
        pts.push_back({p.time, p.state.head<2>(), p.state.z()});
    }

    // 终点
    if (traj.timed_trajectory.empty() || traj.timed_trajectory.back().time < traj.total_time - 1e-6) {
        pts.push_back({traj.total_time, traj.final_state_XYTheta.head<2>(), traj.final_state_XYTheta.z()});
    }

    // 排序
    std::sort(pts.begin(), pts.end(), [](const TimedReferencePoint& a, const TimedReferencePoint& b) {
        return a.t < b.t;
    });

    // 去重
    input.timed_trajectory.clear();
    for (const auto& p: pts) {
        if (input.timed_trajectory.empty() || p.t - input.timed_trajectory.back().t > 1e-6) {
            input.timed_trajectory.push_back(p);
        }
    }

    input.start_vel = traj.start_state.head<2>();
    input.end_vel = traj.final_state.head<2>();
    input.start_acc = Eigen::Vector2d::Zero();
    input.end_acc = Eigen::Vector2d::Zero();

    input.start_yaw_rate = traj.start_state.z();
    input.end_yaw_rate = traj.final_state.z();
    input.start_yaw_acc = 0.0;
    input.end_yaw_acc = 0.0;

    return input;
}

} // namespace ma_spline_opt