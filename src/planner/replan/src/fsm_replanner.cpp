#include "replan/fsm_replanner.h"
#include "path_planning/path_planning.hpp"
#include "traj_optimize/grid_map_esdf.hpp"
#include "traj_optimize/ma_spline_opt/traj_optimizer.h"
#include "utils/expected.hpp"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
namespace replan {
class FsmReplan::Impl {
public:
    Impl() {
        path_planning.set_use_jps(true);
    }

    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        grid_map::GridMap::ConstPtr grid_map
    ) -> path;
    auto set_param(const PlannerConfig& planner_config) -> void;
    path_planning::PathPlanning path_planning;
    ma_spline_opt::MaSplineTrajectoryOptimizer<minco_opt::GridMapESDF> ma_opt_;

private:
    // 内部重规划状态机状态（失败进入 RETRY_PENDING，下一次调用继续尝试）
    enum class State {
        IDLE, // 初始态，尚未生成有效轨迹，收到请求必触发首次规划
        PLANNING, // 规划执行中（单次 plan() 同步执行前端搜索与后端优化时的瞬态）
        MONITORING, // 监控态，持有已提交轨迹，仅检测目标变化/碰撞/横向偏离，无异常则复用旧轨迹
        RETRY_PENDING, // 待重试态，上次规划或优化失败（保留旧轨迹），下一次 plan() 强制重新尝试
    };
    // 驱动内部状态机迁移的事件
    enum class Event {
        START, // 满足重规划条件，开始一次完整规划（IDLE/MONITORING/RETRY_PENDING -> PLANNING）
        PLAN_SUCCEEDED, // 前端搜索与样条优化均成功并提交新轨迹（PLANNING -> MONITORING）
        PLAN_FAILED, // 前端搜索或样条优化失败，保留旧轨迹（PLANNING -> RETRY_PENDING）
        GOAL_REACHED, // 当前位姿已进入 goal_reached_radius，直接结束本次规划（-> MONITORING）
        KEEP_MONITORING, // 未触发任何重规划条件，继续沿用当前已提交轨迹（-> MONITORING）
    };

    State state_ = State::IDLE;
    PlannerConfig planner_config_;
    utils::RobotState old_goal_pose_;
    ResultPath result_;
    std::chrono::steady_clock::time_point last_trajectory_time_; ///< 上一条轨迹的发布时间戳（刹车段按时间推进用）

    auto transition(Event event) -> void;

    // 将过于靠近障碍物的点沿 ESDF 梯度外推到安全距离，确保 A*/碰撞检测可通过
    auto get_safe_pos(const Eigen::Vector2d& pos, const grid_map::GridMap& grid_map, const double& safe_threshold)
        -> Eigen::Vector2d;
    auto check_collision(
        const path_planning::PathPostProcessing::Trajectory& traj,
        const grid_map::GridMap& grid_map,
        const double& safe_threshold
    ) const -> bool;
    auto lateral_deviation(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) const -> double;
    // 反向重规划：沿速度方向生成匀减速刹车段（ESDF 射线检测可用距离）
    auto plan_brake_segment(
        const Eigen::Vector2d& p0,
        const Eigen::Vector2d& v0,
        const grid_map::GridMap& grid_map,
        const double& safe_threshold
    ) const -> ma_spline_opt::BrakeSegment;
};

FsmReplan::FsmReplan(): impl_(std::make_unique<Impl>()) {}
FsmReplan::~FsmReplan() = default;
FsmReplan::FsmReplan(FsmReplan&&) noexcept = default;
FsmReplan& FsmReplan::operator=(FsmReplan&&) noexcept = default;

auto FsmReplan::Impl::transition(Event event) -> void {
    // START: 可规划态 -> PLANNING；成功回 MONITORING，失败回 RETRY_PENDING。
    switch (event) {
        case Event::START:
            if (state_ == State::IDLE || state_ == State::MONITORING || state_ == State::RETRY_PENDING) {
                state_ = State::PLANNING;
            }
            break;
        case Event::PLAN_SUCCEEDED:
            if (state_ == State::PLANNING) state_ = State::MONITORING;
            break;
        case Event::PLAN_FAILED:
            if (state_ == State::PLANNING) state_ = State::RETRY_PENDING;
            break;
        case Event::GOAL_REACHED:
        case Event::KEEP_MONITORING:
            state_ = State::MONITORING;
            break;
    }
}

auto FsmReplan::Impl::set_param(const PlannerConfig& planner_config) -> void {
    planner_config_ = planner_config;
    ma_opt_.set_config(planner_config_.ma_spline_opt_params);
    path_planning.set_param(planner_config_.path_planning_params);
}

FsmReplan::FsmReplan(const PlannerConfig& planner_config): FsmReplan() {
    impl_->set_param(planner_config);
}

auto FsmReplan::set_param(const PlannerConfig& planner_config) -> void {
    impl_->set_param(planner_config);
}

auto FsmReplan::plan(
    const utils::RobotState& goal_pose,
    const utils::RobotState& current_pose,
    grid_map::GridMap::ConstPtr grid_map
) -> path {
    return impl_->plan(goal_pose, current_pose, std::move(grid_map));
}

// 折线碰撞检测辅助函数。
static bool check_path_collision(
    const std::vector<Eigen::Vector2d>& path,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold,
    const double step
);
// 点到线段 [a, b] 的最短距离。
static double point_to_segment_dist(const Eigen::Vector2d& pos, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    const Eigen::Vector2d ab = b - a;
    const double len2 = ab.squaredNorm();
    double t = 0.0;
    if (len2 > 1e-12) {
        t = std::max(0.0, std::min(1.0, (pos - a).dot(ab) / len2));
    }
    const Eigen::Vector2d closest = a + t * ab;
    return (pos - closest).norm();
}
auto FsmReplan::Impl::plan(
    const utils::RobotState& goal_pose,
    const utils::RobotState& current_pose,
    grid_map::GridMap::ConstPtr grid_map
) -> path {
    if (!grid_map) {
        logger::warn(logger::fsm_replan, "grid_map snapshot is null");
        return tl::make_unexpected(PathError::PLANNING_FAILED);
    }

    result_.is_new_trajectory = false;

    const Eigen::Vector2d current_xy(current_pose.p.x(), current_pose.p.y());
    const Eigen::Vector2d goal_xy(goal_pose.p.x(), goal_pose.p.y());
    if (!current_xy.allFinite() || !goal_xy.allFinite()) {
        logger::warn(logger::fsm_replan, "Reject non-finite start or goal pose");
        return tl::make_unexpected(PathError::PLANNING_FAILED);
    }

    const bool oob_occupied = (grid_map->outOfBoundsPolicy() == grid_map::OutOfBoundsPolicy::OCCUPIED);
    if (oob_occupied && !grid_map->isInsideMap(goal_xy)) {
        logger::warn(
            logger::fsm_replan,
            "Reject out-of-bounds goal ({:.2f}, {:.2f}) in navigation mode",
            goal_xy.x(),
            goal_xy.y()
        );
        return tl::make_unexpected(PathError::PLANNING_FAILED);
    }

    // 导航模式允许轻微越界的起点沿虚拟墙向内恢复（<= start_oob_recover_dist），
    // 越界过深时直接拒收，避免在定位异常期间覆盖状态与在途轨迹。
    if (oob_occupied && !grid_map->isInsideMap(current_xy)) {
        const double oob_dist = -std::min(0.0, grid_map->getDistance(current_xy));
        const double max_recover = std::max(0.0, planner_config_.replan_params.start_oob_recover_dist);
        if (oob_dist > max_recover + 1e-9) {
            logger::warn(
                logger::fsm_replan,
                "Reject start ({:.2f}, {:.2f}): out of navigation map by {:.3f}m > start_oob_recover_dist ({:.3f}m)",
                current_xy.x(),
                current_xy.y(),
                oob_dist,
                max_recover
            );
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }
    }

    // 起点≈终点（机器人已到达目标附近）：直接退出，不规划。
    if ((current_xy - goal_xy).norm() < planner_config_.replan_params.goal_reached_radius) {
        result_.path_state = PathState::SUCCESSED;
        transition(Event::GOAL_REACHED);
        return result_;
    }

    // if(grid_map->is_tunnel(current_pose.p.head<2>())){
    //     logger::fsm_replan->error("now robot in tunnel");
    // }

    const auto safe_threshold = planner_config_.path_planning_params.safe_threshold;
    const double hard_clearance = std::max(0.05, safe_threshold * 0.2);

    Eigen::Vector2d diff = (old_goal_pose_.p - goal_pose.p).head<2>();
    Eigen::Vector2d threshold = planner_config_.replan_params.goal_deviation.head<2>();
    const bool goal_changed = diff.cwiseAbs().x() > threshold.x() || diff.cwiseAbs().y() > threshold.y();

    bool must_replan = state_ == State::IDLE || state_ == State::RETRY_PENDING || goal_changed;
    if (!must_replan) {
        const bool path_collision = result_.planning_traj.optimized_path.empty()
            || check_collision(result_.planning_traj, *grid_map, hard_clearance);
        must_replan = path_collision;
    }
    if (!must_replan) {
        // 机器人偏离参考路径超过 replan_lateral_dev；有刹车段时一并检查 p0 -> 首航点直线，避免拷贝路径。
        const Eigen::Vector2d current_xy(current_pose.p.x(), current_pose.p.y());
        const auto& opt_path = result_.planning_traj.optimized_path;
        double lateral_dev = lateral_deviation(current_xy, opt_path);
        if (result_.ma_spline_traj.brake.duration > 0.0 && !opt_path.empty()) {
            const double brake_dev =
                point_to_segment_dist(current_xy, result_.ma_spline_traj.brake.p0, opt_path.front());
            lateral_dev = std::min(lateral_dev, brake_dev);
        }
        must_replan = lateral_dev > planner_config_.replan_params.replan_lateral_dev;
    }
    if (!must_replan) {
        transition(Event::KEEP_MONITORING);
        return result_;
    }

    transition(Event::START);
    {
        utils::TimeConsuming timer("planner_fsm", true); // true 表示允许打印

        Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
        Eigen::Vector2d goal(goal_pose.p.x(), goal_pose.p.y());

        // 若起点/终点过于靠近障碍物或导航模式轻微越界，沿 ESDF/虚拟墙梯度外推到安全点
        const Eigen::Vector2d robot_pos = start;
        start = get_safe_pos(start, *grid_map, safe_threshold);
        goal = get_safe_pos(goal, *grid_map, safe_threshold);
        if (oob_occupied && !grid_map->isInsideMap(robot_pos)
            && (!grid_map->isInsideMap(start) || grid_map->isCollision(start, safe_threshold)))
        {
            logger::warn(
                logger::fsm_replan,
                "Failed to recover out-of-bounds start ({:.2f}, {:.2f}) into safe navigation map interior",
                robot_pos.x(),
                robot_pos.y()
            );
            transition(Event::PLAN_FAILED);
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }
        if (oob_occupied && !grid_map->isInsideMap(goal)) {
            logger::warn(
                logger::fsm_replan,
                "Adjusted goal ({:.2f}, {:.2f}) landed outside navigation map bounds",
                goal.x(),
                goal.y()
            );
            transition(Event::PLAN_FAILED);
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }
        const bool start_moved = (start - robot_pos).norm() > 1e-6;

        // 起点速度/加速度：取机器人在上一条参考（刹车段 + 样条）上最近投影时刻的计划 v(t)、a(t)
        // （同 navkit prepareHotStart）。不使用里程计速度；没有可用的上一条参考时按静止处理。
        Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
        Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
        bool hot_start = false;
        const auto& last_out = result_.ma_spline_traj;
        if (last_out.success && last_out.trajectory.isInitialized()) {
            const double dur = ma_spline_opt::total_duration(last_out);
            const double res = std::max(planner_config_.replan_params.projection_search_resolution, 1e-3);
            Eigen::Vector2d p, v, a;
            double best_t = 0.0;
            double best_d = std::numeric_limits<double>::max();
            // 有刹车段时：反向样条原路经过刹车段，同一位置在两段上都有投影，按位置找会取到刹车段的前向速度
            // （机器人没完全跟到刹停点时还会再生成一次前向刹车段，循环卡死）。
            // 刹车段与控制器一致按时间推进：发布后还在刹车时长内就取该时刻的刹车状态；否则只在样条上找。
            const double brake_T = last_out.brake.duration;
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - last_trajectory_time_).count();
            if (brake_T > 0.0 && elapsed < brake_T) {
                best_t = std::max(0.0, elapsed);
            } else {
                for (double t = brake_T; t <= dur + 1e-9; t += res) {
                    ma_spline_opt::sample_xy(last_out, t, p, v, a);
                    const double d = (p - robot_pos).squaredNorm();
                    if (d < best_d) {
                        best_d = d;
                        best_t = t;
                    }
                }
            }
            // 投影到旧参考终点（已走完）时按静止处理
            if (best_t < dur) {
                ma_spline_opt::sample_xy(last_out, best_t, p, start_vel, start_acc);
                hot_start = true;
            }
        }

        const Eigen::Vector3d current_vel(start_vel.x(), start_vel.y(), current_pose.wz);
        path_planning.set_velocity(current_vel, Eigen::Vector3d::Zero());

        auto trajectory = path_planning.path_planning(*grid_map, start, goal, current_pose.yaw, goal_pose.yaw, 5000);

        if (!trajectory.has_value()) {
            logger::warn(logger::fsm_replan, "planning failed");
            transition(Event::PLAN_FAILED);
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }

        // 新路径首段方向，用于反向判定与速度投影
        auto path_first_dir = [](const std::vector<Eigen::Vector2d>& opt_path) -> Eigen::Vector2d {
            if (opt_path.size() >= 2) {
                const Eigen::Vector2d seg = opt_path[1] - opt_path[0];
                if (seg.norm() > 1e-6) return seg.normalized();
            }
            return Eigen::Vector2d::Zero();
        };
        Eigen::Vector2d first_dir = path_first_dir(trajectory->optimized_path);

        // ---------- 反向：速度与新路径首段夹角 > 90° ----------
        // 先沿当前速度方向匀减速刹停（刹车段），再从刹停点以零速规划反向轨迹。
        ma_spline_opt::BrakeSegment brake;
        constexpr double kMinBrakeSpeed = 0.2; // 低于该速度不单独规划刹车段，直接按静止起步
        const double speed = start_vel.norm();
        if (first_dir.norm() > 0.5 && speed > 1e-6 && start_vel.dot(first_dir) < 0.0) {
            if (speed > kMinBrakeSpeed) {
                brake = plan_brake_segment(robot_pos, start_vel, *grid_map, safe_threshold);

                const Eigen::Vector2d stop =
                    brake.p0 + brake.v0 * brake.duration + 0.5 * brake.acc * brake.duration * brake.duration;
                start = get_safe_pos(stop, *grid_map, safe_threshold);
                logger::info(
                    logger::fsm_replan,
                    "reverse replan: v={:.2f} brake a={:.2f} T={:.2f}s dist={:.2f}m emergency={}",
                    speed,
                    brake.acc.norm(),
                    brake.duration,
                    (stop - robot_pos).norm(),
                    brake.emergency
                );

                path_planning.set_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
                trajectory = path_planning.path_planning(*grid_map, start, goal, current_pose.yaw, goal_pose.yaw, 5000);
                if (!trajectory.has_value()) {
                    logger::warn(logger::fsm_replan, "planning from brake stop point failed");
                    transition(Event::PLAN_FAILED);
                    return tl::make_unexpected(PathError::PLANNING_FAILED);
                }
                first_dir = path_first_dir(trajectory->optimized_path);
            }
            hot_start = false;
            start_vel.setZero();
            start_acc.setZero();
        } else if (first_dir.norm() > 0.5 && speed > 0.1 && start_vel.normalized().dot(first_dir) < 0.5) {
            // 夹角 60°~90°：只保留沿新路径首段方向的速度分量，横向分量和加速度清零
            start_vel = start_vel.dot(first_dir) * first_dir;
            start_acc.setZero();
        }

        // 打印 raw_path / optimized_path 点数,确认直线/近距离时的短路情况
        // logger::info(logger::fsm_replan,
        //     "path points: raw={} optimized={} total_time={:.2f}s",
        //     result_.planning_traj.raw_path.size(),
        //     result_.planning_traj.optimized_path.size(),
        //     result_.planning_traj.total_time
        // );
        minco_opt::GridMapESDF grid_map_esdf(grid_map);
        auto ma_intput = ma_spline_opt::from_path_planning_trajectory(trajectory.value());

        // 起点被挪开：速度只保留沿新路径首段方向的分量（反向置 0），加速度置 0
        if (start_moved) {
            const double along = first_dir.norm() > 0.5 ? std::max(0.0, start_vel.dot(first_dir)) : 0.0;
            start_vel = along * first_dir;
            start_acc = Eigen::Vector2d::Zero();
        }

        ma_intput.start_vel = start_vel;
        ma_intput.start_acc = start_acc;
        logger::debug(
            logger::fsm_replan,
            "start state: hot={} moved={} v=({:.2f},{:.2f}) a=({:.2f},{:.2f})",
            hot_start,
            start_moved,
            start_vel.x(),
            start_vel.y(),
            start_acc.x(),
            start_acc.y()
        );

        ma_intput.model = ma_spline_opt::OptModel::OMNI_XY;

        auto ma_output = ma_opt_.optimize(ma_intput, grid_map_esdf);
        if (ma_output.success == false) {
            logger::info(logger::fsm_replan, "ma opt failed");
            transition(Event::PLAN_FAILED);
            return tl::make_unexpected(PathError::MINCO_OPT_FIALED);
        }
        if (oob_occupied && ma_output.trajectory.isInitialized()) {
            const auto& traj = ma_output.trajectory.getTrajectory();
            const double t_start = traj.getStartTime();
            const double t_end = traj.getEndTime();
            bool traj_oob = false;
            for (double t = t_start; t <= t_end; t += 0.05) {
                const Eigen::Vector2d pt = traj.evaluate(t, 0).head<2>();
                if (!grid_map->isInsideMap(pt)) {
                    traj_oob = true;
                    break;
                }
            }
            if (!traj_oob) {
                const Eigen::Vector2d end_pt = traj.evaluate(t_end, 0).head<2>();
                if (!grid_map->isInsideMap(end_pt)) {
                    traj_oob = true;
                }
            }
            if (traj_oob) {
                logger::warn(logger::fsm_replan, "ma opt trajectory exited navigation map bounds, rejecting");
                transition(Event::PLAN_FAILED);
                return tl::make_unexpected(PathError::MINCO_OPT_FIALED);
            }
        }
        ma_output.brake = brake; // 刹车段拼在样条前面（无刹车时 duration=0）
        // 优化成功后再组装并整体提交，失败时保持旧轨迹不变。
        ResultPath candidate;
        candidate.planning_traj = std::move(trajectory.value());
        candidate.ma_spline_traj = std::move(ma_output);
        candidate.is_new_trajectory = true;
        candidate.path_state = PathState::SUCCESSED;
        last_trajectory_time_ = std::chrono::steady_clock::now();
        result_ = std::move(candidate);
        old_goal_pose_ = goal_pose;
        transition(Event::PLAN_SUCCEEDED);
        return result_;
    }
}

auto FsmReplan::Impl::lateral_deviation(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) const
    -> double {
    if (path.size() < 2) {
        return std::numeric_limits<double>::max();
    }
    // 点到折线各段的最短距离（逐段投影并夹紧到线段内）
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 1; i < path.size(); ++i) {
        min_dist = std::min(min_dist, point_to_segment_dist(pos, path[i - 1], path[i]));
    }
    return min_dist;
}

static bool check_path_collision(
    const std::vector<Eigen::Vector2d>& path,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold,
    const double step
) {
    if (path.empty()) {
        return false;
    }
    auto is_unsafe = [&](const Eigen::Vector2d& pos) -> bool { return grid_map.isCollision(pos, safe_threshold); };

    for (size_t i = 0; i < path.size(); ++i) {
        if (is_unsafe(path[i])) {
            return true;
        }
        if (i + 1 >= path.size()) {
            continue;
        }
        // 只检查航点会漏掉长直线段中间新增的动态障碍物，
        // 因此沿每一段按 dense_sample_resolution 再采样检查。
        const Eigen::Vector2d a = path[i];
        const Eigen::Vector2d b = path[i + 1];
        const double len = (b - a).norm();
        if (len < 1e-9) {
            continue;
        }
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int j = 1; j < n; ++j) {
            const double r = static_cast<double>(j) / n;
            if (is_unsafe(a + r * (b - a))) {
                return true;
            }
        }
    }
    return false;
}

auto FsmReplan::Impl::check_collision(
    const path_planning::PathPostProcessing::Trajectory& traj,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold
) const -> bool {
    // 同时检查 planning原始网格路径与优化后航点路径：
    // 原始路径可发现两个优化航点之间被新障碍挡住的情况；
    // 再对每段折线做密集采样，避免长直线中间漏检。
    const double step = std::max(planner_config_.path_planning_params.dense_sample_resolution, 0.02);
    return check_path_collision(traj.optimized_path, grid_map, safe_threshold, step)
        || check_path_collision(traj.raw_path, grid_map, safe_threshold, step);
}
auto FsmReplan::Impl::get_safe_pos(
    const Eigen::Vector2d& pos,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold
) -> Eigen::Vector2d {
    if (!grid_map.isCollision(pos, safe_threshold)) {
        return pos;
    }
    Eigen::Vector2d safe = pos;
    const double max_push = 1.5; // 最大外推距离，避免把起点推得太远
    double pushed = 0.0;
    for (int i = 0; i < 200; ++i) {
        double d = 0.0;
        Eigen::Vector2d g;
        if (!grid_map.getDistanceAndGradient(safe, d, g)) {
            break;
        }
        double gn = g.norm();
        if (gn < 1e-6) {
            break; // 梯度退化，无法继续外推
        }
        Eigen::Vector2d dir = g / gn; // 梯度指向远离障碍或指向地图内部方向
        double need = (safe_threshold - d) + 0.05;
        double step = std::min(need, max_push - pushed);
        if (step <= 0.0) {
            break;
        }
        safe += dir * step;
        pushed += step;
        if (!grid_map.isCollision(safe, safe_threshold) || pushed >= max_push) {
            return safe;
        }
    }
    return safe;
};
// 沿速度方向匀减速刹停（参考 navkit backup_traj_optimizer_s4）：
// 1. 沿 v 方向在 ESDF 上射线检测，得到距离 < safe_threshold 前的可用距离 L_geo；
// 2. 常规刹车距离 L = v²/(2·brake_acc) 放得下就用 brake_acc；
// 3. 放不下就把减速度提到 v²/(2·L_geo)，上限 brake_acc_max；
// 4. 上限仍放不下：emergency=true，控制器在刹车段内直接发零速，参考段仍按 brake_acc_max 生成。
auto FsmReplan::Impl::plan_brake_segment(
    const Eigen::Vector2d& p0,
    const Eigen::Vector2d& v0,
    const grid_map::GridMap& grid_map,
    const double& safe_threshold
) const -> ma_spline_opt::BrakeSegment {
    ma_spline_opt::BrakeSegment brake;
    const double speed = v0.norm();
    if (speed < 1e-6) return brake;

    const auto& rp = planner_config_.replan_params;
    const double a_nom = std::max(rp.brake_acc, 1e-3);
    const double a_cap = std::max(rp.brake_acc_max, a_nom);
    const Eigen::Vector2d dir = v0 / speed;

    // 射线最远只需检查以最小减速度刹停的距离
    const double max_check = speed * speed / (2.0 * a_nom);
    const double step = std::max(grid_map.getResolution() * 0.5, 0.02);
    // 起点本身已贴近障碍时（同 DDR-opt）把阈值放到起点净空的 0.85 倍（夹紧非负），避免一出发就判为撞墙
    double thr = safe_threshold;
    if (grid_map.isInsideMap(p0)) {
        thr = std::min(thr, std::max(0.0, 0.85 * grid_map.getDistance(p0)));
    }
    double l_geo = max_check;
    for (double s = step; s <= max_check + 1e-9; s += step) {
        const Eigen::Vector2d q = p0 + dir * s;
        if (grid_map.isCollision(q, thr)) {
            l_geo = std::max(0.0, s - step);
            break;
        }
    }

    double a_brake = a_nom;
    if (l_geo < max_check - 1e-9) {
        a_brake = l_geo > 1e-3 ? speed * speed / (2.0 * l_geo) : std::numeric_limits<double>::infinity();
        if (a_brake > a_cap) {
            a_brake = a_cap;
            brake.emergency = true;
        }
    }

    brake.p0 = p0;
    brake.v0 = v0;
    brake.acc = -a_brake * dir;
    brake.duration = speed / a_brake;
    return brake;
}
}
