#include "replan/fsm_replanner.h"
#include "path_planning/path_planning.hpp"
#include "traj_optimize/grid_map_esdf.hpp"
#include "traj_optimize/ma_spline_opt/traj_optimizer.h"
#include "utils/expected.hpp"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>
namespace replan {
class FsmReplan::Impl {
public:
    Impl() {
        path_planning.set_use_jps(planner_config_.path_planning_params.use_jps);
    }

    auto plan(
        const utils::RobotState& goal_pose,
        const utils::RobotState& current_pose,
        grid_map::GridMap::ConstPtr grid_map
    ) -> path;
    auto plan(
        const std::vector<utils::RobotState>& goals,
        const utils::RobotState& current_pose,
        grid_map::GridMap::ConstPtr grid_map
    ) -> path;
    auto set_param(const PlannerConfig& planner_config) -> void;
    auto set_clock(utils::Clock clock) -> void;
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
    std::vector<utils::RobotState> old_goals_; // 已提交任务的原始输入点列（用于变更检测）
    std::vector<utils::RobotState> working_goals_; // 已过滤冗余相邻中间点的工作点列（末尾必为最终点）
    size_t working_active_idx_ = 0; // 当前待通过工作点索引
    std::vector<size_t> window_via_goal_indices_; // 当前样条每个 via_anchor_times 对应的 working_goals_ 索引
    bool window_includes_final_ = true; // 当前样条末端是否已覆盖最终点
    ResultPath result_;
    utils::Clock clock_ = utils::Clock::steady();
    std::optional<double> last_trajectory_time_sec_; ///< 上一条轨迹的提交时刻（秒，跟随 clock_，刹车段按时间推进用）
    double last_monitor_t_ = 0.0; ///< 当前已提交执行轨迹上的监控进度游标（秒）

    auto transition(Event event) -> void;
    auto elapsed_since_last_traj() const -> double;

    // 将过于靠近障碍物的点沿 ESDF 梯度外推到安全距离，确保 A*/碰撞检测可通过
    auto get_safe_pos(const Eigen::Vector2d& pos, const grid_map::GridMap& grid_map, const double& safe_threshold)
        -> Eigen::Vector2d;
    auto check_trajectory_collision(
        const ma_spline_opt::MAsplineOutput& out,
        double t_from,
        const grid_map::GridMap& grid_map,
        double hard_clearance
    ) const -> bool;
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

auto FsmReplan::Impl::set_clock(utils::Clock clock) -> void {
    clock_ = clock;
    last_trajectory_time_sec_.reset();
}

auto FsmReplan::Impl::elapsed_since_last_traj() const -> double {
    if (!last_trajectory_time_sec_.has_value()) {
        return std::numeric_limits<double>::infinity();
    }
    const double dt = clock_.nowSec() - last_trajectory_time_sec_.value();
    // 兼容仿真/回放时钟回拨：倒退时视为刹车段已结束，避免卡死在旧刹车段
    return (dt >= 0.0) ? dt : std::numeric_limits<double>::infinity();
}

FsmReplan::FsmReplan(const PlannerConfig& planner_config): FsmReplan() {
    impl_->set_param(planner_config);
}

auto FsmReplan::set_param(const PlannerConfig& planner_config) -> void {
    impl_->set_param(planner_config);
}

auto FsmReplan::set_clock(utils::Clock clock) -> void {
    impl_->set_clock(clock);
}

auto FsmReplan::plan(
    const utils::RobotState& goal_pose,
    const utils::RobotState& current_pose,
    grid_map::GridMap::ConstPtr grid_map
) -> path {
    return impl_->plan(goal_pose, current_pose, std::move(grid_map));
}

auto FsmReplan::plan(
    const std::vector<utils::RobotState>& goals,
    const utils::RobotState& current_pose,
    grid_map::GridMap::ConstPtr grid_map
) -> path {
    return impl_->plan(goals, current_pose, std::move(grid_map));
}

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

// 仅采样执行轨迹（刹车段 + 五次样条）在局部时间 t 的 2D 位置（跳过速度/加速度求导）。
static auto sample_traj_pos(const ma_spline_opt::MAsplineOutput& out, double t) -> Eigen::Vector2d {
    const auto& b = out.brake;
    t = std::max(0.0, t);
    if (t < b.duration) {
        return b.p0 + b.v0 * t + 0.5 * b.acc * t * t;
    }
    const auto& traj = out.trajectory.getTrajectory();
    const double ts = out.trajectory.getStartTime() + std::clamp(t - b.duration, 0.0, out.trajectory.getDuration());
    return traj.evaluate(ts, 0).head<2>();
}

struct TrajProjection {
    double t = 0.0;
    double dist = 0.0;
};

// 求机器人在执行轨迹（刹车段 + 样条）上的当前局部时间游标与横向距离：
// - 有刹车段且仍在刹车时长内：游标与控制器一致按经过时间推进，距离取到直线刹车段的最短距离；
// - 样条段内：先在 hint_t 附近局部搜索防自交/折返误跳，若局部误差超过回退阈值再全样条段兜底。
static auto project_on_trajectory(
    const ma_spline_opt::MAsplineOutput& out,
    const Eigen::Vector2d& pos,
    double elapsed,
    double hint_t,
    double res,
    double local_fallback_dist
) -> TrajProjection {
    const double dur = ma_spline_opt::total_duration(out);
    const double brake_T = out.brake.duration;
    if (brake_T > 0.0 && elapsed < brake_T) {
        const Eigen::Vector2d brake_end =
            out.brake.p0 + out.brake.v0 * brake_T + 0.5 * out.brake.acc * brake_T * brake_T;
        return {std::max(0.0, elapsed), point_to_segment_dist(pos, out.brake.p0, brake_end)};
    }

    const double step = std::max(res, 1e-3);
    const double anchor = std::clamp(hint_t, brake_T, dur);
    const double t_lo = std::max(brake_T, anchor - 0.2);
    const double t_hi = std::min(dur, anchor + 1.5);

    double best_t = anchor;
    double best_d2 = (sample_traj_pos(out, anchor) - pos).squaredNorm();
    auto scan_range = [&](double lo, double hi) {
        double t = lo;
        while (true) {
            const double d2 = (sample_traj_pos(out, t) - pos).squaredNorm();
            if (d2 < best_d2) {
                best_d2 = d2;
                best_t = t;
            }
            if (t >= hi - 1e-9) break;
            t = std::min(t + step, hi);
        }
    };

    scan_range(t_lo, t_hi);
    if (best_d2 > local_fallback_dist * local_fallback_dist && (t_lo > brake_T + 1e-9 || t_hi < dur - 1e-9)) {
        scan_range(brake_T, dur);
    }
    return {best_t, std::sqrt(best_d2)};
}

// 比较原始目标点列是否发生变化（任一点 xy 超 goal_deviation 或长度变化）
static auto goals_sequence_changed(
    const std::vector<utils::RobotState>& old_goals,
    const std::vector<utils::RobotState>& new_goals,
    const Eigen::Vector2d& threshold
) -> bool {
    if (old_goals.size() != new_goals.size()) {
        return true;
    }
    for (size_t i = 0; i < old_goals.size(); ++i) {
        const double dx = std::abs(old_goals[i].p.x() - new_goals[i].p.x());
        const double dy = std::abs(old_goals[i].p.y() - new_goals[i].p.y());
        if (dx > threshold.x() || dy > threshold.y()) {
            return true;
        }
    }
    return false;
}

// 过滤间距过小的冗余相邻中间途径点，始终严格保留最终目标点。
static auto filter_redundant_via_points(const std::vector<utils::RobotState>& goals, double min_spacing)
    -> std::vector<utils::RobotState> {
    if (goals.size() <= 1 || min_spacing <= 0.0) {
        return goals;
    }
    std::vector<utils::RobotState> filtered;
    filtered.reserve(goals.size());
    for (size_t i = 0; i + 1 < goals.size(); ++i) {
        const Eigen::Vector2d pt(goals[i].p.x(), goals[i].p.y());
        if (filtered.empty()) {
            filtered.push_back(goals[i]);
            continue;
        }
        const Eigen::Vector2d prev(filtered.back().p.x(), filtered.back().p.y());
        if ((pt - prev).norm() >= min_spacing) {
            filtered.push_back(goals[i]);
        }
    }
    const Eigen::Vector2d final_xy(goals.back().p.x(), goals.back().p.y());
    while (!filtered.empty()) {
        const Eigen::Vector2d prev(filtered.back().p.x(), filtered.back().p.y());
        if ((final_xy - prev).norm() < min_spacing) {
            filtered.pop_back();
        } else {
            break;
        }
    }
    filtered.push_back(goals.back());
    return filtered;
}

// 估算执行轨迹自局部时刻 t_from 至末端的剩余弧长（m），用于滑动视野滚动延伸判定。
static auto remaining_arc_length(const ma_spline_opt::MAsplineOutput& out, double t_from) -> double {
    const double dur = ma_spline_opt::total_duration(out);
    const double t_start = std::clamp(t_from, 0.0, dur);
    if (t_start >= dur - 1e-6) {
        return 0.0;
    }
    constexpr double kDt = 0.05;
    Eigen::Vector2d prev = sample_traj_pos(out, t_start);
    double length = 0.0;
    double t = t_start;
    while (t < dur - 1e-9) {
        t = std::min(t + kDt, dur);
        const Eigen::Vector2d cur = sample_traj_pos(out, t);
        length += (cur - prev).norm();
        prev = cur;
    }
    return length;
}

auto FsmReplan::Impl::plan(
    const utils::RobotState& goal_pose,
    const utils::RobotState& current_pose,
    grid_map::GridMap::ConstPtr grid_map
) -> path {
    return plan(std::vector<utils::RobotState> {goal_pose}, current_pose, std::move(grid_map));
}

auto FsmReplan::Impl::plan(
    const std::vector<utils::RobotState>& goals,
    const utils::RobotState& current_pose,
    grid_map::GridMap::ConstPtr grid_map
) -> path {
    if (!grid_map) {
        logger::warn(logger::fsm_replan, "grid_map snapshot is null");
        return tl::make_unexpected(PathError::PLANNING_FAILED);
    }

    const auto& rp = planner_config_.replan_params;
    const int max_waypoints = std::max(1, rp.max_waypoints);
    if (goals.empty() || static_cast<int>(goals.size()) > max_waypoints) {
        logger::warn(
            logger::fsm_replan,
            "Reject goals sequence with invalid size {} (allowed 1..{})",
            goals.size(),
            max_waypoints
        );
        return tl::make_unexpected(PathError::PLANNING_FAILED);
    }

    result_.is_new_trajectory = false;

    const Eigen::Vector2d current_xy(current_pose.p.x(), current_pose.p.y());
    if (!current_xy.allFinite()) {
        logger::warn(logger::fsm_replan, "Reject non-finite start pose");
        return tl::make_unexpected(PathError::PLANNING_FAILED);
    }

    const bool oob_occupied = (grid_map->outOfBoundsPolicy() == grid_map::OutOfBoundsPolicy::OCCUPIED);
    for (size_t i = 0; i < goals.size(); ++i) {
        const Eigen::Vector2d g_xy(goals[i].p.x(), goals[i].p.y());
        if (!g_xy.allFinite()) {
            logger::warn(logger::fsm_replan, "Reject non-finite goal pose at index {}", i);
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }
        if (oob_occupied && !grid_map->isInsideMap(g_xy)) {
            logger::warn(
                logger::fsm_replan,
                "Reject out-of-bounds goal[{}] ({:.2f}, {:.2f}) in navigation mode",
                i,
                g_xy.x(),
                g_xy.y()
            );
            return tl::make_unexpected(PathError::PLANNING_FAILED);
        }
    }

    // 导航模式允许轻微越界的起点沿虚拟墙向内恢复（<= start_oob_recover_dist），
    // 越界过深时直接拒收，避免在定位异常期间覆盖状态与在途轨迹。
    if (oob_occupied && !grid_map->isInsideMap(current_xy)) {
        const double oob_dist = -std::min(0.0, grid_map->getDistance(current_xy));
        const double max_recover = std::max(0.0, rp.start_oob_recover_dist);
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

    const double safe_threshold = planner_config_.path_planning_params.safe_threshold;
    const double hard_clearance = std::min(std::max(1e-3, rp.hard_clearance), safe_threshold);
    const double r_pass = std::max(0.0, rp.via_pass_radius);
    const double r_opt = std::clamp(rp.via_opt_radius, 0.0, r_pass);
    const double via_hyst = std::max(0.0, rp.via_time_pass_hysteresis);
    const double elapsed_traj = elapsed_since_last_traj();

    const Eigen::Vector2d goal_threshold(rp.goal_deviation.x(), rp.goal_deviation.y());
    const bool goals_changed = goals_sequence_changed(old_goals_, goals, goal_threshold);

    const auto& cur_traj = result_.ma_spline_traj;
    const bool has_cur_traj = cur_traj.success && cur_traj.trajectory.isInitialized();

    std::vector<utils::RobotState> candidate_goals;
    size_t candidate_active_idx = 0;
    std::optional<TrajProjection> proj_opt;

    if (!goals_changed && !working_goals_.empty()) {
        if (has_cur_traj) {
            const TrajProjection proj = project_on_trajectory(
                cur_traj,
                current_xy,
                elapsed_traj,
                last_monitor_t_,
                rp.projection_search_resolution,
                rp.replan_lateral_dev
            );
            proj_opt = proj;
            last_monitor_t_ = proj.t;
        }

        // 严格按序推进队首中间途径点：空间半径或轨迹时间锚点越过（未偏离轨迹）满足任一即推进，
        // 避免绕障无法进入途径点半径时回头死循环；推进本身不触发重规划。
        while (working_active_idx_ + 1 < working_goals_.size()) {
            const Eigen::Vector2d head_xy(
                working_goals_[working_active_idx_].p.x(),
                working_goals_[working_active_idx_].p.y()
            );
            bool passed = ((current_xy - head_xy).norm() <= r_pass);
            if (!passed && proj_opt.has_value() && proj_opt->dist <= rp.replan_lateral_dev
                && window_via_goal_indices_.size() == cur_traj.via_anchor_times.size())
            {
                for (size_t k = 0; k < window_via_goal_indices_.size(); ++k) {
                    if (window_via_goal_indices_[k] == working_active_idx_) {
                        if (proj_opt->t > cur_traj.via_anchor_times[k] + via_hyst) {
                            passed = true;
                        }
                        break;
                    }
                }
            }
            if (!passed) {
                break;
            }
            ++working_active_idx_;
        }

        // 仅当所有中间点均已通过，且当前位于最终点到达半径内时直接完成。
        if (working_active_idx_ + 1 == working_goals_.size()) {
            const Eigen::Vector2d final_xy(working_goals_.back().p.x(), working_goals_.back().p.y());
            if ((current_xy - final_xy).norm() < rp.goal_reached_radius) {
                result_.path_state = PathState::SUCCESSED;
                result_.active_goal_index = working_active_idx_;
                result_.remaining_goals = 0;
                transition(Event::GOAL_REACHED);
                return result_;
            }
        }

        candidate_goals = working_goals_;
        candidate_active_idx = working_active_idx_;
    } else {
        // 新任务：在局部候选中过滤相邻冗余中间点，并跳过出发即已在通过半径内的前导中间点。
        candidate_goals = filter_redundant_via_points(goals, std::max(0.0, rp.via_min_spacing));
        candidate_active_idx = 0;
        while (candidate_active_idx + 1 < candidate_goals.size()) {
            const Eigen::Vector2d head_xy(
                candidate_goals[candidate_active_idx].p.x(),
                candidate_goals[candidate_active_idx].p.y()
            );
            if ((current_xy - head_xy).norm() <= r_pass) {
                ++candidate_active_idx;
            } else {
                break;
            }
        }

        // 起点≈最终点：直接退出，不规划退化路径（A* 单点）。
        if (candidate_active_idx + 1 == candidate_goals.size()) {
            const Eigen::Vector2d final_xy(candidate_goals.back().p.x(), candidate_goals.back().p.y());
            if ((current_xy - final_xy).norm() < rp.goal_reached_radius) {
                old_goals_ = goals;
                working_goals_ = std::move(candidate_goals);
                working_active_idx_ = candidate_active_idx;
                window_via_goal_indices_.clear();
                window_includes_final_ = true;
                result_.path_state = PathState::SUCCESSED;
                result_.active_goal_index = working_active_idx_;
                result_.remaining_goals = 0;
                transition(Event::GOAL_REACHED);
                return result_;
            }
        }
    }

    bool must_replan = (state_ == State::IDLE) || (state_ == State::RETRY_PENDING) || goals_changed || !has_cur_traj;
    if (!must_replan) {
        const TrajProjection& proj = proj_opt.value();
        must_replan = (proj.dist > rp.replan_lateral_dev)
            || check_trajectory_collision(cur_traj, proj.t, *grid_map, hard_clearance);
        if (!must_replan && !window_includes_final_ && rp.horizon_length > 0.0 && elapsed_traj >= 0.3) {
            const double eff_replan_dist = std::min(std::max(0.0, rp.horizon_replan_dist), 0.7 * rp.horizon_length);
            if (remaining_arc_length(cur_traj, proj.t) <= eff_replan_dist) {
                must_replan = true;
            }
        }
    }
    if (!must_replan) {
        result_.active_goal_index = working_active_idx_;
        result_.remaining_goals = working_goals_.size() - working_active_idx_;
        transition(Event::KEEP_MONITORING);
        return result_;
    }

    transition(Event::START);
    {
        utils::TimeConsuming timer("planner_fsm", true); // true 表示允许打印

        // 滑动规划视野截断：至少包含下一个目标点；视野内保留全部中间途径点直到累计弧长达到 horizon_length
        std::vector<size_t> win_via_indices;
        size_t win_goal_idx = candidate_active_idx;
        const int max_via_in_window = std::max(0, planner_config_.path_planning_params.max_traj_num - 1);
        double accum_dist =
            (Eigen::Vector2d(candidate_goals[candidate_active_idx].p.x(), candidate_goals[candidate_active_idx].p.y())
             - current_xy)
                .norm();
        while (win_goal_idx + 1 < candidate_goals.size()) {
            if (rp.horizon_length > 0.0 && accum_dist >= rp.horizon_length) {
                break;
            }
            if (static_cast<int>(win_via_indices.size()) >= max_via_in_window) {
                break;
            }
            const Eigen::Vector2d p_cur(candidate_goals[win_goal_idx].p.x(), candidate_goals[win_goal_idx].p.y());
            const Eigen::Vector2d p_next(
                candidate_goals[win_goal_idx + 1].p.x(),
                candidate_goals[win_goal_idx + 1].p.y()
            );
            win_via_indices.push_back(win_goal_idx);
            ++win_goal_idx;
            accum_dist += (p_next - p_cur).norm();
        }
        const bool win_includes_final = (win_goal_idx + 1 == candidate_goals.size());
        const utils::RobotState& win_goal_state = candidate_goals[win_goal_idx];

        Eigen::Vector2d start(current_pose.p.x(), current_pose.p.y());
        Eigen::Vector2d goal(win_goal_state.p.x(), win_goal_state.p.y());

        // 若起点/中间点/窗口终点过于靠近障碍物或导航模式轻微越界，沿 ESDF/虚拟墙梯度外推到安全点
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

        std::vector<Eigen::Vector2d> safe_vias;
        safe_vias.reserve(win_via_indices.size());
        for (size_t idx: win_via_indices) {
            const Eigen::Vector2d raw_via(candidate_goals[idx].p.x(), candidate_goals[idx].p.y());
            const Eigen::Vector2d safe_via = get_safe_pos(raw_via, *grid_map, safe_threshold);
            if (oob_occupied && !grid_map->isInsideMap(safe_via)) {
                logger::warn(
                    logger::fsm_replan,
                    "Adjusted via[{}] ({:.2f}, {:.2f}) landed outside navigation map bounds",
                    idx,
                    safe_via.x(),
                    safe_via.y()
                );
                transition(Event::PLAN_FAILED);
                return tl::make_unexpected(PathError::PLANNING_FAILED);
            }
            safe_vias.push_back(safe_via);
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
            const double best_t = proj_opt.has_value() ? proj_opt->t
                                                       : project_on_trajectory(
                                                             last_out,
                                                             robot_pos,
                                                             elapsed_traj,
                                                             last_monitor_t_,
                                                             rp.projection_search_resolution,
                                                             rp.replan_lateral_dev
                                                         )
                                                             .t;
            // 投影到旧参考终点（已走完）时按静止处理
            if (best_t < dur) {
                Eigen::Vector2d p;
                ma_spline_opt::sample_xy(last_out, best_t, p, start_vel, start_acc);
                hot_start = true;
            }
        }

        const int timeout_ms = std::max(1, planner_config_.path_planning_params.search_timeout_ms);
        const Eigen::Vector3d current_vel(start_vel.x(), start_vel.y(), current_pose.wz);
        path_planning.set_velocity(current_vel, Eigen::Vector3d::Zero());

        auto run_frontend = [&](const Eigen::Vector2d& from) {
            if (safe_vias.empty()) {
                return path_planning
                    .path_planning(*grid_map, from, goal, current_pose.yaw, win_goal_state.yaw, timeout_ms);
            }
            return path_planning.path_planning_through_poses(
                *grid_map,
                from,
                safe_vias,
                goal,
                current_pose.yaw,
                win_goal_state.yaw,
                timeout_ms
            );
        };

        auto trajectory = run_frontend(start);
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
                trajectory = run_frontend(start);
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

        minco_opt::GridMapESDF grid_map_esdf(grid_map);
        auto ma_intput = ma_spline_opt::from_path_planning_trajectory(trajectory.value());

        // 注入中间途径点软球约束锚点（单点导航 safe_vias 为空时零开销跳过）
        std::vector<size_t> active_win_via_goal_indices;
        if (!safe_vias.empty() && trajectory->via_knot_indices.size() == safe_vias.size()
            && ma_intput.timed_trajectory.size() == trajectory->timed_trajectory.size())
        {
            const int num_knots = static_cast<int>(ma_intput.timed_trajectory.size());
            const double via_weight = std::max(0.0, planner_config_.ma_spline_opt_params.via_point_weight);
            ma_intput.via_anchors.reserve(safe_vias.size());
            active_win_via_goal_indices.reserve(safe_vias.size());
            for (size_t k = 0; k < safe_vias.size(); ++k) {
                const int knot_idx = trajectory->via_knot_indices[k];
                if (knot_idx <= 0 || knot_idx >= num_knots - 1) {
                    continue;
                }
                ma_spline_opt::ViaPointAnchor anchor;
                anchor.knot_index = knot_idx;
                anchor.target = safe_vias[k];
                anchor.pass_radius = r_opt;
                anchor.weight = via_weight;
                ma_intput.via_anchors.push_back(anchor);
                active_win_via_goal_indices.push_back(win_via_indices[k]);
            }
        }

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
            "start state: hot={} moved={} v=({:.2f},{:.2f}) a=({:.2f},{:.2f}) vias={}",
            hot_start,
            start_moved,
            start_vel.x(),
            start_vel.y(),
            start_acc.x(),
            start_acc.y(),
            ma_intput.via_anchors.size()
        );

        ma_intput.model = ma_spline_opt::OptModel::OMNI_XY;

        auto ma_output = ma_opt_.optimize(ma_intput, grid_map_esdf);
        if (ma_output.success == false) {
            logger::info(logger::fsm_replan, "ma opt failed");
            transition(Event::PLAN_FAILED);
            return tl::make_unexpected(PathError::MINCO_OPT_FIALED);
        }
        ma_output.brake = brake; // 刹车段拼在样条前面（无刹车时 duration=0）
        for (double& t_anchor: ma_output.via_anchor_times) {
            t_anchor += brake.duration;
        }
        // 统一提交门禁：用与监控态一致的 check_trajectory_collision 校验整条执行轨迹的硬净空与地图边界
        // （导航模式下 OutOfBoundsPolicy::OCCUPIED 时 isCollision 同步拦截越界点），避免提交违规轨迹导致下帧震荡重规划。
        if (check_trajectory_collision(ma_output, 0.0, *grid_map, hard_clearance)) {
            logger::warn(
                logger::fsm_replan,
                "ma opt trajectory violated hard_clearance ({:.2f}m) or map bounds, rejecting",
                hard_clearance
            );
            transition(Event::PLAN_FAILED);
            return tl::make_unexpected(PathError::MINCO_OPT_FIALED);
        }
        // 优化成功后再组装并整体提交，失败时保持旧轨迹与旧任务进度不变。
        ResultPath candidate;
        candidate.planning_traj = std::move(trajectory.value());
        candidate.ma_spline_traj = std::move(ma_output);
        candidate.is_new_trajectory = true;
        candidate.path_state = PathState::SUCCESSED;
        candidate.active_goal_index = candidate_active_idx;
        candidate.remaining_goals = candidate_goals.size() - candidate_active_idx;
        last_trajectory_time_sec_ = clock_.nowSec();
        last_monitor_t_ = 0.0;
        result_ = std::move(candidate);
        old_goals_ = goals;
        working_goals_ = std::move(candidate_goals);
        working_active_idx_ = candidate_active_idx;
        window_via_goal_indices_ = std::move(active_win_via_goal_indices);
        window_includes_final_ = win_includes_final;
        transition(Event::PLAN_SUCCEEDED);
        return result_;
    }
}

auto FsmReplan::Impl::check_trajectory_collision(
    const ma_spline_opt::MAsplineOutput& out,
    double t_from,
    const grid_map::GridMap& grid_map,
    double hard_clearance
) const -> bool {
    if (!out.success || !out.trajectory.isInitialized()) {
        return true;
    }

    const double dur = ma_spline_opt::total_duration(out);
    const double brake_T = out.brake.duration;
    const double max_spatial_step = std::max(planner_config_.path_planning_params.dense_sample_resolution, 0.05);
    constexpr double kTimeStep = 0.05;

    // 沿局部时间区间 [t_begin, t_end] 采样执行轨迹；相邻采样间距超过 max_spatial_step 时按线段细分，
    // 防止高速段漏检薄障碍物，同时保证区间起止端点精确受检。
    auto check_interval = [&](double t_begin, double t_end, double thr) -> bool {
        if (t_end < t_begin - 1e-9) {
            return false;
        }
        Eigen::Vector2d prev_p = sample_traj_pos(out, t_begin);
        if (grid_map.isCollision(prev_p, thr)) {
            return true;
        }
        double t = t_begin;
        while (t < t_end - 1e-9) {
            t = std::min(t + kTimeStep, t_end);
            const Eigen::Vector2d cur_p = sample_traj_pos(out, t);
            const double seg_len = (cur_p - prev_p).norm();
            if (seg_len > max_spatial_step) {
                const int n = static_cast<int>(std::ceil(seg_len / max_spatial_step));
                for (int j = 1; j < n; ++j) {
                    const double r = static_cast<double>(j) / n;
                    if (grid_map.isCollision(prev_p + r * (cur_p - prev_p), thr)) {
                        return true;
                    }
                }
            }
            if (grid_map.isCollision(cur_p, thr)) {
                return true;
            }
            prev_p = cur_p;
        }
        return false;
    };

    const double t_clamped = std::clamp(t_from, 0.0, dur);

    // 1. 若仍在非急停刹车段内，先检剩余刹车段 [t_clamped, brake_T]；
    //    若刹车起点本身贴墙，沿用 plan_brake_segment 的 0.85*d0 阈值规则，避免刚生成即自触发重规划。
    //    若为急停刹车段（emergency=true），控制器原地发零速并在刹车结束后跳至样条起点，跳过刹车几何段。
    if (brake_T > 0.0 && t_clamped < brake_T && !out.brake.emergency) {
        double brake_thr = hard_clearance;
        const Eigen::Vector2d p_cur = sample_traj_pos(out, t_clamped);
        if (grid_map.isInsideMap(p_cur)) {
            brake_thr = std::min(brake_thr, std::max(0.0, 0.85 * grid_map.getDistance(p_cur)));
        }
        if (check_interval(t_clamped, brake_T, brake_thr)) {
            return true;
        }
    }

    // 2. 仅检查剩余前向样条段 [max(t_clamped, brake_T), dur]，不检测已走过的身后路段
    const double spline_begin = std::max(t_clamped, brake_T);
    return check_interval(spline_begin, dur, hard_clearance);
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
