#include "path_planning/path_planning.hpp"

namespace path_planning {
class PathPlanningImpl {
public:
    auto set_param(const PathPostProcessing::PathPostProcessingParams& params) -> void {
        use_jps_ = params.use_jps;
        astar_.set_safe_threshold(params.safe_threshold);
        jps_.set_safe_threshold(params.safe_threshold);
        jps_.set_max_snap_radius(params.jps_start_snap_radius);
        post_processing_.set_params(params);
    }
    auto set_velocity(const Eigen::Vector3d& start_vel, const Eigen::Vector3d& end_vel) -> void {
        post_processing_.set_start_vel(start_vel.head<2>().norm());
        post_processing_.set_end_vel(end_vel.head<2>().norm());

        post_processing_.set_start_velocity_vector(start_vel);
        post_processing_.set_end_velocity_vector(end_vel);
    }

    // 选择底层图搜索算法:默认 A*,开启后改用 JPS
    auto set_use_jps(bool use_jps) -> void {
        use_jps_ = use_jps;
    }
    auto path_planning(
        const grid_map::GridMap& grid_map,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        double start_yaw,
        double goal_yaw,
        int timeout_ms
    ) -> std::optional<PathPostProcessing::Trajectory> {
        utils::TimeConsuming timer("Planning", false); // true 表示允许打印

        PathPostProcessing::Trajectory traj;

        // 1. Grid search: JPS or original A*
        traj.raw_path = use_jps_ ? jps_.jps_search(grid_map, start, goal, timeout_ms)
                                 : astar_.original_astar_search(grid_map, start, goal, timeout_ms);

        if (traj.raw_path.empty()) {
            return std::nullopt;
        }
        // 2. Path optimization (remove redundant waypoints)
        traj.optimized_path = post_processing_.optimize_path(grid_map, traj.raw_path);

        // 3. State sampling (generate 5D states)
        traj.path_states = post_processing_.sample_path_states(traj.optimized_path);

        // 4. Time allocation (trapezoidal velocity profile)
        post_processing_.assign_trajectory_timing(traj);

        // 5. Fill additional information
        post_processing_.fill_additional_trajectory_info(traj, start, goal, start_yaw, goal_yaw);
        if (traj.timed_trajectory.empty() || traj.total_time <= 1e-6) {
            logger::warn(logger::planning, "生成的轨迹没有有效时间，已丢弃");
            return std::nullopt;
        }
        return traj;
    };

private:
    AStar astar_;
    JPS jps_;
    bool use_jps_ = false;
    PathPostProcessing post_processing_;
};

PathPlanning::PathPlanning() : impl_(std::make_unique<PathPlanningImpl>()) {}

PathPlanning::~PathPlanning() = default;   // ← 在此处实例化 unique_ptr 的析构

PathPlanning::PathPlanning(PathPlanning&&) noexcept = default;
PathPlanning& PathPlanning::operator=(PathPlanning&&) noexcept = default;

auto PathPlanning::set_param(const PathPostProcessing::PathPostProcessingParams& params) -> void {
    impl_->set_param(params);
};
auto PathPlanning::set_velocity(const Eigen::Vector3d& start_vel, const Eigen::Vector3d& end_vel) -> void {
    impl_->set_velocity(start_vel, end_vel);
};
auto PathPlanning::set_use_jps(bool use_jps) -> void {
    impl_->set_use_jps(use_jps);
};

auto PathPlanning::path_planning(
    const grid_map::GridMap& grid_map,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    double start_yaw,
    double goal_yaw,
    int timeout_ms
) -> std::optional<PathPostProcessing::Trajectory> {
    return impl_->path_planning(grid_map, start, goal, start_yaw, goal_yaw, timeout_ms);
};
};
