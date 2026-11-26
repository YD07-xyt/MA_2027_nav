#pragma once
#include "map/core/grid_map.hpp"
#include <Eigen/Core>
#include <vector>

namespace path_planning {

class PathPostProcessing {
public:
    // 高密度采样点，用于等效距离与速度分配
    struct DenseSample {
        double linear_dist = 0.0;
        double equiv_dist = 0.0;
        double time = 0.0;
        Eigen::Vector2d pos = Eigen::Vector2d::Zero();
        double yaw = 0.0;
    };

private:
    // 5D state (x,y,theta,dtheta,ds)
    struct PathState {
        Eigen::Vector2d position;
        double theta = 0;
        double delta_theta = 0;
        double delta_s = 0;
    };

    // Timed trajectory point
    struct TimedTrajectoryPoint {
        Eigen::Vector3d state;
        double time = 0;
    };

public:
    // Trajectory data structure
    struct Trajectory {
        std::vector<Eigen::Vector2d> raw_path;
        std::vector<Eigen::Vector2d> optimized_path;
        std::vector<PathState> path_states;
        std::vector<TimedTrajectoryPoint> timed_trajectory;

        Eigen::Vector3d start_state = Eigen::Vector3d::Zero();
        Eigen::Vector3d final_state = Eigen::Vector3d::Zero();
        Eigen::Vector3d start_state_XYTheta = Eigen::Vector3d::Zero();
        Eigen::Vector3d final_state_XYTheta = Eigen::Vector3d::Zero();

        Eigen::VectorXd time_segments;
        // 仅多点规划非空：每个中间途径点对应的 timed_trajectory 索引（无效/退化项为 -1）
        std::vector<int> via_knot_indices;

        double total_time = 0;
        double total_length = 0;
        double weighted_length = 0;
    };

public:
    struct PathPostProcessingParams {
        bool use_jps = true; // true 使用 JPS，false 使用 A*
        int search_timeout_ms = 5000; // 单次前端栅格搜索超时 (ms)，须 > 0
        double jps_start_snap_radius = 1.5; // JPS 起点不可通行时环形寻找最近自由格的最大半径 (m)，须 >= 0
        double safe_threshold;
        double max_vel;
        double max_acc;
        double time_resolution;
        int min_traj_num;
        int max_traj_num = 12;
        double start_vel {0.0};
        double end_vel {0.0};
        double dense_sample_resolution = 0.1;
        double rotation_penalty_weight = 1.0;
    };

private:
    PathPostProcessingParams params_;
    Eigen::Vector3d start_velocity_vector_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d final_velocity_vector_ = Eigen::Vector3d::Zero();

public:
    auto set_start_velocity_vector(const Eigen::Vector3d& vel) -> void {
        start_velocity_vector_ = vel;
    }

    auto set_end_velocity_vector(const Eigen::Vector3d& vel) -> void {
        final_velocity_vector_ = vel;
    }

    auto set_start_vel(const double& start_vel) -> void {
        params_.start_vel = start_vel;
    }

    auto set_end_vel(const double& end_vel) -> void {
        params_.end_vel = end_vel;
    }

    auto set_params(const PathPostProcessingParams& params) -> void {
        params_ = params;
    }

public:
    // 下列需要查询地图的方法按调用同步只读借用外部 GridMap 快照（不保存引用）。
    auto check_collision(const grid_map::GridMap& map, const Eigen::Vector2d& pos) const -> bool;
    auto optimize_path(const grid_map::GridMap& map, const std::vector<Eigen::Vector2d>& path)
        -> std::vector<Eigen::Vector2d>;
    auto check_line_collision(const grid_map::GridMap& map, const Eigen::Vector2d& start, const Eigen::Vector2d& end)
        const -> bool;
    auto sample_path_states(const std::vector<Eigen::Vector2d>& path) -> std::vector<PathState>;
    auto normalize_angle(double ref_angle, double& angle) const -> void;
    auto assign_trajectory_timing(Trajectory& traj) -> void;
    // 多点保锚点时间分配：中间途径点不减速，按各子段时长比例重采样并将锚点索引写入 traj.via_knot_indices。
    // opt_via_indices 为 traj.optimized_path 中有序的中间途径点顶点索引（0 < idx < size - 1）；空时等价于单点版本。
    auto assign_trajectory_timing_with_anchors(Trajectory& traj, const std::vector<size_t>& opt_via_indices) -> void;

    auto fill_additional_trajectory_info(
        Trajectory& traj,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        double start_yaw,
        double goal_yaw
    ) -> void;

    auto evaluate_duration(double length, double start_vel, double end_vel, double max_vel, double max_acc) const
        -> double;

    auto evaluate_length(
        double t,
        double total_length,
        double total_time,
        double start_vel,
        double end_vel,
        double max_vel,
        double max_acc
    ) const -> double;
};

} // namespace path_planning