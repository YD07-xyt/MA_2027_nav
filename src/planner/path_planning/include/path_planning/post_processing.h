#pragma once
#include "map/core/grid_map.hpp"
#include <Eigen/Core>
#include <vector>

namespace path_planning {

class PathPostProcessing {
public:
    // 高密度采样点，用于检测隧道/速度分配
    struct DenseSample {
        double linear_dist = 0.0;
        double equiv_dist = 0.0;
        double time = 0.0;
        Eigen::Vector2d pos = Eigen::Vector2d::Zero();
        double yaw = 0.0;
    };

    // 一个隧道区间
    struct TunnelInterval {
        int entry_idx = -1; // 入口在 dense 中的 index
        int exit_idx = -1; // 出口在 dense 中的 index
        double entry_s = 0.0; // 入口等效里程
        double exit_s = 0.0; // 出口等效里程
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

        double total_time = 0;
        double total_length = 0;
        double weighted_length = 0;

        bool if_cut = false;
        std::vector<Eigen::Vector3d> UnOccupied_positions;
        double UnOccupied_initT = 0.0;
    };

public:
    struct PathPostProcessingParams {
        double safe_threshold;
        double max_vel;
        double max_acc;
        double time_resolution;
        int min_traj_num;
        int max_traj_num = 12;
        double traj_cut_length;
        double distance_weight;
        double yaw_weight;
        double start_vel {0.0};
        double end_vel {0.0};
        double dense_sample_resolution = 0.1;
        double rotation_penalty_weight = 1.0;

        // 云台折叠相关
        double fold_time = 1.0; // 折叠所需时间 s
        double unfold_time = 1.0; // 展开所需时间 s
        double fold_margin = 0.5; // 折叠/展开安全时间裕量 s
        double fold_prep_speed = 0.05; // 入口前准备速度 m/s
        double fold_prep_margin = 0.3; // 额外安全距离 m
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
    auto assign_trajectory_timing(const grid_map::GridMap& map, Trajectory& traj) -> void;

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

    // ============================================================
    // 折叠方案
    // ============================================================

    // 检测 dense 路径上所有隧道区间
    auto detect_tunnel_intervals(const grid_map::GridMap& map, const std::vector<DenseSample>& dense) const
        -> std::vector<TunnelInterval>;

    // 根据隧道区间，在 vmax 上施加入口前降速
    auto apply_fold_speed_limits(
        std::vector<double>& vmax,
        const std::vector<DenseSample>& dense,
        const std::vector<TunnelInterval>& tunnels
    ) const -> void;
};

} // namespace path_planning