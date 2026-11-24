#include "path_planning/post_processing.h"

namespace path_planning {
auto PathPostProcessing::optimize_path(const grid_map::GridMap& map, const std::vector<Eigen::Vector2d>& path)
    -> std::vector<Eigen::Vector2d> {
    if (path.size() < 2) {
        return path;
    }
    std::vector<Eigen::Vector2d> optimized_path;
    optimized_path.push_back(path[0]);
    Eigen::Vector2d prev_pose = path[0];
    double cost1, cost2, cost3;

    // Check first path segment
    if (!check_line_collision(map, path[0], path[1])) {
        cost1 = (path[0] - path[1]).norm();
    } else {
        cost1 = std::numeric_limits<double>::infinity();
    }
    for (size_t i = 1; i < path.size() - 1; ++i) {
        const auto& pose1 = path[i];
        const auto& pose2 = path[i + 1];

        // Calculate current segment cost
        if (!check_line_collision(map, pose1, pose2)) {
            cost2 = (pose1 - pose2).norm();
        } else {
            cost2 = std::numeric_limits<double>::infinity();
        }
        // Calculate skip current point cost
        if (!check_line_collision(map, prev_pose, pose2)) {
            cost3 = (prev_pose - pose2).norm();
        } else {
            cost3 = std::numeric_limits<double>::infinity();
        }
        // Decide whether to skip current point
        if (cost3 < cost1 + cost2) {
            cost1 = cost3;
        } else {
            optimized_path.push_back(path[i]);
            cost1 = (pose1 - pose2).norm();
            prev_pose = pose1;
        }
    }

    optimized_path.push_back(path.back());
    return optimized_path;
}
auto PathPostProcessing::check_collision(const grid_map::GridMap& map, const Eigen::Vector2d& pos) const -> bool {
    return map.isCollision(pos, params_.safe_threshold);
}

/*  Bresenham 直线算法 枚举线段经过的所有栅格索引，
    并逐一调用 check_collision，判断两点间的直线段是否穿过障碍物，
    用于后续路径优化时的跳过可行性判断*/
auto PathPostProcessing::check_line_collision(
    const grid_map::GridMap& map,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end
) const -> bool {
    Eigen::Vector2i start_idx, end_idx;
    map.posToIndex(start, start_idx);
    map.posToIndex(end, end_idx);

    int dx = abs(end_idx.x() - start_idx.x());
    int dy = abs(end_idx.y() - start_idx.y());
    int sx = start_idx.x() < end_idx.x() ? 1 : -1;
    int sy = start_idx.y() < end_idx.y() ? 1 : -1;
    int err = dx - dy;

    while (true) {
        Eigen::Vector2d pos;
        map.indexToPos(start_idx, pos);
        if (check_collision(map, pos)) return true;
        if (start_idx == end_idx) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            start_idx.x() += sx;
        }
        if (e2 < dx) {
            err += dx;
            start_idx.y() += sy;
        }
    }
    return false;
}

auto PathPostProcessing::sample_path_states(const std::vector<Eigen::Vector2d>& path) -> std::vector<PathState> {
    std::vector<PathState> states;
    if (path.empty()) return states;

    // Initial state
    states.push_back({path[0], 0, 0, 0});

    for (size_t i = 1; i < path.size(); ++i) {
        const auto& prev = states.back();
        const auto& curr_pos = path[i];

        // Calculate orientation angle
        double theta = atan2(curr_pos.y() - prev.position.y(), curr_pos.x() - prev.position.x());

        // Normalize angle
        normalize_angle(prev.theta, theta);

        // Calculate segment length
        double delta_s = (curr_pos - prev.position).norm();

        states.push_back({curr_pos, theta, theta - prev.theta, delta_s});
    }
    return states;
}

auto PathPostProcessing::normalize_angle(double ref_angle, double& angle) const -> void {
    while (ref_angle - angle > M_PI)
        angle += 2 * M_PI;
    while (ref_angle - angle < -M_PI)
        angle -= 2 * M_PI;
}

auto PathPostProcessing::assign_trajectory_timing(const grid_map::GridMap& map, Trajectory& traj) -> void {
    /**1. 高密度采样
        -> 保留路径细节，避免拐角信息丢失

        2. 转弯惩罚
        -> 过弯多的区域等效距离更长，时间分配更充裕

        3. 前向后向速度传播
        -> 局部限速更真实，避免末端速度/时间不合理

        4. min/max waypoints
        -> 优化变量数量可控，不会因为 time_resolution 太小而爆炸

        5. 均匀初始时间
        -> 每段时间相等，优化器在此基础上微调 */

    if (traj.optimized_path.size() < 2) return;

    // ============================================================
    // 1. 高密度采样 + 等效距离（含转弯惩罚）
    // ============================================================

    std::vector<DenseSample> dense;

    // 起点
    Eigen::Vector2d start_pos = traj.optimized_path.front();
    double start_yaw = 0.0;
    if (!traj.path_states.empty()) start_yaw = traj.path_states.front().theta;

    dense.push_back({0.0, 0.0, 0.0, start_pos, start_yaw});

    Eigen::Vector2d last_pos = start_pos;
    double last_yaw = start_yaw;

    double accumulated_linear = 0.0;
    double accumulated_equiv = 0.0;

    for (size_t i = 1; i < traj.optimized_path.size(); ++i) {
        const Eigen::Vector2d curr_pos = traj.optimized_path[i];

        const double seg_len = (curr_pos - last_pos).norm();
        double curr_yaw = std::atan2(curr_pos.y() - last_pos.y(), curr_pos.x() - last_pos.x());

        // 角度连续化
        double d_yaw = curr_yaw - last_yaw;
        while (d_yaw > M_PI)
            d_yaw -= 2.0 * M_PI;
        while (d_yaw < -M_PI)
            d_yaw += 2.0 * M_PI;

        // 高密度细分
        const int subdivisions = std::max(1, static_cast<int>(std::ceil(seg_len / params_.dense_sample_resolution)));

        for (int j = 1; j <= subdivisions; ++j) {
            const double ratio = static_cast<double>(j) / subdivisions;

            const Eigen::Vector2d pos = last_pos + ratio * (curr_pos - last_pos);

            const double yaw = last_yaw + ratio * d_yaw;

            const double ds_linear = (pos - dense.back().pos).norm();

            double d_theta = std::abs(yaw - dense.back().yaw);
            while (d_theta > M_PI)
                d_theta = 2.0 * M_PI - d_theta;

            // 等效距离 = 线性距离 + 转弯惩罚 * |Δθ|
            const double ds_equiv = ds_linear + params_.rotation_penalty_weight * d_theta;

            accumulated_linear += ds_linear;
            accumulated_equiv += ds_equiv;

            dense.push_back({accumulated_linear, accumulated_equiv, 0.0, pos, yaw});
        }

        last_pos = curr_pos;
        last_yaw = curr_yaw;
    }

    if (dense.size() < 2) return;

    traj.total_length = accumulated_linear;
    traj.weighted_length = accumulated_equiv;

    // ============================================================
    // 2. 前向 / 后向速度传播
    // ============================================================
    const int M = static_cast<int>(dense.size());

    std::vector<double> vmax(M, params_.max_vel);
    vmax.front() = params_.start_vel;
    vmax.back() = params_.end_vel;

    // ============================================================
    // 折叠降速
    // ============================================================
    std::vector<TunnelInterval> tunnel_intervals = detect_tunnel_intervals(map, dense);

    apply_fold_speed_limits(vmax, dense, tunnel_intervals);
    ///==================================================

    ///==================================================////
    // 前向：受加速能力限制
    for (int i = 1; i < M; ++i) {
        const double ds = dense[i].equiv_dist - dense[i - 1].equiv_dist;
        const double v_forward = std::sqrt(std::max(0.0, vmax[i - 1] * vmax[i - 1] + 2.0 * params_.max_acc * ds));

        vmax[i] = std::min(vmax[i], v_forward);
    }

    // 后向：受减速能力限制
    for (int i = M - 2; i >= 0; --i) {
        const double ds = dense[i + 1].equiv_dist - dense[i].equiv_dist;
        const double v_backward = std::sqrt(std::max(0.0, vmax[i + 1] * vmax[i + 1] + 2.0 * params_.max_acc * ds));

        vmax[i] = std::min(vmax[i], v_backward);
    }

    // ============================================================
    // 3. 积分得到到达时间
    // ============================================================
    dense.front().time = 0.0;
    for (int i = 1; i < M; ++i) {
        const double ds = dense[i].equiv_dist - dense[i - 1].equiv_dist;
        const double v_avg = 0.5 * (vmax[i - 1] + vmax[i]);

        if (v_avg > 1e-6) dense[i].time = dense[i - 1].time + ds / v_avg;
        else dense[i].time = dense[i - 1].time;
    }

    const double total_time = dense.back().time;
    if (total_time <= 1e-6) return;

    traj.total_time = total_time;

    // ============================================================
    // 4. 限制 min/max waypoints，生成均匀初始时间
    // ============================================================
    int num_segments = static_cast<int>(std::lround(total_time / params_.time_resolution));

    num_segments = std::max(params_.min_traj_num, num_segments);
    num_segments = std::min(params_.max_traj_num, num_segments);

    const double sample_time = total_time / num_segments;

    traj.time_segments = Eigen::VectorXd::Constant(num_segments, sample_time);

    // ============================================================
    // 5. 按均匀时间重采样 timed_trajectory
    // ============================================================
    auto interpolateDense = [&](double t) -> std::pair<Eigen::Vector2d, double> {
        if (t <= 0.0) return {dense.front().pos, dense.front().yaw};

        if (t >= total_time - 1e-9) return {dense.back().pos, dense.back().yaw};

        size_t idx = 1;
        while (idx < dense.size() && dense[idx].time < t)
            ++idx;

        if (idx >= dense.size()) return {dense.back().pos, dense.back().yaw};

        const auto& a = dense[idx - 1];
        const auto& b = dense[idx];

        const double dt = b.time - a.time;
        const double r = dt > 1e-9 ? (t - a.time) / dt : 0.0;

        Eigen::Vector2d pos = a.pos + r * (b.pos - a.pos);
        double yaw = a.yaw + r * (b.yaw - a.yaw);

        return {pos, yaw};
    };

    traj.timed_trajectory.clear();
    traj.timed_trajectory.reserve(num_segments + 1);

    for (int k = 0; k <= num_segments; ++k) {
        const double t = k * sample_time;
        const auto [pos, yaw] = interpolateDense(t);

        TimedTrajectoryPoint point;
        point.state = Eigen::Vector3d(pos.x(), pos.y(), yaw);
        point.time = t;
        traj.timed_trajectory.push_back(point);
    }
}

auto PathPostProcessing::fill_additional_trajectory_info(
    Trajectory& traj,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    double start_yaw,
    double goal_yaw
) -> void {
    // Set start/goal states
    traj.start_state_XYTheta << start.x(), start.y(), start_yaw;
    traj.final_state_XYTheta << goal.x(), goal.y(), goal_yaw;

    // timed_trajectory 通常已经包含首尾点，联合优化会直接使用它们。
    if (!traj.timed_trajectory.empty()) {
        traj.timed_trajectory.front().state.z() = start_yaw;
        traj.timed_trajectory.back().state.z() = goal_yaw;
    }

    // Set default velocity/acceleration (can be adjusted as needed)
    traj.start_state = start_velocity_vector_; // Initial velocity 0
    traj.final_state = final_velocity_vector_; // Final velocity 0
}

auto PathPostProcessing::evaluate_duration(
    double length,
    double start_vel,
    double end_vel,
    double max_vel,
    double max_acc
) const -> double {
    double critical_len = (max_vel * max_vel - start_vel * start_vel) / (2 * max_acc)
        + (max_vel * max_vel - end_vel * end_vel) / (2 * max_acc);

    if (length >= critical_len) {
        return (max_vel - start_vel) / max_acc + (max_vel - end_vel) / max_acc + (length - critical_len) / max_vel;
    } else {
        double tmp_vel = sqrt(0.5 * (start_vel * start_vel + end_vel * end_vel + 2 * max_acc * length));
        return (tmp_vel - start_vel) / max_acc + (tmp_vel - end_vel) / max_acc;
    }
}

auto PathPostProcessing::evaluate_length(
    double t,
    double total_length,
    double total_time,
    double start_vel,
    double end_vel,
    double max_vel,
    double max_acc
) const -> double {
    double critical_len = (max_vel * max_vel - start_vel * start_vel) / (2 * max_acc)
        + (max_vel * max_vel - end_vel * end_vel) / (2 * max_acc);

    if (total_length >= critical_len) {
        double t1 = (max_vel - start_vel) / max_acc;
        double t2 = t1 + (total_length - critical_len) / max_vel;

        if (t <= t1) {
            return start_vel * t + 0.5 * max_acc * t * t;
        } else if (t <= t2) {
            return start_vel * t1 + 0.5 * max_acc * t1 * t1 + (t - t1) * max_vel;
        } else {
            return start_vel * t1 + 0.5 * max_acc * t1 * t1 + (t2 - t1) * max_vel + max_vel * (t - t2)
                - 0.5 * max_acc * (t - t2) * (t - t2);
        }
    } else {
        double tmp_vel = sqrt(0.5 * (start_vel * start_vel + end_vel * end_vel + 2 * max_acc * total_length));
        double tmp_t = (tmp_vel - start_vel) / max_acc;

        if (t <= tmp_t) {
            return start_vel * t + 0.5 * max_acc * t * t;
        } else {
            return start_vel * tmp_t + 0.5 * max_acc * tmp_t * tmp_t + tmp_vel * (t - tmp_t)
                - 0.5 * max_acc * (t - tmp_t) * (t - tmp_t);
        }
    }
}
//对每个隧道入口，在入口前一段距离内限速：
auto PathPostProcessing::apply_fold_speed_limits(
    std::vector<double>& vmax,
    const std::vector<DenseSample>& dense,
    const std::vector<TunnelInterval>& tunnels
) const -> void {
    if (tunnels.empty() || dense.empty()) {
        return;
    }

    // 入口前需要多长距离保持低速，才能保证有 fold_time 的折叠时间
    const double prep_dist = params_.fold_prep_speed * params_.fold_time + params_.fold_prep_margin;

    for (const auto& tunnel: tunnels) {
        if (tunnel.entry_idx <= 0) {
            continue;
        }

        const double entry_s = tunnel.entry_s;
        const double min_s = entry_s - prep_dist;

        // 从入口前一个点开始，向前反向遍历
        for (int i = tunnel.entry_idx - 1; i >= 0; --i) {
            if (dense[i].equiv_dist <= min_s) {
                break; // 已经超出准备距离，再往前不需要限速
            }
            vmax[i] = std::min(vmax[i], params_.fold_prep_speed);
        }
    }
}
//用状态机扫描：
auto PathPostProcessing::detect_tunnel_intervals(const grid_map::GridMap& map, const std::vector<DenseSample>& dense)
    const -> std::vector<TunnelInterval> {
    std::vector<TunnelInterval> intervals;

    if (dense.empty()) {
        return intervals;
    }

    bool inside = false;
    TunnelInterval current;

    for (int i = 0; i < static_cast<int>(dense.size()); ++i) {
        const bool in_tunnel = map.is_tunnel(dense[i].pos);

        if (!inside && in_tunnel) {
            // 进入隧道
            inside = true;
            current.entry_idx = i;
            current.entry_s = dense[i].equiv_dist;
        }

        if (inside && !in_tunnel) {
            // 离开隧道
            inside = false;
            current.exit_idx = i;
            current.exit_s = dense[i].equiv_dist;
            intervals.push_back(current);
        }
    }

    // 如果终点仍处于隧道内
    if (inside) {
        current.exit_idx = static_cast<int>(dense.size()) - 1;
        current.exit_s = dense.back().equiv_dist;
        intervals.push_back(current);
    }

    return intervals;
}
}
