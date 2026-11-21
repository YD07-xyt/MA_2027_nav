#include "map/perception/dynamic_stvl_layer.hpp"

#include <algorithm>
#include <cmath>

namespace ma_map {

namespace {

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kHalfPi = 0.5 * kPi;
    constexpr double kRadToDeg = 180.0 / kPi;
    constexpr float kNarrowDriftGate = 0.25f; // 已锁定后允许跟踪的窄门限 (m)
    constexpr int kDriftReacquireMissFrames = 3; // 连续窄门限丢失帧数达到后立即用宽门限重捕获

    inline auto packXY(int32_t vx, int32_t vy) -> uint64_t {
        return (static_cast<uint64_t>(static_cast<uint32_t>(vx)) << 32U)
            | static_cast<uint64_t>(static_cast<uint32_t>(vy));
    }

    inline auto lidarDirToBins(const Eigen::Vector3d& pl, int& az_idx, int& el_idx) -> void {
        const double az_deg = (std::atan2(pl.y(), pl.x()) + kPi) * kRadToDeg;
        const double el_deg = (std::atan2(pl.z(), std::hypot(pl.x(), pl.y())) + kHalfPi) * kRadToDeg;
        az_idx = std::clamp(static_cast<int>(std::floor(az_deg)), 0, 359);
        el_idx = std::clamp(static_cast<int>(std::floor(el_deg)), 0, 179);
    }

    auto medianOf(std::vector<float>& vals) -> float {
        const size_t n = vals.size();
        const size_t mid = n / 2;
        std::nth_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(mid), vals.end());
        const float hi = vals[mid];
        if ((n & 1U) != 0U) {
            return hi;
        }
        const float lo = *std::max_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(mid));
        return 0.5f * (lo + hi);
    }

} // namespace

DynamicStvlLayer::DynamicStvlLayer() {
    setConfig(Config {});
}

DynamicStvlLayer::DynamicStvlLayer(const Config& cfg) {
    setConfig(cfg);
}

auto DynamicStvlLayer::setConfig(const Config& cfg) -> void {
    cfg_ = cfg;
    if (!std::isfinite(cfg_.voxel_size) || cfg_.voxel_size <= 0.0) {
        cfg_.voxel_size = 0.05;
    }
    inv_voxel_size_ = 1.0 / cfg_.voxel_size;
    range_img_.assign(static_cast<size_t>(kAzBins) * static_cast<size_t>(kElBins), 0.0f);
    drift_samples_.reserve(2048);
    drift_wide_samples_.reserve(2048);
    drift_inliers_.reserve(2048);
    erase_scratch_.reserve(512);
    confirmed_voxels_scratch_.reserve(std::min<size_t>(cfg_.max_active_voxels, 4096));
    emitted_xy_scratch_.reserve(std::min<size_t>(cfg_.max_active_voxels, 4096));
}

auto DynamicStvlLayer::reset() -> void {
    active_voxels_.clear();
    std::fill(range_img_.begin(), range_img_.end(), 0.0f);
    drift_samples_.clear();
    drift_wide_samples_.clear();
    drift_inliers_.clear();
    erase_scratch_.clear();
    confirmed_voxels_scratch_.clear();
    emitted_xy_scratch_.clear();
    frame_seq_ = 0;
    delta_z_ = 0.0f;
    has_delta_z_ = false;
    drift_miss_frames_ = 0;
    last_prior_ptr_ = nullptr;
    has_prev_pose_ = false;
    prev_q_base_w_ = Eigen::Quaterniond::Identity();
    tilt_hold_left_ = 0;
    has_last_time_ = false;
    last_time_was_explicit_ = false;
    last_explicit_sec_ = 0.0;
    last_wall_sec_ = 0.0;
}

auto DynamicStvlLayer::computeEffectiveDt(double now_sec) -> float {
    float raw_dt = cfg_.nominal_dt_sec;
    if (now_sec >= 0.0 && std::isfinite(now_sec)) {
        if (has_last_time_ && last_time_was_explicit_) {
            const double diff = now_sec - last_explicit_sec_;
            if (diff < 0.0) {
                // 仿真重置 / 时钟回退：本帧不推进衰减时间，并以新时间重建基线。
                last_explicit_sec_ = now_sec;
                last_time_was_explicit_ = true;
                has_last_time_ = true;
                return 0.0f;
            }
            raw_dt = static_cast<float>(diff);
        }
        last_explicit_sec_ = now_sec;
        last_time_was_explicit_ = true;
        has_last_time_ = true;
    } else {
        const double now_wall_sec = clock_.nowSec();
        if (has_last_time_ && !last_time_was_explicit_) {
            const double diff = now_wall_sec - last_wall_sec_;
            last_wall_sec_ = now_wall_sec;
            has_last_time_ = true;
            last_time_was_explicit_ = false;
            if (diff < 0.0) {
                return 0.0f;
            }
            raw_dt = static_cast<float>(diff);
        } else {
            last_wall_sec_ = now_wall_sec;
            last_time_was_explicit_ = false;
            has_last_time_ = true;
        }
    }
    return std::clamp(raw_dt, cfg_.dt_min_sec, cfg_.dt_max_sec);
}

auto DynamicStvlLayer::updateDriftEstimate(
    std::span<const Eigen::Vector3d> pts_world,
    const SensorPose& pose,
    const SensorModel& sensor,
    const StaticNavContext& static_ctx
) -> void {
    const StaticTerrainPrior* prior = static_ctx.terrain_prior.get();
    if (prior == nullptr || !prior->isValid()) {
        delta_z_ = 0.0f;
        has_delta_z_ = false;
        drift_miss_frames_ = 0;
        last_prior_ptr_ = nullptr;
        return;
    }
    if (prior != last_prior_ptr_) {
        delta_z_ = 0.0f;
        has_delta_z_ = false;
        drift_miss_frames_ = 0;
        last_prior_ptr_ = prior;
    }

    drift_samples_.clear();
    drift_wide_samples_.clear();
    const double max_r_sq = cfg_.nav_max_range * cfg_.nav_max_range;
    const double min_r_sq = sensor.config().min_range * sensor.config().min_range;
    const size_t stride = std::max<size_t>(1, pts_world.size() / 2048);

    for (size_t i = 0; i < pts_world.size(); i += stride) {
        const Eigen::Vector3d& pw = pts_world[i];
        if (!pw.allFinite() || sensor.isSelfHit(pw, pose)) {
            continue;
        }
        const double dx = pw.x() - pose.p_base_w.x();
        const double dy = pw.y() - pose.p_base_w.y();
        const double r_xy_sq = dx * dx + dy * dy;
        if (r_xy_sq < min_r_sq || r_xy_sq > max_r_sq) {
            continue;
        }
        const Eigen::Vector2d xy(pw.x(), pw.y());
        if (static_ctx.static_esdf != nullptr) {
            if (!static_ctx.static_esdf->isInsideMap(xy)) {
                continue;
            }
            if (static_ctx.static_esdf->getDistance(xy) <= cfg_.wall_suppress_dist) {
                continue;
            }
        }
        const float gz = prior->sampleGroundZ(xy, 1);
        if (!std::isfinite(gz)) {
            continue;
        }
        const float diff = static_cast<float>(pw.z()) - gz;
        if (std::abs(diff) > cfg_.drift_max_abs) {
            continue;
        }
        if (!has_delta_z_) {
            // 首次捕获用宽门限（绝对物理漂移上限），避免初始 LIO 漂移超过 0.25m 时锁死。
            drift_samples_.push_back(diff);
        } else {
            drift_wide_samples_.push_back(diff);
            if (std::abs(diff - delta_z_) <= kNarrowDriftGate) {
                drift_samples_.push_back(diff);
            }
        }
    }

    if (has_delta_z_) {
        if (static_cast<int>(drift_samples_.size()) < cfg_.drift_min_samples) {
            if (drift_miss_frames_ < kDriftReacquireMissFrames) {
                ++drift_miss_frames_;
            }
            // LIO Z 突跳 > 0.25m 时（即使有少量噪声点落在旧窄门限内），连续丢失 3 帧后用宽门限样本立即重捕获。
            if (drift_miss_frames_ >= kDriftReacquireMissFrames
                && static_cast<int>(drift_wide_samples_.size()) >= cfg_.drift_min_samples)
            {
                float est = 0.0f;
                if (estimateRobustDrift(drift_wide_samples_, est)) {
                    delta_z_ = est;
                    drift_miss_frames_ = 0;
                }
            }
            return;
        }
        drift_miss_frames_ = 0;
    }

    if (static_cast<int>(drift_samples_.size()) < cfg_.drift_min_samples) {
        return;
    }

    float est = 0.0f;
    if (!estimateRobustDrift(drift_samples_, est)) {
        return;
    }
    if (!has_delta_z_) {
        delta_z_ = est;
        has_delta_z_ = true;
    } else {
        delta_z_ = std::clamp(0.65f * delta_z_ + 0.35f * est, -cfg_.drift_max_abs, cfg_.drift_max_abs);
    }
}

auto DynamicStvlLayer::estimateRobustDrift(std::vector<float>& samples, float& est) -> bool {
    if (static_cast<int>(samples.size()) < cfg_.drift_min_samples) {
        return false;
    }

    // 用第 25 百分位作为地面下界锚点，再取 [p25 - 0.04, p25 + 0.06] 的中位数，
    // 在无障碍时零偏估计，有低矮动态障碍时完全免疫上拉。
    const size_t q25_idx = samples.size() / 4;
    std::nth_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(q25_idx), samples.end());
    const float p25 = samples[q25_idx];

    drift_inliers_.clear();
    for (float d: samples) {
        if (d >= p25 - 0.04f && d <= p25 + 0.06f) {
            drift_inliers_.push_back(d);
        }
    }
    if (static_cast<int>(drift_inliers_.size()) < std::max(10, cfg_.drift_min_samples / 2)) {
        return false;
    }

    est = std::clamp(medianOf(drift_inliers_), -cfg_.drift_max_abs, cfg_.drift_max_abs);
    return true;
}

auto DynamicStvlLayer::isRayClearedInRangeImage(int az_idx, int el_idx, float voxel_range) const -> bool {
    const float clear_thresh = voxel_range + cfg_.ray_clear_margin;
    bool has_behind_hit = false;
    for (int de = -1; de <= 1; ++de) {
        const int e = el_idx + de;
        if (e < 0 || e >= kElBins) {
            continue;
        }
        const size_t row_off = static_cast<size_t>(e) * static_cast<size_t>(kAzBins);
        for (int da = -1; da <= 1; ++da) {
            const int a = (az_idx + da + kAzBins) % kAzBins;
            const float r_bin = range_img_[row_off + static_cast<size_t>(a)];
            if (r_bin <= 0.0f) {
                continue;
            }
            if (r_bin <= clear_thresh) {
                return false;
            }
            has_behind_hit = true;
        }
    }
    return has_behind_hit;
}

auto DynamicStvlLayer::update(
    std::span<const Eigen::Vector3d> pts_world,
    const SensorPose& pose,
    const Eigen::Quaterniond& q_base_w,
    const SensorModel& sensor,
    const StaticNavContext& static_ctx,
    double now_sec,
    std::vector<Eigen::Vector3f>* debug_cloud
) -> LocalTerrainFrame {
    LocalTerrainFrame frame;
    frame.terrain_resolution = cfg_.voxel_size;
    if (pose.p_base_w.allFinite()) {
        frame.robot_pos = Eigen::Vector2d(pose.p_base_w.x(), pose.p_base_w.y());
        frame.has_robot_pos = true;
    }

    if (!pose.p_base_w.allFinite() || !pose.p_lidar_w.allFinite()) {
        return frame;
    }

    sensor.checkExtrinsicsOnce(pts_world, pose);
    const float effective_dt = computeEffectiveDt(now_sec);

    // 1. 姿态跳变门控
    if (has_prev_pose_) {
        const double tilt_deg = SensorModel::tiltAngleDeg(prev_q_base_w_, q_base_w);
        if (tilt_deg > cfg_.tilt_jump_deg) {
            tilt_hold_left_ = std::max(tilt_hold_left_, cfg_.tilt_hold_frames);
        }
    }
    prev_q_base_w_ = (q_base_w.squaredNorm() > 1e-12) ? q_base_w.normalized() : Eigen::Quaterniond::Identity();
    has_prev_pose_ = true;

    const bool tilt_held = (tilt_hold_left_ > 0);
    if (tilt_held) {
        --tilt_hold_left_;
    }

    ++frame_seq_;
    if (frame_seq_ == 0) {
        frame_seq_ = 1;
    }
    const uint32_t cur_frame = frame_seq_;

    // 2. 非倾角跳变帧：更新 Z 漂移、填充极坐标深度图，并按静态先验差分写入活跃动态体素
    if (!tilt_held) {
        updateDriftEstimate(pts_world, pose, sensor, static_ctx);

        std::fill(range_img_.begin(), range_img_.end(), 0.0f);

        const StaticTerrainPrior* prior = (static_ctx.terrain_prior && static_ctx.terrain_prior->isValid())
            ? static_ctx.terrain_prior.get()
            : nullptr;
        const grid_map::GridMap* static_esdf = static_ctx.static_esdf.get();
        const grid_map::GridMap* sem_snap = static_ctx.current_snapshot.get();

        const double min_r_sq = sensor.config().min_range * sensor.config().min_range;
        const double sensor_max_r_sq = sensor.config().max_range * sensor.config().max_range;
        const double nav_max_r_sq = cfg_.nav_max_range * cfg_.nav_max_range;
        const Eigen::Vector3d base_z_axis = pose.R_w_base.col(2);

        for (const Eigen::Vector3d& pw: pts_world) {
            if (!pw.allFinite() || sensor.isSelfHit(pw, pose)) {
                continue;
            }
            const Eigen::Vector3d pl = pose.worldToLidar(pw);
            const double r_sq = pl.squaredNorm();
            if (r_sq < min_r_sq || r_sq > sensor_max_r_sq) {
                continue;
            }
            const float r_f = static_cast<float>(std::sqrt(r_sq));

            // 所有有效非自遮挡回波均写入极坐标深度图（含远墙、地面、限高杆），作为射线清除背景证据
            int az_idx = 0;
            int el_idx = 0;
            lidarDirToBins(pl, az_idx, el_idx);
            float& bin_r =
                range_img_[static_cast<size_t>(el_idx) * static_cast<size_t>(kAzBins) + static_cast<size_t>(az_idx)];
            if (bin_r <= 0.0f || r_f < bin_r) {
                bin_r = r_f;
            }

            // 以下进入动态障碍前景筛选
            const double dx_b = pw.x() - pose.p_base_w.x();
            const double dy_b = pw.y() - pose.p_base_w.y();
            const double r_xy_sq = dx_b * dx_b + dy_b * dy_b;
            if (r_xy_sq > nav_max_r_sq) {
                continue;
            }

            const Eigen::Vector2d xy(pw.x(), pw.y());

            // 静态边界与纯静态 ESDF 墙体光晕抑制（彻底消除墙边抖动与高台边沿残影）
            if (static_esdf != nullptr) {
                if (!static_esdf->isInsideMap(xy)) {
                    continue;
                }
                if (static_esdf->getDistance(xy) <= cfg_.wall_suppress_dist) {
                    continue;
                }
            }

            // 2.5D 地面高度与悬空下沿剥离：有先验时用先验地面 + Z 漂移；
            // 无先验（或先验缺格）时用机体倾斜地面平面，避免上下坡把坡面点误判为动态障碍。
            float height_above_ground = 0.0f;
            float z_corr = static_cast<float>(pw.z());
            bool used_prior = false;
            if (prior != nullptr) {
                z_corr = static_cast<float>(pw.z()) - delta_z_;
                const float prior_gz = prior->sampleGroundZ(xy, 2);
                if (std::isfinite(prior_gz)) {
                    height_above_ground = z_corr - prior_gz;
                    used_prior = true;
                }
            }
            if (!used_prior) {
                // z_rel_base = base_z_axis · (pw - p_base)：平地时 base_z_axis=(0,0,1)，
                // 完全退化为 pw.z - (p_base.z - fallback_ground_offset)。
                const Eigen::Vector3d rel = pw - pose.p_base_w;
                height_above_ground = static_cast<float>(base_z_axis.dot(rel)) + cfg_.fallback_ground_offset;
            }

            const float r_xy = static_cast<float>(std::sqrt(r_xy_sq));
            const float ground_margin = cfg_.ground_margin_base + cfg_.ground_margin_per_m * r_xy;
            if (height_above_ground < ground_margin || height_above_ground > cfg_.dynamic_max_height) {
                continue;
            }

            // 已知静态悬空结构（限高杆/洞顶）抑制
            if (prior != nullptr) {
                const float oh_z = prior->sampleOverhangMinZ(xy, 1);
                if (std::isfinite(oh_z) && z_corr >= oh_z - cfg_.overhang_margin) {
                    continue;
                }
            } else if (sem_snap != nullptr && sem_snap->isInsideMap(xy) && sem_snap->is_tunnel(xy)
                       && height_above_ground >= cfg_.tunnel_clearance_z)
            {
                continue;
            }

            const VoxelKey key {
                static_cast<int32_t>(std::floor(pw.x() * inv_voxel_size_)),
                static_cast<int32_t>(std::floor(pw.y() * inv_voxel_size_)),
                static_cast<int32_t>(std::floor(pw.z() * inv_voxel_size_)),
            };
            auto it = active_voxels_.find(key);
            if (it == active_voxels_.end()) {
                if (active_voxels_.size() >= cfg_.max_active_voxels) {
                    continue;
                }
                DynamicVoxel dv;
                dv.last_hit_frame = cur_frame;
                dv.hit_frames = 1;
                dv.pts_in_frame = 1;
                dv.miss_in_fov = 0;
                dv.age_sec = 0.0f;
                active_voxels_.emplace(key, dv);
            } else {
                DynamicVoxel& dv = it->second;
                if (dv.last_hit_frame != cur_frame) {
                    dv.last_hit_frame = cur_frame;
                    if (dv.hit_frames < 1000) {
                        ++dv.hit_frames;
                    }
                    dv.pts_in_frame = 1;
                    dv.miss_in_fov = 0;
                    dv.age_sec = 0.0f;
                } else if (dv.pts_in_frame < 1000) {
                    ++dv.pts_in_frame;
                }
            }
        }
    }

    // 3. STVL 时空衰减 + 极坐标 3x3 射线清除
    erase_scratch_.clear();
    const double keep_r_sq = (cfg_.nav_max_range + 1.0) * (cfg_.nav_max_range + 1.0);
    for (auto& kv: active_voxels_) {
        const VoxelKey& key = kv.first;
        DynamicVoxel& dv = kv.second;
        if (dv.last_hit_frame == cur_frame) {
            continue;
        }

        dv.age_sec += effective_dt;
        dv.pts_in_frame = 0;

        const Eigen::Vector3d p_vox(
            (static_cast<double>(key.vx) + 0.5) * cfg_.voxel_size,
            (static_cast<double>(key.vy) + 0.5) * cfg_.voxel_size,
            (static_cast<double>(key.vz) + 0.5) * cfg_.voxel_size
        );
        const double dx_b = p_vox.x() - pose.p_base_w.x();
        const double dy_b = p_vox.y() - pose.p_base_w.y();
        if (dx_b * dx_b + dy_b * dy_b > keep_r_sq) {
            erase_scratch_.push_back(key);
            continue;
        }

        const Eigen::Vector3d pl = pose.worldToLidar(p_vox);
        const bool in_fov = sensor.inFov(pl, cfg_.nav_max_range);
        // 射线清除不再受名义 inFov() 硬门控：只要极坐标深度图在体素方向存在严格更远的真实回波，
        // 该激光束就物理穿过了体素（即使落在视场边界容差附近）。in_fov 仍只用于选择衰减时间常数。
        if (!tilt_held && pl.allFinite()) {
            int az_idx = 0;
            int el_idx = 0;
            lidarDirToBins(pl, az_idx, el_idx);
            const float voxel_r = static_cast<float>(pl.norm());
            if (isRayClearedInRangeImage(az_idx, el_idx, voxel_r)) {
                ++dv.miss_in_fov;
            }
        }

        const bool ray_erased = (static_cast<int>(dv.miss_in_fov) >= cfg_.miss_clear_frames);
        const bool time_expired = in_fov ? (dv.age_sec > cfg_.decay_in_fov_sec) : (dv.age_sec > cfg_.decay_blind_sec);
        if (ray_erased || time_expired) {
            erase_scratch_.push_back(key);
        }
    }
    for (const auto& k: erase_scratch_) {
        active_voxels_.erase(k);
    }

    // 4. 动态体素确认（2 帧确认，或单帧 3D 成簇即刻确认）并投影到 2D obstacle_cells
    // 使用可复用 scratch 向量排序去重，避免每帧构造 unordered_set 的节点分配。
    if (debug_cloud != nullptr) {
        debug_cloud->clear();
        debug_cloud->reserve(active_voxels_.size());
    }
    confirmed_voxels_scratch_.clear();

    for (const auto& kv: active_voxels_) {
        const VoxelKey& key = kv.first;
        const DynamicVoxel& dv = kv.second;
        bool confirmed = (static_cast<int>(dv.hit_frames) >= cfg_.confirm_min_frames);
        if (!confirmed && cfg_.fast_confirm_with_neighbor && dv.last_hit_frame == cur_frame && dv.pts_in_frame >= 1) {
            int neighbor_hits = 0;
            for (int dx = -1; dx <= 1 && neighbor_hits < 2; ++dx) {
                for (int dy = -1; dy <= 1 && neighbor_hits < 2; ++dy) {
                    for (int dz = -1; dz <= 1 && neighbor_hits < 2; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const auto nit = active_voxels_.find(VoxelKey {key.vx + dx, key.vy + dy, key.vz + dz});
                        if (nit != active_voxels_.end() && nit->second.last_hit_frame == cur_frame) {
                            ++neighbor_hits;
                        }
                    }
                }
            }
            confirmed = (neighbor_hits >= 2);
        }

        if (!confirmed) {
            continue;
        }

        confirmed_voxels_scratch_.push_back(key);
        if (debug_cloud != nullptr) {
            debug_cloud->emplace_back(
                static_cast<float>((static_cast<double>(key.vx) + 0.5) * cfg_.voxel_size),
                static_cast<float>((static_cast<double>(key.vy) + 0.5) * cfg_.voxel_size),
                static_cast<float>((static_cast<double>(key.vz) + 0.5) * cfg_.voxel_size)
            );
        }
    }

    std::sort(
        confirmed_voxels_scratch_.begin(),
        confirmed_voxels_scratch_.end(),
        [](const VoxelKey& a, const VoxelKey& b) {
            if (a.vx != b.vx) {
                return a.vx < b.vx;
            }
            if (a.vy != b.vy) {
                return a.vy < b.vy;
            }
            return a.vz < b.vz;
        }
    );

    emitted_xy_scratch_.clear();
    for (const VoxelKey& key: confirmed_voxels_scratch_) {
        const uint64_t xy = packXY(key.vx, key.vy);
        if (!emitted_xy_scratch_.empty() && emitted_xy_scratch_.back() == xy) {
            continue;
        }
        emitted_xy_scratch_.push_back(xy);
        frame.obstacle_cells.emplace_back(
            (static_cast<double>(key.vx) + 0.5) * cfg_.voxel_size,
            (static_cast<double>(key.vy) + 0.5) * cfg_.voxel_size,
            (static_cast<double>(key.vz) + 0.5) * cfg_.voxel_size
        );
    }

    return frame;
}

} // namespace ma_map
