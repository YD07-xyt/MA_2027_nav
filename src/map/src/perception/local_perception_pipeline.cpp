#include "local_perception_pipeline.hpp"

#include "utils/logger.hpp"

#include <cmath>
#include <utility>

namespace ma_map {

LocalPerceptionPipeline::LocalPerceptionPipeline(const rog_map::Config& cfg):
    cfg_(cfg),
    use_stvl_prior_(cfg.perception_backend == "stvl_prior") {
    // 始终初始化 rog_map_ptr_，保障既有 visualizer::map_viz_callback 与 odom 状态查询安全无空指针。
    rog_map_ptr_ = std::make_shared<rog_map::ROGMap>();
    rog_map_ptr_->setConfig(cfg_);
    rog_map_ptr_->init();

    // 初始化 legacy_rog 地形与持久地面层组件（回退后端与 get_ground_layer() 兼容）
    Terrain::TerrainAnalyzer::Config terrain_cfg;
    terrain_cfg.kernel_size = cfg_.terrain_kernel_size;
    terrain_cfg.max_step_height = cfg_.terrain_max_step_height;
    terrain_cfg.robot_height = cfg_.terrain_robot_height;
    terrain_cfg.steep_threshold = cfg_.terrain_steep_threshold;
    terrain_cfg.min_overhang_clearance = cfg_.terrain_min_overhang_clearance;
    terrain_cfg.overhang_free_voxels = cfg_.terrain_overhang_free_voxels;
    terrain_cfg.overhang_unknown_skip = cfg_.terrain_overhang_unknown_skip;
    terrain_cfg.overhang_ground_radius = cfg_.terrain_overhang_ground_radius;
    terrain_cfg.obstacle_band = cfg_.terrain_obstacle_band;
    terrain_cfg.ground_jump_thresh = cfg_.terrain_ground_jump_thresh;
    terrain_cfg.ground_min_radius = cfg_.terrain_ground_min_radius;
    terrain_cfg.reach_en = cfg_.terrain_reach_en;
    terrain_cfg.max_slope_deg = cfg_.terrain_max_slope_deg;
    terrain_cfg.reach_step_tol = cfg_.terrain_reach_step_tol;
    terrain_cfg.plane_rms_max = cfg_.terrain_plane_rms_max;
    terrain_cfg.drop_edge_en = cfg_.terrain_drop_edge_en;
    terrain_cfg.drop_thresh = cfg_.terrain_drop_thresh;
    terrain_cfg.resolution = static_cast<float>(cfg_.resolution);
    terrain_cfg.map_size_x = static_cast<float>(cfg_.local_update_box_d.x());
    terrain_cfg.map_size_y = static_cast<float>(cfg_.local_update_box_d.y());
    terrain_analyzer_.setConfig(terrain_cfg);

    if (cfg_.ground_layer_en) {
        Terrain::GroundLayer::Config gl;
        gl.resolution = static_cast<float>(cfg_.resolution);
        gl.size = cfg_.ground_layer_size;
        gl.min_range = static_cast<float>(cfg_.raycast_range_min);
        gl.max_range = cfg_.ground_layer_max_range;
        gl.drop_votes = cfg_.ground_layer_drop_votes;
        gl.rise_votes = cfg_.ground_layer_rise_votes;
        gl.rise_max = cfg_.ground_layer_rise_max;
        gl.footprint_radius = cfg_.ground_layer_footprint_radius;
        gl.footprint_ground_offset = cfg_.ground_layer_footprint_offset;
        gl.seed_radius = cfg_.ground_layer_seed_radius;
        gl.float_free_voxels = cfg_.terrain_overhang_free_voxels;
        gl.tilt_jump_deg = cfg_.ground_layer_tilt_jump_deg;
        gl.tilt_hold_frames = cfg_.ground_layer_tilt_hold_frames;
        gl.drop_min_support = cfg_.ground_layer_drop_min_support;
        ground_layer_.setConfig(gl);
        terrain_analyzer_.setGroundLayer(&ground_layer_);
    }

    // 初始化 stvl_prior 感知后端组件
    SensorModel::Config scfg;
    scfg.t_base_lidar = cfg_.perception_t_base_lidar;
    scfg.rpy_deg = cfg_.perception_rpy_deg;
    scfg.fov_min_elev_deg = cfg_.perception_fov_min_elev_deg;
    scfg.fov_max_elev_deg = cfg_.perception_fov_max_elev_deg;
    scfg.fov_margin_deg = cfg_.perception_fov_margin_deg;
    scfg.min_range = cfg_.perception_sensor_min_range;
    scfg.max_range = cfg_.perception_sensor_max_range;
    scfg.self_filter_min = cfg_.perception_self_filter_min;
    scfg.self_filter_max = cfg_.perception_self_filter_max;
    sensor_model_.setConfig(scfg);

    OnlineTerrainMapper::Config mcfg;
    mcfg.resolution = cfg_.grid_map_resolution;
    mcfg.trust_range = cfg_.perception_map_trust_range;
    mcfg.clear_range = cfg_.perception_map_clear_range;
    mcfg.clear_stop_margin = cfg_.perception_map_clear_stop_margin;
    mcfg.min_z = cfg_.perception_map_min_z;
    mcfg.max_z = cfg_.perception_map_max_z;
    mcfg.static_min_hits = cfg_.perception_map_static_min_hits;
    mcfg.static_clear_ratio = cfg_.perception_map_static_clear_ratio;
    mcfg.preview_radius = cfg_.perception_map_preview_radius;
    mcfg.footprint_radius = cfg_.ground_layer_en ? static_cast<double>(cfg_.ground_layer_footprint_radius) : 0.20;
    mcfg.footprint_ground_offset = cfg_.ground_layer_en ? static_cast<double>(cfg_.ground_layer_footprint_offset) : 0.0;
    mcfg.tilt_jump_deg = cfg_.perception_map_tilt_jump_deg;
    mcfg.tilt_hold_frames = cfg_.perception_map_tilt_hold_frames;
    mcfg.terrain.robot_height = cfg_.terrain_robot_height;
    mcfg.terrain.min_overhang_clearance = cfg_.terrain_min_overhang_clearance;
    mcfg.terrain.max_slope_deg = cfg_.perception_map_max_slope_deg;
    mcfg.terrain.step_tol = cfg_.perception_map_step_tol;
    mcfg.terrain.plane_rms_max = cfg_.perception_map_plane_rms_max;
    mcfg.terrain.max_surface_thickness = cfg_.perception_map_max_surface_thickness;
    mcfg.terrain.overhang_ground_radius = cfg_.terrain_overhang_ground_radius;
    online_mapper_.setConfig(mcfg);

    DynamicStvlLayer::Config dcfg;
    dcfg.voxel_size = cfg_.grid_map_resolution;
    dcfg.nav_max_range = cfg_.perception_nav_max_range;
    dcfg.wall_suppress_dist = cfg_.perception_nav_wall_suppress_dist;
    dcfg.ground_margin_base = cfg_.perception_nav_ground_margin_base;
    dcfg.ground_margin_per_m = cfg_.perception_nav_ground_margin_per_m;
    dcfg.dynamic_max_height = cfg_.perception_nav_dynamic_max_height;
    dcfg.overhang_margin = cfg_.perception_nav_overhang_margin;
    dcfg.tunnel_clearance_z = cfg_.perception_nav_tunnel_clearance_z;
    dcfg.fallback_ground_offset = cfg_.ground_layer_en ? static_cast<double>(cfg_.ground_layer_footprint_offset) : 0.0;
    dcfg.ray_clear_margin = cfg_.perception_nav_ray_clear_margin;
    dcfg.miss_clear_frames = cfg_.perception_nav_miss_clear_frames;
    dcfg.decay_in_fov_sec = cfg_.perception_nav_decay_in_fov_sec;
    dcfg.decay_blind_sec = cfg_.perception_nav_decay_blind_sec;
    dcfg.confirm_min_frames = cfg_.perception_nav_confirm_min_frames;
    dcfg.fast_confirm_with_neighbor = cfg_.perception_nav_fast_confirm_with_neighbor;
    dcfg.tilt_jump_deg = cfg_.perception_nav_tilt_jump_deg;
    dcfg.tilt_hold_frames = cfg_.perception_nav_tilt_hold_frames;
    dcfg.drift_max_abs = cfg_.perception_nav_drift_max_abs;
    dcfg.drift_min_samples = cfg_.perception_nav_drift_min_samples;
    dynamic_stvl_.setConfig(dcfg);

    scratch_pts_.reserve(20000);

    logger::MaMap->info(
        "[LocalPerceptionPipeline] backend='{}', sensor t_bl=[{:.3f},{:.3f},{:.3f}] rpy_deg=[{:.1f},{:.1f},{:.1f}]",
        cfg_.perception_backend,
        scfg.t_base_lidar.x(),
        scfg.t_base_lidar.y(),
        scfg.t_base_lidar.z(),
        scfg.rpy_deg.x(),
        scfg.rpy_deg.y(),
        scfg.rpy_deg.z()
    );

    last_empty_warn_t_ = clock_.nowSec();
}

auto LocalPerceptionPipeline::setClock(utils::Clock clock) -> void {
    clock_ = clock;
    // 构造期默认 steady_clock 的告警基线在切换到仿真/手动时钟域后失效，重新取基线。
    last_empty_warn_t_ = clock_.nowSec();
    dynamic_stvl_.setClock(clock);
}

auto LocalPerceptionPipeline::setMode(MapMode mode) -> void {
    const uint8_t next = static_cast<uint8_t>(mode);
    const uint8_t prev = mode_.exchange(next, std::memory_order_acq_rel);
    setForgetThresholdForMode(mode == MapMode::MAPPING);
    if (prev != next) {
        resetDynamic();
    }
}

auto LocalPerceptionPipeline::resetDynamic() -> void {
    dynamic_stvl_.reset();
    std::lock_guard<std::mutex> lock(viz_mtx_);
    if (!last_dynamic_3d_.empty()) {
        last_dynamic_3d_.clear();
        dynamic_3d_rev_.fetch_add(1, std::memory_order_release);
    }
}

auto LocalPerceptionPipeline::setVizCapture(bool filtered_cloud, bool dynamic_3d) -> void {
    if (!use_stvl_prior_) {
        return;
    }
    const uint8_t next =
        static_cast<uint8_t>((filtered_cloud ? kVizCaptureFiltered : 0U) | (dynamic_3d ? kVizCaptureDynamic3d : 0U));
    if (viz_capture_mask_.load(std::memory_order_relaxed) == next) {
        return;
    }
    const uint8_t prev = viz_capture_mask_.exchange(next, std::memory_order_acq_rel);
    const uint8_t disabled = static_cast<uint8_t>(prev & static_cast<uint8_t>(~next));
    if (disabled == 0U) {
        return;
    }
    std::lock_guard<std::mutex> lock(viz_mtx_);
    if ((disabled & kVizCaptureFiltered) != 0U && !last_filtered_cloud_.empty()) {
        last_filtered_cloud_.clear();
        filtered_cloud_rev_.fetch_add(1, std::memory_order_release);
    }
    if ((disabled & kVizCaptureDynamic3d) != 0U && !last_dynamic_3d_.empty()) {
        last_dynamic_3d_.clear();
        dynamic_3d_rev_.fetch_add(1, std::memory_order_release);
    }
}

auto LocalPerceptionPipeline::setForgetThresholdForMode(bool mapping_mode) -> void {
    // 导航模式：动态障碍离开后 nav_forget_threshold 秒遗忘；建图模式按 forget_threshold（默认关）
    const double thresh = mapping_mode ? cfg_.forget_threshold : cfg_.nav_forget_threshold;
    rog_map_ptr_->setForgetThreshold(thresh);
}

auto LocalPerceptionPipeline::updateOdom(const rog_map::Pose& pose, double stamp_sec) -> void {
    rog_map_ptr_->updateRobotState(pose);
    odom_stamp_is_event_ = std::isfinite(stamp_sec) && stamp_sec > 0.0;
    odom_stamp_sec_ = odom_stamp_is_event_ ? stamp_sec : 0.0;
    odom_rx_sec_ = clock_.nowSec();
}

auto LocalPerceptionPipeline::updateCloud(const rog_map::PointCloud& cloud, double stamp_sec) -> void {
    if (!rog_map_ptr_->robot_state_.rcv) {
        logger::MaMap->warn("No odom received, skip cloud callback.");
        return;
    }

    // 时钟域安全：事件时间（ROS header.stamp，真机为 Unix epoch）只与事件时间比较；
    // 本地时钟（steady / 仿真 /clock）只与接收时刻比较，绝不相减。
    const double now_sec = clock_.nowSec();
    const bool cloud_is_event = std::isfinite(stamp_sec) && stamp_sec > 0.0;
    const double age = (cloud_is_event && odom_stamp_is_event_) ? stamp_sec - odom_stamp_sec_ : now_sec - odom_rx_sec_;
    // age > timeout：odom 过期；age < -timeout：仿真重置/大范围乱序，同样按过期丢弃，
    // 避免把负年龄当成“永远新鲜”而持续使用错误姿态。
    if (!std::isfinite(age) || age > cfg_.odom_timeout || age < -cfg_.odom_timeout) {
        logger::MaMap->warn("Odom timeout, skip cloud callback.");
        return;
    }

    std::lock_guard<std::mutex> lock(input_mtx_);
    cloud_ = cloud;
    pc_pose_ = std::make_pair(rog_map_ptr_->robot_state_.p, rog_map_ptr_->robot_state_.q);
    pending_cloud_stamp_sec_ = cloud_is_event ? stamp_sec : now_sec;
    ++unfinished_frame_cnt_;
    map_empty_ = false;
}

auto LocalPerceptionPipeline::processPendingFrame(const StaticNavContext& static_ctx
) -> std::optional<LocalTerrainFrame> {
    int pending = 0;
    bool empty = true;
    double pending_stamp_sec = 0.0;
    {
        std::lock_guard<std::mutex> lock(input_mtx_);
        empty = map_empty_;
        pending = unfinished_frame_cnt_;
        if (!empty && pending > 0) {
            std::swap(scratch_pc_, cloud_);
            scratch_pose_ = pc_pose_;
            pending_stamp_sec = pending_cloud_stamp_sec_;
            unfinished_frame_cnt_ = 0;
        }
    }

    if (empty) {
        const double cur_t = clock_.nowSec();
        if (cfg_.ros_callback_en && (cur_t - last_empty_warn_t_ > 1.0)) {
            logger::MaMap->warn("No point cloud input, check the topic name.");
            last_empty_warn_t_ = cur_t;
        }
        return std::nullopt;
    }

    if (pending == 0) {
        return std::nullopt;
    }

    if (pending > 1) {
        logger::MaMap->warn("Unfinished frame cnt > 1, the map may not work in real-time");
    }

    if (use_stvl_prior_) {
        scratch_pts_.clear();
        if (scratch_pts_.capacity() < scratch_pc_.size()) {
            scratch_pts_.reserve(scratch_pc_.size());
        }
        const int stride = std::max(1, cfg_.point_filt_num);
        const bool filter_intensity = (cfg_.intensity_thresh > 0);
        for (size_t i = 0; i < scratch_pc_.size(); i += static_cast<size_t>(stride)) {
            const auto& pt = scratch_pc_.points[i];
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            if (filter_intensity && pt.intensity < static_cast<float>(cfg_.intensity_thresh)) {
                continue;
            }
            scratch_pts_.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y), static_cast<double>(pt.z));
        }

        const SensorPose sp = sensor_model_.computePose(scratch_pose_.first, scratch_pose_.second);
        const MapMode cur_mode = static_cast<MapMode>(mode_.load(std::memory_order_acquire));
        const uint8_t viz_mask = viz_capture_mask_.load(std::memory_order_acquire);
        const bool want_filtered = (viz_mask & kVizCaptureFiltered) != 0U;
        const bool want_dynamic_3d = (viz_mask & kVizCaptureDynamic3d) != 0U;

        std::optional<LocalTerrainFrame> frame;
        if (cur_mode == MapMode::MAPPING) {
            frame = online_mapper_.integrateFrame(
                scratch_pts_,
                sp,
                scratch_pose_.second,
                sensor_model_,
                /*build_local_preview=*/true,
                /*preview_cloud=*/nullptr,
                want_filtered ? &scratch_filtered_cloud_ : nullptr
            );
        } else {
            frame = dynamic_stvl_.update(
                scratch_pts_,
                sp,
                scratch_pose_.second,
                sensor_model_,
                static_ctx,
                pending_stamp_sec,
                want_dynamic_3d ? &scratch_dynamic_3d_ : nullptr,
                want_filtered ? &scratch_filtered_cloud_ : nullptr
            );
        }

        if (frame.has_value()) {
            {
                std::lock_guard<std::mutex> lock(terrain_mtx_);
                last_terrain_map_ = frame->obstacle_cells;
            }
            if (want_filtered || (cur_mode == MapMode::NAVIGATION && want_dynamic_3d)) {
                std::lock_guard<std::mutex> lock(viz_mtx_);
                const uint8_t cur_mask = viz_capture_mask_.load(std::memory_order_acquire);
                if (want_filtered && (cur_mask & kVizCaptureFiltered) != 0U
                    && (!scratch_filtered_cloud_.empty() || !last_filtered_cloud_.empty()))
                {
                    std::swap(last_filtered_cloud_, scratch_filtered_cloud_);
                    filtered_cloud_rev_.fetch_add(1, std::memory_order_release);
                }
                if (cur_mode == MapMode::NAVIGATION && want_dynamic_3d && (cur_mask & kVizCaptureDynamic3d) != 0U
                    && (!scratch_dynamic_3d_.empty() || !last_dynamic_3d_.empty()))
                {
                    std::swap(last_dynamic_3d_, scratch_dynamic_3d_);
                    dynamic_3d_rev_.fetch_add(1, std::memory_order_release);
                }
            }
        }
        return frame;
    }

    // ---------- 回退后端：legacy_rog ----------
    rog_map_ptr_->updateProbMap(scratch_pc_, scratch_pose_);
    const auto voxel_query = [this](const utils::Vec3f& pos) { return rog_map_ptr_->getGridType(pos); };
    if (cfg_.ground_layer_en) {
        // 用原始点云更新持久地面层（ROG 更新之后，悬空判据要用本帧射线结果）
        const utils::Vec3f sensor = scratch_pose_.first + scratch_pose_.second * cfg_.sensor_offset;
        ground_layer_.update(scratch_pc_, scratch_pose_, sensor, voxel_query);
    }

    const utils::Vec3f box_max = rog_map_ptr_->robot_state_.p + cfg_.visualization_range / 2.0;
    const utils::Vec3f box_min = rog_map_ptr_->robot_state_.p - cfg_.visualization_range / 2.0;
    scratch_real_occ_.clear();
    rog_map_ptr_->boxSearch(box_min, box_max, utils::OCCUPIED, scratch_real_occ_);

    LocalTerrainFrame frame;
    frame.obstacle_cells =
        terrain_analyzer_.analyze(rog_map_ptr_->robot_state_, scratch_real_occ_, frame.free_cells, voxel_query);
    frame.robot_pos = Eigen::Vector2d(scratch_pose_.first.x(), scratch_pose_.first.y());
    frame.has_robot_pos = true;
    frame.terrain_resolution = cfg_.resolution;
    frame.robot_footprint_radius =
        cfg_.ground_layer_en ? static_cast<double>(cfg_.ground_layer_footprint_radius) : 0.20;

    {
        std::lock_guard<std::mutex> lock(terrain_mtx_);
        last_terrain_map_ = frame.obstacle_cells;
    }

    return frame;
}

auto LocalPerceptionPipeline::getLastTerrainMap() const -> utils::vec_E<utils::Vec3f> {
    std::lock_guard<std::mutex> lock(terrain_mtx_);
    return last_terrain_map_;
}

auto LocalPerceptionPipeline::copyFilteredCloud(std::vector<Eigen::Vector3f>& out) const -> uint64_t {
    std::lock_guard<std::mutex> lock(viz_mtx_);
    out = last_filtered_cloud_;
    return filtered_cloud_rev_.load(std::memory_order_acquire);
}

auto LocalPerceptionPipeline::copyDynamic3dCloud(std::vector<Eigen::Vector3f>& out) const -> uint64_t {
    std::lock_guard<std::mutex> lock(viz_mtx_);
    out = last_dynamic_3d_;
    return dynamic_3d_rev_.load(std::memory_order_acquire);
}

auto LocalPerceptionPipeline::extractStatic3dCloud(
    const Eigen::Vector3d& box_min,
    const Eigen::Vector3d& box_max,
    std::vector<Eigen::Vector3f>& out,
    size_t max_points
) const -> uint64_t {
    if (!use_stvl_prior_) {
        out.clear();
        return 0;
    }
    return online_mapper_.extractStaticVoxelCloud(box_min, box_max, out, max_points);
}

} // namespace ma_map
