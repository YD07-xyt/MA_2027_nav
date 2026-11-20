#include "local_perception_pipeline.hpp"

#include "utils/logger.hpp"

#include <chrono>
#include <utility>

namespace ma_map {

LocalPerceptionPipeline::LocalPerceptionPipeline(const rog_map::Config& cfg): cfg_(cfg) {
    rog_map_ptr_ = std::make_shared<rog_map::ROGMap>();
    rog_map_ptr_->setConfig(cfg_);
    rog_map_ptr_->init();

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

    last_empty_warn_t_ =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

auto LocalPerceptionPipeline::setForgetThresholdForMode(bool mapping_mode) -> void {
    // 导航模式：动态障碍离开后 nav_forget_threshold 秒遗忘；建图模式按 forget_threshold（默认关）
    const double thresh = mapping_mode ? cfg_.forget_threshold : cfg_.nav_forget_threshold;
    rog_map_ptr_->setForgetThreshold(thresh);
}

auto LocalPerceptionPipeline::updateOdom(const rog_map::Pose& pose) -> void {
    rog_map_ptr_->updateRobotState(pose);
}

auto LocalPerceptionPipeline::updateCloud(const rog_map::PointCloud& cloud) -> void {
    if (!rog_map_ptr_->robot_state_.rcv) {
        logger::MaMap->warn("No odom received, skip cloud callback.");
        return;
    }
    const double cbk_t =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    if (cbk_t - rog_map_ptr_->robot_state_.rcv_time > cfg_.odom_timeout) {
        logger::MaMap->warn("Odom timeout, skip cloud callback.");
        return;
    }

    std::lock_guard<std::mutex> lock(input_mtx_);
    cloud_ = cloud;
    pc_pose_ = std::make_pair(rog_map_ptr_->robot_state_.p, rog_map_ptr_->robot_state_.q);
    ++unfinished_frame_cnt_;
    map_empty_ = false;
}

auto LocalPerceptionPipeline::processPendingFrame() -> std::optional<LocalTerrainFrame> {
    int pending = 0;
    bool empty = true;
    {
        std::lock_guard<std::mutex> lock(input_mtx_);
        empty = map_empty_;
        pending = unfinished_frame_cnt_;
        if (!empty && pending > 0) {
            std::swap(scratch_pc_, cloud_);
            scratch_pose_ = pc_pose_;
            unfinished_frame_cnt_ = 0;
        }
    }

    if (empty) {
        const double cur_t =
            std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
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

} // namespace ma_map
