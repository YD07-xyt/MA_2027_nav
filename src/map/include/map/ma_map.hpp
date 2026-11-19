#pragma once
#include "3d_occ_map/rog_map.h"
#include "grid_map.hpp"
#include "map/3d_occ_map/config.hpp"
#include "map/terrain_analysis.hpp"
#include "utils/logger.hpp"
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>
namespace ma_map {
class MaMap {
public:
    explicit MaMap(const std::string& cfg_path) {
        rog_map_ptr_ = std::make_shared<rog_map::ROGMap>();

        cfg_ = rog_map::Config(cfg_path);
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
        terrain_cfg.resolution = cfg_.resolution;
        terrain_cfg.map_size_x = cfg_.local_update_box_d.x();
        terrain_cfg.map_size_y = cfg_.local_update_box_d.y();
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
        snapshot_pool_ = std::make_shared<BufferPool>(
            cfg_.grid_map_x_left,
            cfg_.grid_map_x_right,
            cfg_.grid_map_y_left,
            cfg_.grid_map_y_right,
            cfg_.grid_map_resolution
        );
        auto initial_slot = snapshot_pool_->acquire();
        grid_resolution_ = initial_slot.map->getResolution();
        grid_resolution_inv_ = 1.0 / grid_resolution_;
        grid_origin_ = initial_slot.map->getOrigin();
        grid_voxel_num_ = initial_slot.map->getVoxelNum();

        const auto voxel_num = grid_voxel_num_;

        occupancy_2d = grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
        global_2d_occ_ = grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
        dynamic_2d_occ_ = grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
        free_streak_ = grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
        free_last_frame_ = grid_map::RowMatrixXi::Constant(voxel_num.x(), voxel_num.y(), -10);
        semantics_2d_.assign(
            static_cast<size_t>(voxel_num.x()) * static_cast<size_t>(voxel_num.y()),
            static_cast<uint8_t>(grid_map::GridMap::Semantics::NONE)
        );
        initial_slot.semantics_version = semantics_version_;
        front_grid_map_ = snapshot_pool_->make_snapshot(std::move(initial_slot));
    }
    auto set_global_map(grid_map::RowMatrixXi global_map) -> void {
        std::lock_guard<std::mutex> lock(back_mtx_);
        if (global_map.rows() != grid_voxel_num_.x() || global_map.cols() != grid_voxel_num_.y()) {
            logger::MaMap->error(
                "set_global_map size mismatch: got {}x{}, expected {}x{}",
                global_map.rows(),
                global_map.cols(),
                grid_voxel_num_.x(),
                grid_voxel_num_.y()
            );
            return;
        }
        global_2d_occ_ = std::move(global_map);
        occupancy_2d = dynamic_2d_occ_.cwiseMax(global_2d_occ_);
        publish_occupancy_locked(occupancy_2d);
    }
    // 语义标记写到持久语义层并发布新快照（复用当前占据与 ESDF，避免被下一帧覆盖且不重算 ESDF）
    auto set_semantics_polygon(const std::vector<Eigen::Vector2d>& polygon, grid_map::GridMap::Semantics semantics)
        -> void {
        std::lock_guard<std::mutex> lock(back_mtx_);
        auto slot = snapshot_pool_->acquire();
        if (slot.semantics_version != semantics_version_) {
            slot.map->copySemanticsFrom(semantics_2d_);
            slot.semantics_version = semantics_version_;
        }
        if (!slot.map->semantics_polygon_region(polygon, semantics)) {
            snapshot_pool_->release(std::move(slot));
            return;
        }
        const auto updated_sem = slot.map->semanticsData();
        semantics_2d_.assign(updated_sem.begin(), updated_sem.end());
        ++semantics_version_;
        slot.semantics_version = semantics_version_;

        auto current_front = get_grid_map();
        if (!current_front || !slot.map->copyOccAndEsdfFrom(*current_front)) {
            slot.map->overwriteMapAndUpdateEsdf(occupancy_2d, &esdf_workspace_);
        }
        current_front.reset();

        auto next_front = snapshot_pool_->make_snapshot(std::move(slot));
        publish_snapshot(std::move(next_front));
    }
    auto set_mapping_model(bool mapping_model) -> void {
        mapping_model_ = mapping_model;
        // 导航模式：动态障碍离开后 nav_forget_threshold 秒遗忘；建图模式按 forget_threshold（默认关）
        rog_map_ptr_->setForgetThreshold(mapping_model ? cfg_.forget_threshold : cfg_.nav_forget_threshold);
    }
    auto update_map() -> void {
        if (map_empty_) {
            static double last_print_t =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            double cur_t =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
                logger::MaMap->warn("No point cloud input, check the topic name.");
                last_print_t = cur_t;
            }
            return;
        }

        if (unfinished_frame_cnt == 0) {
            return;
        }

        if (unfinished_frame_cnt > 1) {
            logger::MaMap->warn("Unfinished frame cnt > 1, the map may not work in real-time");
        }
        static rog_map::PointCloud temp_pc;
        static rog_map::Pose temp_pose;

        updete_lock.lock();
        temp_pc = cloud_;
        temp_pose = pc_pose_;
        unfinished_frame_cnt = 0;
        updete_lock.unlock();

        rog_map_ptr_->updateProbMap(temp_pc, temp_pose);
        if (cfg_.ground_layer_en) {
            // 用原始点云更新持久地面层（ROG 更新之后，悬空判据要用本帧射线结果）
            const utils::Vec3f sensor = temp_pose.first + temp_pose.second * cfg_.sensor_offset;
            ground_layer_.update(temp_pc, temp_pose, sensor, [this](const utils::Vec3f& pos) {
                return rog_map_ptr_->getGridType(pos);
            });
        }
        update_terrain();
        {
            std::lock_guard<std::mutex> lock(back_mtx_);
            if (!mapping_model_) {
                // 导航模式：动态层每帧重建，静态信息来自先验 global_2d_occ_
                dynamic_2d_occ_.setZero();
            }

            // 障碍格：立即写 1，并清空该格的空闲计数
            for_each_terrain_cell(terrain_map_, [this](int x, int y) {
                dynamic_2d_occ_(x, y) = 1;
                free_streak_(x, y) = 0;
            });
            if (mapping_model_) {
                // 建图模式：dynamic_2d_occ_ 是持久地图，窗口内以 3D 当前结果为准回写，
                // 窗口外/无观测格保持原值。空闲证据需连续 kFreeFramesToClear 帧才写 0，
                // 防止细杆/网格单帧漏检被清掉。
                ++frame_id_;
                for_each_terrain_cell(terrain_free_, [this](int x, int y) {
                    if (dynamic_2d_occ_(x, y) == 0) {
                        return;
                    }
                    // 上一帧也是空闲证据才累加，中间漏观测则重新计数
                    free_streak_(x, y) = (free_last_frame_(x, y) == frame_id_ - 1) ? free_streak_(x, y) + 1 : 1;
                    free_last_frame_(x, y) = frame_id_;
                    if (free_streak_(x, y) >= kFreeFramesToClear) {
                        dynamic_2d_occ_(x, y) = 0;
                        free_streak_(x, y) = 0;
                    }
                });
            }
            occupancy_2d = dynamic_2d_occ_.cwiseMax(global_2d_occ_);
            // grid_map_ptr_->inflate(0.02);
            publish_occupancy_locked(occupancy_2d);
        }
    };
    auto update_odom(const rog_map::Pose& pose) -> void {
        rog_map_ptr_->updateRobotState(pose);
    }
    auto update_cloud(const rog_map::PointCloud& cloud) -> void {
        if (!rog_map_ptr_->robot_state_.rcv) {
            logger::MaMap->warn("No odom received, skip cloud callback.");
            return;
        }
        double cbk_t =
            std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        if (cbk_t - rog_map_ptr_->robot_state_.rcv_time > cfg_.odom_timeout) {
            logger::MaMap->warn("Odom timeout, skip cloud callback.");
            return;
        }
        updete_lock.lock();
        cloud_ = cloud;
        pc_pose_ = std::make_pair(rog_map_ptr_->robot_state_.p, rog_map_ptr_->robot_state_.q);
        unfinished_frame_cnt++;
        map_empty_ = false;
        updete_lock.unlock();
    }
public:
    auto get_terrain_map() -> utils::vec_E<utils::Vec3f> {
        return terrain_map_;
    }
    // 返回最近一次发布的只读快照。调用方持有该 ConstPtr 期间内容不可变；
    // 最后一个持有者释放后，底层 GridMap 自动归还对象池复用（不阻塞写线程）。
    auto get_grid_map() const -> grid_map::GridMap::ConstPtr {
        std::lock_guard<std::mutex> lock(front_mtx_);
        return front_grid_map_;
    }
    auto get_occupancy_2d() const -> grid_map::RowMatrixXi {
        std::lock_guard<std::mutex> lock(back_mtx_);
        return occupancy_2d;
    }
    // 持久地面层（只读，地图线程外读取需自行保证不与 update_map 并发）
    auto get_ground_layer() const -> const Terrain::GroundLayer& {
        return ground_layer_;
    }
    auto get_rog_map() -> std::shared_ptr<rog_map::ROGMap> {
        return rog_map_ptr_;
    }
    auto get_config()->rog_map::Config{
        return cfg_;
    }
    auto get_global_occ() const -> grid_map::RowMatrixXi {
        std::lock_guard<std::mutex> lock(back_mtx_);
        return global_2d_occ_;
    }
private:
    // 池化不可变快照对象池：默认预热 2 槽，上限 4 槽（满载约 9 MB）。
    // 读线程持有旧快照时不阻塞发布；池空时非阻塞溢出分配，归还时超过上限直接释放。
    class BufferPool: public std::enable_shared_from_this<BufferPool> {
    public:
        static constexpr size_t kPrewarmSlots = 2;
        static constexpr size_t kMaxSlots = 4;

        struct Slot {
            std::unique_ptr<grid_map::GridMap> map;
            uint64_t semantics_version = 0;
        };

        BufferPool(double x_left, double x_right, double y_left, double y_right, double resolution):
            x_left_(x_left),
            x_right_(x_right),
            y_left_(y_left),
            y_right_(y_right),
            resolution_(resolution) {
            free_slots_.reserve(kMaxSlots);
            for (size_t i = 0; i < kPrewarmSlots; ++i) {
                free_slots_.push_back(Slot {create_grid_map(), 0});
            }
        }

        auto acquire() -> Slot {
            {
                std::lock_guard<std::mutex> lock(mtx_);
                if (!free_slots_.empty()) {
                    Slot slot = std::move(free_slots_.back());
                    free_slots_.pop_back();
                    return slot;
                }
            }
            return Slot {create_grid_map(), 0};
        }

        auto release(Slot slot) -> void {
            if (!slot.map) return;
            std::unique_ptr<grid_map::GridMap> overflow;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                if (free_slots_.size() < kMaxSlots) {
                    free_slots_.push_back(std::move(slot));
                    return;
                }
                overflow = std::move(slot.map);
            }
        }

        auto make_snapshot(Slot slot) -> grid_map::GridMap::ConstPtr {
            grid_map::GridMap* raw = slot.map.release();
            return grid_map::GridMap::ConstPtr(raw, RecycleDeleter {weak_from_this(), slot.semantics_version});
        }

    private:
        struct RecycleDeleter {
            std::weak_ptr<BufferPool> pool;
            uint64_t semantics_version = 0;

            void operator()(grid_map::GridMap* raw) const {
                std::unique_ptr<grid_map::GridMap> ptr(raw);
                if (!ptr) return;
                if (auto p = pool.lock()) {
                    p->release(Slot {std::move(ptr), semantics_version});
                }
            }
        };

        auto create_grid_map() const -> std::unique_ptr<grid_map::GridMap> {
            auto map = std::make_unique<grid_map::GridMap>();
            map->init(x_left_, x_right_, y_left_, y_right_, resolution_);
            return map;
        }

        double x_left_ = 0.0;
        double x_right_ = 0.0;
        double y_left_ = 0.0;
        double y_right_ = 0.0;
        double resolution_ = 0.0;
        std::mutex mtx_;
        std::vector<Slot> free_slots_;
    };

    auto pos_to_index(const Eigen::Vector2d& pos, Eigen::Vector2i& id) const -> void {
        id << static_cast<int>((pos.x() - grid_origin_.x()) * grid_resolution_inv_),
            static_cast<int>((pos.y() - grid_origin_.y()) * grid_resolution_inv_);
    }

    // 须在持有 back_mtx_ 时调用。
    auto publish_occupancy_locked(const grid_map::RowMatrixXi& occ) -> void {
        auto current_front = get_grid_map();
        if (current_front && current_front->equalsOccupancy(occ)) {
            return;
        }
        current_front.reset();

        auto slot = snapshot_pool_->acquire();
        if (!slot.map->overwriteMapAndUpdateEsdf(occ, &esdf_workspace_)) {
            snapshot_pool_->release(std::move(slot));
            return;
        }
        if (slot.semantics_version != semantics_version_) {
            slot.map->copySemanticsFrom(semantics_2d_);
            slot.semantics_version = semantics_version_;
        }
        auto next_front = snapshot_pool_->make_snapshot(std::move(slot));
        publish_snapshot(std::move(next_front));
    }

    auto publish_snapshot(grid_map::GridMap::ConstPtr next_front) -> void {
        grid_map::GridMap::ConstPtr old_front;
        {
            std::lock_guard<std::mutex> lock(front_mtx_);
            old_front = std::move(front_grid_map_);
            front_grid_map_ = std::move(next_front);
        }
    }

    bool mapping_model_ = true; // true: 建图模式，false: 规划模式
    utils::vec_E<utils::Vec3f> terrain_map_;
    grid_map::RowMatrixXi global_2d_occ_;     // 永久层（可 set_global_map 加载）
    // 导航模式：动态层（每帧重建）；建图模式：持久地图（窗口内按 3D 回写，窗口外保持）
    grid_map::RowMatrixXi dynamic_2d_occ_;
    grid_map::RowMatrixXi occupancy_2d;
    utils::vec_E<utils::Vec3f> terrain_free_;   // 本帧空闲证据（terrain 格中心）
    grid_map::RowMatrixXi free_streak_;         // 建图模式：连续空闲证据帧数
    grid_map::RowMatrixXi free_last_frame_;     // 建图模式：最近一次空闲证据的帧号
    int frame_id_ {0};
    static constexpr int kFreeFramesToClear = 2; // 连续 2 帧空闲证据才写 0

    // 把 terrain 格中心列表按真实占地映射到 2D 细格：terrain 点是 terrain 格（边长 cfg_.resolution）
    // 的中心，正好落在 2D 细格边界上，直接 posToIndex 截断会随浮点误差左右跳一格。
    // 两端各向格内收 1/4 细格再取索引，避开边界截断。
    template<typename F>
    auto for_each_terrain_cell(const utils::vec_E<utils::Vec3f>& cells, F&& fn) -> void {
        const double terrain_half = 0.5 * cfg_.resolution;
        const double eps = 0.25 * grid_resolution_;
        const Eigen::Vector2i max_idx(dynamic_2d_occ_.rows() - 1, dynamic_2d_occ_.cols() - 1);
        for (const auto& c: cells) {
            Eigen::Vector2i lo, hi;
            pos_to_index(Eigen::Vector2d(c.x() - terrain_half + eps, c.y() - terrain_half + eps), lo);
            pos_to_index(Eigen::Vector2d(c.x() + terrain_half - eps, c.y() + terrain_half - eps), hi);
            lo = lo.cwiseMax(Eigen::Vector2i::Zero());
            hi = hi.cwiseMin(max_idx);
            for (int x = lo.x(); x <= hi.x(); ++x) {
                for (int y = lo.y(); y <= hi.y(); ++y) {
                    fn(x, y);
                }
            }
        }
    }
    
    auto update_terrain() -> void {
        utils::Vec3f box_max = rog_map_ptr_->robot_state_.p + cfg_.visualization_range / 2;
        utils::Vec3f box_min = rog_map_ptr_->robot_state_.p - cfg_.visualization_range / 2;
        utils::vec_E<utils::Vec3f> inf_occ_map, real_occ_map;
        //rog_map_ptr_->boxSearchInflate(box_min, box_max, utils::OCCUPIED, inf_occ_map);
        rog_map_ptr_->boxSearch(box_min, box_max, utils::OCCUPIED, real_occ_map);
        // 悬空判据需要 3D 空闲/未知证据：杆下方被射线扫空才放行（只读查询，地图线程内调用）
        const auto voxel_query = [this](const utils::Vec3f& pos) {
            return rog_map_ptr_->getGridType(pos);
        };
        terrain_map_ = terrain_analyzer_.analyze(
            rog_map_ptr_->robot_state_, real_occ_map, terrain_free_, voxel_query);
        // utils::vec_E<utils::Vec3f> output;
        // for (auto pt: terrain_map) {
        //     if (pt.z() - rog_map_ptr_->robot_state_.p.z() < 0.07) {
        //         //continue;
        //     }
        //     output.emplace_back(pt);
        // }
    }
    
private:
    rog_map::Pose pc_pose_;
    rog_map::PointCloud cloud_;
    double grid_resolution_ = 0.0;
    double grid_resolution_inv_ = 0.0;
    Eigen::Vector2d grid_origin_ = Eigen::Vector2d::Zero();
    Eigen::Vector2i grid_voxel_num_ = Eigen::Vector2i::Zero();
    std::vector<uint8_t> semantics_2d_;
    uint64_t semantics_version_ = 0;
    grid_map::GridMap::EsdfWorkspace esdf_workspace_;
    std::shared_ptr<BufferPool> snapshot_pool_;
    grid_map::GridMap::ConstPtr front_grid_map_;
    mutable std::mutex back_mtx_;
    mutable std::mutex front_mtx_;
    std::shared_ptr<rog_map::ROGMap> rog_map_ptr_;
    Terrain::TerrainAnalyzer terrain_analyzer_;
    Terrain::GroundLayer ground_layer_;
    rog_map::Config cfg_;
    bool map_empty_ = true;
    int unfinished_frame_cnt {0};
    rog_map::mutex updete_lock;
};
}