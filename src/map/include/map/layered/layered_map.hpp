#pragma once

#include "map/core/grid_geometry.hpp"
#include "map/core/grid_map.hpp"
#include "map/core/map_io.hpp"
#include "map/layered/map_layers.hpp"
#include "map/layered/snapshot_pool.hpp"
#include "map/perception/local_terrain_frame.hpp"
#include "utils/expected.hpp"

#include <Eigen/Core>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ma_map {

struct LoadReport {
    grid_map::GridGeometry geometry;
    int occupied_count = 0;
    int free_count = 0;
    int unknown_count = 0;
    bool has_terrain_prior = false;
    std::string terrain_prior_path;
};

// 导航模式感知所需的只读静态上下文快照：
// - static_esdf      : 仅由 StaticMapLayer 计算的纯静态 ESDF（绝不被动态障碍污染），未加载静态图时为 nullptr。
// - terrain_prior    : 可选 2.5D 静态地面/悬空先验（.terrain.bin），未加载或缺失时为 nullptr。
// - current_snapshot : 当前已发布的 2D 地图快照（用于无锁查询 TUNNEL 等语义层）。
struct StaticNavContext {
    grid_map::GridMap::ConstPtr static_esdf;
    std::shared_ptr<const StaticTerrainPrior> terrain_prior;
    grid_map::GridMap::ConstPtr current_snapshot;
};

// Layer 3：分层 2D 全局地图协调器（与 3D 感知/PCL/ROS 完全解耦）。
// 并发契约：
// - getSnapshot() 仅持有轻量 front_mtx_，O(1) 返回不可变 GridMap::ConstPtr，不阻塞或被写路径 ESDF 计算阻塞。
// - 写操作（updateFromPerception / setMode / ensureCovers / loadStaticMap / setGlobalOccupancy / setSemanticsPolygon）
//   由 back_mtx_ 串行化，构建完成后再原子替换前端快照。
class LayeredMap {
public:
    struct Config {
        double x_left = 6.0;
        double x_right = 22.0;
        double y_left = 10.0;
        double y_right = 10.0;
        double resolution = 0.05;
        double expand_chunk_m = 4.0;
        double max_map_size_x = 80.0;
        double max_map_size_y = 80.0;
        int save_margin_cells = 2;
        bool unknown_as_occupied = true;
    };

    explicit LayeredMap(const Config& config);

    auto setMode(MapMode mode) -> void;
    auto mode() const -> MapMode;

    // 应用一帧局部地形输出：
    // - MAPPING 模式：按需分块扩图（受 max_map_size 约束）→ 更新 PersistentMapLayer → 合成并按需发布快照。
    // - NAVIGATION 模式：不扩图 → 重建 DynamicObstacleLayer（严格裁剪到固定边界）→ 合成 Static|Dynamic 并按需发布快照。
    auto updateFromPerception(const LocalTerrainFrame& frame) -> void;

    // 确认地图覆盖世界坐标点 world_pt：
    // - NAVIGATION 模式：只读检查 geometry_.isInside(world_pt)，绝不扩图。
    // - MAPPING 模式：若越界则尝试按块扩图覆盖；若超出 max_map_size 上限则返回 false 且不修改原地图。
    auto ensureCovers(const Eigen::Vector2d& world_pt) -> bool;

    // 从 .pgm / .yaml 加载先验静态地图，严格继承文件内的 resolution、origin 与尺寸并立即发布快照。
    // 失败时原地图与旧快照均保持不变。
    auto loadStaticMap(std::string_view map_path) -> tl::expected<LoadReport, std::string>;

    // 保存已探索三态地图（建图默认裁剪到已探索包围盒 + save_margin_cells，不写多余空白外框）。
    // 磁盘写操作在锁外执行，不阻塞并发读取或后续感知更新。
    auto saveExploredMap(std::string_view output_path, const std::optional<SaveOptions>& options = std::nullopt) const
        -> tl::expected<SaveReport, std::string>;

    // 兼容旧接口：向当前几何尺寸的静态层写入 0/1 占据矩阵并发布快照。
    auto setGlobalOccupancy(const grid_map::RowMatrixXi& global_occ) -> bool;

    // 更新世界系语义多边形并发布新快照（复用现有占据与 ESDF，不重算 ESDF）。
    auto
    setSemanticsPolygon(const std::vector<Eigen::Vector2d>& polygon, grid_map::GridMap::Semantics semantics) -> bool;

    auto getSnapshot() const -> grid_map::GridMap::ConstPtr;
    auto getStaticNavContext() const -> StaticNavContext;
    auto defaultSaveOptions() const -> SaveOptions;
    auto getOccupancyMatrix() const -> grid_map::RowMatrixXi;
    auto getStaticOccupancyMatrix() const -> grid_map::RowMatrixXi;

private:
    auto
    expandToCoverBoxLocked(const Eigen::Vector2d& req_min, const Eigen::Vector2d& req_max, bool strict_cover) -> bool;
    auto expandByCellsLocked(int dx_neg, int dx_pos, int dy_neg, int dy_pos) -> void;
    auto composeOccupancyLocked() -> void;
    auto publishComposedLocked(bool force_publish) -> void;
    auto publishSnapshot(grid_map::GridMap::ConstPtr next_front) -> void;
    auto rebuildStaticEsdfLocked() const -> void;

    Config config_;
    MapMode mode_ = MapMode::MAPPING;
    grid_map::OutOfBoundsPolicy oob_policy_ = grid_map::OutOfBoundsPolicy::FREE;
    grid_map::GridGeometry geometry_;
    bool max_size_warned_ = false;

    StaticMapLayer static_layer_;
    PersistentMapLayer persistent_layer_;
    DynamicObstacleLayer dynamic_layer_;
    SemanticsLayer semantics_layer_;

    std::vector<uint8_t> composed_occ_;
    mutable grid_map::GridMap::EsdfWorkspace esdf_workspace_;
    std::shared_ptr<SnapshotPool> snapshot_pool_;
    grid_map::GridMap::ConstPtr front_snapshot_;
    mutable grid_map::GridMap::ConstPtr front_static_esdf_;
    mutable std::shared_ptr<const StaticTerrainPrior> front_terrain_prior_;
    mutable std::atomic<bool> static_esdf_dirty_ {false};

    mutable std::mutex back_mtx_;
    mutable std::mutex front_mtx_;
};

} // namespace ma_map
