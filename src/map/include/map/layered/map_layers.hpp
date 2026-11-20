#pragma once

#include "map/core/grid_geometry.hpp"
#include "map/core/grid_map.hpp"
#include "map/core/map_io.hpp"
#include "map/perception/local_terrain_frame.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace ma_map {

// 地图运行模式：
// - MAPPING    : 建图模式（允许按块动态扩图、越界 FREE、保存仅导出已探索三态图）
// - NAVIGATION : 导航模式（固定静态图边界、越界 OCCUPIED 虚拟墙、动态障碍仅在图内生效）
enum class MapMode : uint8_t {
    MAPPING = 0,
    NAVIGATION = 1,
};

// Nav2 风格 2D 全局地图分层基类：每层维护独立缓冲与生命周期。
class MapLayer {
public:
    virtual ~MapLayer() = default;

    virtual auto name() const -> std::string_view = 0;

    // 地图几何变更时同步各层缓冲：
    // - preserve_data=true  : 按 shift（新索引 = 旧索引 + shift）平移保留旧格数据，新暴露区域按各层语义填充。
    // - preserve_data=false : 按 new_geom 清空重建（例如加载新静态图）。
    virtual auto
    resize(const grid_map::GridGeometry& new_geom, const Eigen::Vector2i& shift, bool preserve_data) -> void = 0;

    virtual auto reset() -> void = 0;
};

// 先验静态地图层：持有从 PGM/YAML 加载或外部注入的静态三态栅格与 0/1 占据。
class StaticMapLayer final: public MapLayer {
public:
    auto name() const -> std::string_view override {
        return "static_map";
    }

    auto
    resize(const grid_map::GridGeometry& new_geom, const Eigen::Vector2i& shift, bool preserve_data) -> void override;
    auto reset() -> void override;

    auto load(const LoadedMap& loaded) -> void;
    auto setOccupancyMatrix(const grid_map::RowMatrixXi& mat, bool* changed = nullptr) -> bool;

    auto geometry() const -> const grid_map::GridGeometry& {
        return geom_;
    }

    auto occupancy() const -> std::span<const uint8_t> {
        return occ_;
    }

    auto cells() const -> std::span<const CellState> {
        return cells_;
    }

    auto hasLoadedCells() const -> bool {
        return has_loaded_cells_;
    }

    auto toOccupancyMatrix() const -> grid_map::RowMatrixXi;

private:
    grid_map::GridGeometry geom_;
    std::vector<uint8_t> occ_;
    std::vector<CellState> cells_;
    bool has_loaded_cells_ = false;
    bool from_binary_matrix_ = false;
};

// 建图模式持久三态层：
// - UNKNOWN : 未观测（在线建图规划可通行，保存为 PGM 205）
// - FREE    : 已观测可通行（保存为 PGM 254）
// - OCCUPIED: 已观测障碍（保存为 PGM 0；需连续 kFreeFramesToClear 帧空闲证据才清除）
class PersistentMapLayer final: public MapLayer {
public:
    static constexpr int kFreeFramesToClear = 2;

    auto name() const -> std::string_view override {
        return "persistent_map";
    }

    auto
    resize(const grid_map::GridGeometry& new_geom, const Eigen::Vector2i& shift, bool preserve_data) -> void override;
    auto reset() -> void override;

    auto update(const LocalTerrainFrame& frame) -> void;

    auto occupancy() const -> std::span<const uint8_t> {
        return occ_;
    }

    auto cells() const -> std::span<const CellState> {
        return cells_;
    }

private:
    grid_map::GridGeometry geom_;
    std::vector<CellState> cells_;
    std::vector<uint8_t> occ_;
    std::vector<int> free_streak_;
    std::vector<int> free_last_frame_;
    int frame_id_ = 0;
};

// 导航模式动态障碍层：每帧清空并按当前局部感知结果重建，严格裁剪到固定静态图范围内。
class DynamicObstacleLayer final: public MapLayer {
public:
    auto name() const -> std::string_view override {
        return "dynamic_obstacle";
    }

    auto
    resize(const grid_map::GridGeometry& new_geom, const Eigen::Vector2i& shift, bool preserve_data) -> void override;
    auto reset() -> void override;

    auto update(const LocalTerrainFrame& frame) -> void;

    auto occupancy() const -> std::span<const uint8_t> {
        return occ_;
    }

private:
    grid_map::GridGeometry geom_;
    std::vector<uint8_t> occ_;
};

// 语义标记层（例如隧道区域）：保留世界系多边形列表，支持扩图/重载后自动重栅格化，并仅在 AABB 内增量光栅化。
class SemanticsLayer final: public MapLayer {
public:
    auto name() const -> std::string_view override {
        return "semantics";
    }

    auto
    resize(const grid_map::GridGeometry& new_geom, const Eigen::Vector2i& shift, bool preserve_data) -> void override;
    auto reset() -> void override;

    auto setPolygon(const std::vector<Eigen::Vector2d>& polygon, grid_map::GridMap::Semantics semantics) -> bool;

    auto data() const -> std::span<const uint8_t> {
        return buffer_;
    }

    auto version() const -> uint64_t {
        return version_;
    }

private:
    struct PolygonEntry {
        std::vector<Eigen::Vector2d> polygon;
        grid_map::GridMap::Semantics semantics = grid_map::GridMap::Semantics::NONE;
        Eigen::Vector2d min_p = Eigen::Vector2d::Zero();
        Eigen::Vector2d max_p = Eigen::Vector2d::Zero();
    };

    static auto
    computeAabb(const std::vector<Eigen::Vector2d>& polygon, Eigen::Vector2d& min_p, Eigen::Vector2d& max_p) -> void;
    static auto pointInPolygon(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& polygon) -> bool;
    static auto polygonsEqual(const std::vector<Eigen::Vector2d>& a, const std::vector<Eigen::Vector2d>& b) -> bool;

    auto rasterizeEntry(
        const std::vector<Eigen::Vector2d>& polygon,
        const Eigen::Vector2d& min_p,
        const Eigen::Vector2d& max_p,
        uint8_t value,
        const Eigen::Vector2i& clip_lo,
        const Eigen::Vector2i& clip_hi
    ) -> bool;

    grid_map::GridGeometry geom_;
    std::vector<uint8_t> buffer_;
    uint64_t version_ = 0;
    std::vector<PolygonEntry> polygons_;
};

} // namespace ma_map
