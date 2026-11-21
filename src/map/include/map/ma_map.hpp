#pragma once

#include "map/3d_occ_map/config.hpp"
#include "map/3d_occ_map/rog_map.h"
#include "map/core/grid_map.hpp"
#include "map/core/map_io.hpp"
#include "map/layered/layered_map.hpp"
#include "map/layered/map_layers.hpp"
#include "map/perception/ground_layer.hpp"
#include "utils/clock.hpp"
#include "utils/expected.hpp"

#include <Eigen/Core>

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ma_map {

// Layer 4：MaMap 薄门面（PImpl），组合 LocalPerceptionPipeline（3D 局部感知）与 LayeredMap（2D 分层全局地图）。
// 保持既有公共接口兼容，同时提供双模式语义（动态扩图建图 + 固定边界导航）、静态图精确加载与已探索区域保存。
class MaMap {
public:
    explicit MaMap(const std::string& cfg_path);
    ~MaMap();
    MaMap(MaMap&&) noexcept;
    MaMap& operator=(MaMap&&) noexcept;
    MaMap(const MaMap&) = delete;
    MaMap& operator=(const MaMap&) = delete;

    // 模式切换：MAPPING（建图，允许动态扩图，越界 FREE） / NAVIGATION（导航，固定边界，越界 OCCUPIED）。
    auto set_mode(MapMode mode) -> void;
    auto mode() const -> MapMode;
    auto set_mapping_model(bool mapping_model) -> void;

    // 注入时间源（ROS 仿真 /clock 或离线手动时钟）；默认使用 steady_clock。
    auto set_clock(utils::Clock clock) -> void;

    // 感知输入与更新。stamp_sec 为消息事件时间（s）；缺省 NaN 时按本地接收时间处理，
    // 保持既有单参数调用方完全兼容。
    auto update_odom(const rog_map::Pose& pose, double stamp_sec = std::numeric_limits<double>::quiet_NaN()) -> void;
    auto
    update_cloud(const rog_map::PointCloud& cloud, double stamp_sec = std::numeric_limits<double>::quiet_NaN()) -> void;
    auto update_map() -> void;

    // 确认地图覆盖世界坐标点 world_pt：
    // - MAPPING 模式若越界则尝试按块扩图（受 max_map_size 限制）；
    // - NAVIGATION 模式仅只读检查是否在固定静态图边界内，绝不扩图。
    auto ensure_covers(const Eigen::Vector2d& world_pt) -> bool;

    // 从 .pgm / .yaml 加载先验静态图，严格继承文件 resolution/origin/尺寸并立即发布快照。
    auto load_static_map(std::string_view map_path) -> tl::expected<LoadReport, std::string>;

    // 保存三态地图（建图默认裁剪到已探索包围盒 + save_margin_cells，不写入未探索空白外框）。
    auto save_explored_map(std::string_view output_path, const std::optional<SaveOptions>& options = std::nullopt) const
        -> tl::expected<SaveReport, std::string>;

    // 兼容既有接口：向当前几何尺寸的静态层写入 0/1 占据矩阵并发布快照。
    auto set_global_map(grid_map::RowMatrixXi global_map) -> void;

    // 语义标记写到持久语义层并发布新快照（复用当前占据与 ESDF，不重算 ESDF）。
    auto
    set_semantics_polygon(const std::vector<Eigen::Vector2d>& polygon, grid_map::GridMap::Semantics semantics) -> void;

    // 返回最近一次发布的只读快照。调用方持有该 ConstPtr 期间内容不可变；
    // 最后一个持有者释放后，底层 GridMap 自动归还对象池复用（不阻塞写线程）。
    auto get_grid_map() const -> grid_map::GridMap::ConstPtr;
    auto get_occupancy_2d() const -> grid_map::RowMatrixXi;
    auto get_global_occ() const -> grid_map::RowMatrixXi;
    auto get_terrain_map() const -> utils::vec_E<utils::Vec3f>;
    // 持久地面层（只读，地图线程外读取需自行保证不与 update_map 并发）
    auto get_ground_layer() const -> const Terrain::GroundLayer&;
    auto get_rog_map() const -> std::shared_ptr<rog_map::ROGMap>;
    auto get_config() const -> rog_map::Config;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ma_map
