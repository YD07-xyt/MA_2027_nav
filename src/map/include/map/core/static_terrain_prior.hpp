#pragma once

#include "map/core/grid_geometry.hpp"
#include "utils/expected.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ma_map {

// 伴随静态 2D 地图（.pgm + .yaml）存储的 2.5D 静态地形先验（.terrain.bin）。
// 栅格几何与静态 2D 图严格逐格对齐（行主序 ix * ny + iy，世界坐标系，单位 m）：
// - ground_z       : 该格静态地面世界系高度 (m)；NaN 表示该格无已知地面（如墙体、未探索区）。
// - overhang_min_z : 该格已知静态悬空结构（限高杆/洞顶/桥底）的下沿世界系高度 (m)；+inf 表示无已知悬空顶。
// 线程契约：加载或构建完成后作为不可变值对象经 shared_ptr<const StaticTerrainPrior> 跨线程只读共享。
struct StaticTerrainPrior {
    static constexpr uint32_t kFormatVersion = 1;
    static constexpr char kMagic[4] = {'M', 'A', 'T', 'P'};

    grid_map::GridGeometry geometry;
    std::vector<float> ground_z;
    std::vector<float> overhang_min_z;

    static auto createEmpty(const grid_map::GridGeometry& geom) -> StaticTerrainPrior;

    auto isValid() const -> bool {
        if (!geometry.isValid()) {
            return false;
        }
        const size_t n = geometry.cellCount();
        return ground_z.size() == n && overhang_min_z.size() == n;
    }

    auto hasGroundAt(int ix, int iy) const -> bool {
        if (!isValid() || ix < 0 || iy < 0 || ix >= geometry.voxel_num.x() || iy >= geometry.voxel_num.y()) {
            return false;
        }
        return std::isfinite(ground_z[static_cast<size_t>(geometry.toAddress(ix, iy))]);
    }

    auto groundZAt(int ix, int iy) const -> float {
        if (!isValid() || ix < 0 || iy < 0 || ix >= geometry.voxel_num.x() || iy >= geometry.voxel_num.y()) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return ground_z[static_cast<size_t>(geometry.toAddress(ix, iy))];
    }

    auto overhangMinZAt(int ix, int iy) const -> float {
        if (!isValid() || ix < 0 || iy < 0 || ix >= geometry.voxel_num.x() || iy >= geometry.voxel_num.y()) {
            return std::numeric_limits<float>::infinity();
        }
        return overhang_min_z[static_cast<size_t>(geometry.toAddress(ix, iy))];
    }

    auto setGroundZ(int ix, int iy, float z) -> void {
        if (!isValid() || ix < 0 || iy < 0 || ix >= geometry.voxel_num.x() || iy >= geometry.voxel_num.y()) {
            return;
        }
        ground_z[static_cast<size_t>(geometry.toAddress(ix, iy))] = z;
    }

    auto setOverhangMinZ(int ix, int iy, float z) -> void {
        if (!isValid() || ix < 0 || iy < 0 || ix >= geometry.voxel_num.x() || iy >= geometry.voxel_num.y()) {
            return;
        }
        overhang_min_z[static_cast<size_t>(geometry.toAddress(ix, iy))] = z;
    }

    // 采样世界坐标 (x, y) 处的静态地面高度：
    // - 命中格本身有有限 ground_z 时直接返回；
    // - 否则按切比雪夫半径 r = 1..fallback_radius_cells 逐环搜索，取最近非空环的中位数，避免跨高台边沿均值混合；
    // - 越界或半径内无有效地面时返回 NaN。
    auto sampleGroundZ(const Eigen::Vector2d& world_xy, int fallback_radius_cells = 2) const -> float;

    // 采样世界坐标 (x, y) 邻域（radius_cells 切比雪夫方窗）内的最低已知悬空下沿高度；无已知悬空返回 +inf。
    auto sampleOverhangMinZ(const Eigen::Vector2d& world_xy, int radius_cells = 1) const -> float;

    // 裁剪子区域 [x_min..x_max] x [y_min..y_max]，与 MapIO::save 的裁剪窗口严格对齐。
    auto crop(int x_min, int x_max, int y_min, int y_max) const -> StaticTerrainPrior;

    // 为 mask 中置位但当前 ground_z 为 NaN 的格（如清理后的针孔 FREE 格）插值填充邻域中位数地面高度。
    auto interpolateMissingGround(std::span<const uint8_t> target_mask, int search_radius = 2) -> int;

    // 由 .pgm / .yaml / .yml / .terrain.bin 路径推导同 stem 伴随文件路径 <stem>.terrain.bin。
    static auto companionPath(std::string_view map_path) -> std::string;

    // 原子写入 .terrain.bin（先写 .tmp 再 rename）。失败无残留且不修改已有目标文件。
    static auto save(std::string_view path, const StaticTerrainPrior& prior) -> tl::expected<void, std::string>;

    // 读取 .terrain.bin（不校验外部几何，适用于离线检视与单元测试）。
    static auto load(std::string_view path) -> tl::expected<StaticTerrainPrior, std::string>;

    // 读取 .terrain.bin 并严格校验其几何与 expected_geom 一致（分辨率、原点、尺寸完全匹配）。
    static auto load(std::string_view path, const grid_map::GridGeometry& expected_geom)
        -> tl::expected<StaticTerrainPrior, std::string>;
};

} // namespace ma_map
