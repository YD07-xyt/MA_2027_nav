#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace grid_map {

// 地图外区域语义策略：
// - FREE     : 兼容旧约定/建图模式（地图外视为自由，getDistance 返回 max，isCollision 返回 false）。
// - OCCUPIED : 导航定图模式（地图外与四条地图边界视为虚拟墙障碍：
//              地图内 d_eff = min(d_esdf, d_bound)，地图外返回负欧氏距离与指向地图内的单位梯度）。
enum class OutOfBoundsPolicy : uint8_t {
    FREE = 0,
    OCCUPIED = 1,
};

// 2D 栅格几何值对象（不可变快照元数据与坐标换算单一真相源）。
// 坐标系：世界系 2D (x, y)，单位米 (m)。
// 边界约定：左闭右开 [min_boundary, max_boundary)，保证 isInside(pos) 为真时 posToIndex(pos) 必然合法。
struct GridGeometry {
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    Eigen::Vector2i voxel_num = Eigen::Vector2i::Zero();
    double resolution = 0.0;
    double resolution_inv = 0.0;
    Eigen::Vector2d map_size = Eigen::Vector2d::Zero();
    Eigen::Vector2d min_boundary = Eigen::Vector2d::Zero();
    Eigen::Vector2d max_boundary = Eigen::Vector2d::Zero();

    static auto
    fromOriginAndSize(const Eigen::Vector2d& map_origin, const Eigen::Vector2i& voxels, double res) -> GridGeometry {
        GridGeometry g;
        if (voxels.x() <= 0 || voxels.y() <= 0 || res <= 0.0 || !std::isfinite(res) || !map_origin.allFinite()) {
            return g;
        }
        g.origin = map_origin;
        g.voxel_num = voxels;
        g.resolution = res;
        g.resolution_inv = 1.0 / res;
        g.map_size = Eigen::Vector2d(static_cast<double>(voxels.x()) * res, static_cast<double>(voxels.y()) * res);
        g.min_boundary = map_origin;
        g.max_boundary = map_origin + g.map_size;
        return g;
    }

    // 按原点向四个方向的延伸量构建几何：x ∈ [-x_left, +x_right], y ∈ [-y_left, +y_right]。
    // 对整数倍尺寸使用 -1e-9 容差，避免浮点除法误差导致体素数多算 1 格。
    static auto fromBounds(double x_left, double x_right, double y_left, double y_right, double res) -> GridGeometry {
        if (res <= 0.0 || !std::isfinite(res)) {
            return GridGeometry {};
        }
        const double size_x = x_right + x_left;
        const double size_y = y_right + y_left;
        if (size_x <= 0.0 || size_y <= 0.0 || !std::isfinite(size_x) || !std::isfinite(size_y) || !std::isfinite(x_left)
            || !std::isfinite(y_left))
        {
            return GridGeometry {};
        }
        const int nx = std::max(1, static_cast<int>(std::ceil(size_x / res - 1e-9)));
        const int ny = std::max(1, static_cast<int>(std::ceil(size_y / res - 1e-9)));
        return fromOriginAndSize(Eigen::Vector2d(-x_left, -y_left), Eigen::Vector2i(nx, ny), res);
    }

    auto isValid() const -> bool {
        return voxel_num.x() > 0 && voxel_num.y() > 0 && resolution > 0.0;
    }

    auto cellCount() const -> size_t {
        return isValid() ? static_cast<size_t>(voxel_num.x()) * static_cast<size_t>(voxel_num.y()) : 0;
    }

    auto diagonal() const -> double {
        return map_size.norm();
    }

    auto posToIndex(const Eigen::Vector2d& pos) const -> Eigen::Vector2i {
        return Eigen::Vector2i(
            static_cast<int>(std::floor((pos.x() - origin.x()) * resolution_inv)),
            static_cast<int>(std::floor((pos.y() - origin.y()) * resolution_inv))
        );
    }

    auto indexToPos(const Eigen::Vector2i& id) const -> Eigen::Vector2d {
        return Eigen::Vector2d((id.x() + 0.5) * resolution + origin.x(), (id.y() + 0.5) * resolution + origin.y());
    }

    auto isInside(const Eigen::Vector2i& idx) const -> bool {
        return idx.x() >= 0 && idx.y() >= 0 && idx.x() < voxel_num.x() && idx.y() < voxel_num.y();
    }

    auto isInside(const Eigen::Vector2d& pos) const -> bool {
        if (!isValid()) {
            return false;
        }
        if (!(pos.x() >= min_boundary.x() && pos.x() < max_boundary.x() && pos.y() >= min_boundary.y()
              && pos.y() < max_boundary.y()))
        {
            return false;
        }
        const double lx = (pos.x() - origin.x()) * resolution_inv;
        const double ly = (pos.y() - origin.y()) * resolution_inv;
        return lx >= 0.0 && lx < static_cast<double>(voxel_num.x()) && ly >= 0.0
            && ly < static_cast<double>(voxel_num.y());
    }

    auto toAddress(int x, int y) const -> int {
        return x * voxel_num.y() + y;
    }

    auto toAddress(const Eigen::Vector2i& id) const -> int {
        return id.x() * voxel_num.y() + id.y();
    }

    auto operator==(const GridGeometry& other) const -> bool {
        return voxel_num == other.voxel_num && resolution == other.resolution && origin == other.origin;
    }

    auto operator!=(const GridGeometry& other) const -> bool {
        return !(*this == other);
    }
};

} // namespace grid_map
