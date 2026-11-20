/*
    GridMap -> minco_opt::ESDFInterface 适配器
    把现有 2D 栅格地图的 ESDF(有符号距离场)暴露给 MincoOptimizer 使用。
*/

#pragma once

#include "map/core/grid_map.hpp"
#include "traj_optimize/esdf_Interface.hpp"
#include <memory>
#include <utility>

namespace minco_opt {

/**
 * @brief grid_map::GridMap -> ESDFInterface 适配器
 *
 * 语义对齐:
 * - getDistance: 自由空间为正、障碍物内为负,与库期望的"有符号距离"一致;
 *   地图外点 GridMap::getDistance 返回 max,库内部有 isInside 前置判断,安全。
 * - getGradient : 距离场梯度(方向远离障碍),供避障代价链式求导。
 * - isInside    : 地图外点不做避障/可行性采样惩罚。
 *
 * 注意:构造后 map 指针必须保持有效,直到优化结束
 *       (由调用方持有的 shared_ptr 保证)。
 */
class GridMapESDF : public ESDFInterface<GridMapESDF> {
public:
    explicit GridMapESDF(std::shared_ptr<const grid_map::GridMap> map) : map_(std::move(map)) {}

    auto getDistance(double x, double y) const ->double{
        return map_->getDistance(Eigen::Vector2d(x, y));
    }

    auto getGradient(double x, double y) const -> Eigen::Vector2d {
        double d = 0.0;
        Eigen::Vector2d grad = Eigen::Vector2d::Zero();
        map_->getDistanceAndGradient(Eigen::Vector2d(x, y), d, grad);
        return grad;
    }

    auto getDistanceAndGradient(double x, double y, double& distance, Eigen::Vector2d& gradient) const -> bool {
        return map_->getDistanceAndGradient(Eigen::Vector2d(x, y), distance, gradient);
    }

    auto isInside(double x, double y) const -> bool {
        return map_->isInsideMap(Eigen::Vector2d(x, y));
    }

    auto isCollision(double x, double y, double safe_distance) const -> bool {
        return map_->isCollision(Eigen::Vector2d(x, y), safe_distance);
    }

    auto outOfBoundsOccupied() const -> bool {
        return map_->outOfBoundsPolicy() == grid_map::OutOfBoundsPolicy::OCCUPIED;
    }

    auto resolution() const -> double {
        return map_->getResolution();
    }

private:
    std::shared_ptr<const grid_map::GridMap> map_;
};

} // namespace minco_opt
