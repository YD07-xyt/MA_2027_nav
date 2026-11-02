#pragma once

#include "utils/scope_timer.hpp"
#include "map/grid_map.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <spdlog/spdlog.h>
#include <unordered_map>
#include <vector>

namespace path_planning {

/*
 * 8 连通栅格 A*。
 *  - 节点用栅格线性地址 idx = x * ny + y 表示,搜索状态存放在扁平数组中,
 *    通过代际计数器(search_gen_)复用,避免每次搜索清零和堆分配。
 *  - 占用判定(栅格中心 ESDF < safe_threshold)按栅格惰性求值并缓存,
 *    语义与 JPS 一致;越界栅格视为被占用。
 *  - 启发函数为八叉距离(以栅格为单位),f 相同时优先展开 h 更小的节点。
 */
class AStar {
public:
    AStar(grid_map::GridMap& map, double safe_threshold);
    AStar()=default;

    auto set_map(const grid_map::GridMap& map)-> void;
    auto set_safe_threshold(const double& safe_threshold)->void{
      safe_threshold_=safe_threshold;
      std::fill(blocked_cache_.begin(), blocked_cache_.end(), 0); // 安全阈值变化后占用可能改变
    }
    // 返回航点路径(栅格中心,首点为精确起点,末点为精确终点)。
    // 返回空 vector = 起点/终点碰撞、无路径或超时。
    auto original_astar_search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, int timeout_ms = 1000)
        -> std::vector<Eigen::Vector2d>;


private:
    grid_map::GridMap map_;
    double safe_threshold_ = 0.0;

    // 缓存的栅格几何信息(与 map_ 保持一致)
    int nx_ = 0;
    int ny_ = 0;
    Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
    double resolution_ = 0.1;

    // 逐栅格占用缓存:0 = 未知, 1 = 自由, -1 = 被占用
    mutable std::vector<int8_t> blocked_cache_;

    // 单次搜索的扁平数组,带代际保护
    std::vector<int32_t> open_gen_;
    std::vector<int32_t> closed_gen_;
    std::vector<double> g_score_;
    std::vector<int32_t> parent_; // 父节点线性地址,-1 = 无
    int32_t search_gen_ = 0;

    // 带缓存的逐栅格占用判定;越界栅格一律视为被占用。
    auto blocked(int x, int y) const -> bool;

    // 八叉距离(以栅格为单位)
    static auto heuristic_cells(int dx, int dy) -> double;

    // Reconstruct path
    auto reconstruct_path(int goal_lin) const -> std::vector<Eigen::Vector2d>;

    auto check_collision(const Eigen::Vector2d& pos) const -> bool;

};

} // namespace path_planning