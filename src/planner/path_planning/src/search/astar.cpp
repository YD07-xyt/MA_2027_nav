#include "path_planning/search/astar.h"
#include "utils/logger.hpp"

#include <cstdlib>
#include <tuple>

namespace path_planning {

namespace {
constexpr double K_SQRT2 = 1.4142135623730951; // 对角步长(栅格单位)
constexpr int K_TIMEOUT_CHECK_INTERVAL = 256;  // 每展开 N 个节点检查一次超时
}

auto AStar::sync_map_cache(const grid_map::GridMap& map) -> bool {
    const Eigen::Vector2i voxel_num = map.getVoxelNum();
    nx_ = voxel_num.x();
    ny_ = voxel_num.y();
    origin_ = map.getOrigin();
    resolution_ = map.getResolution();

    if (nx_ <= 0 || ny_ <= 0 || resolution_ <= 0.0) {
        return false;
    }

    const size_t n = static_cast<size_t>(nx_) * static_cast<size_t>(ny_);
    if (open_gen_.size() != n) {
        open_gen_.assign(n, 0);
        closed_gen_.assign(n, 0);
        g_score_.assign(n, 0.0);
        parent_.assign(n, -1);
        search_gen_ = 0; // 数组刚被整体清零,代际可以安全复位
    }
    // 尺寸不变时不能复位 search_gen_,否则上一轮的 open/closed 标记会被误认为本轮记录。

    const uint64_t esdf_rev = map.esdf_revision();
    const bool cache_hit = cache_valid_ && esdf_rev != 0 && cached_esdf_revision_ == esdf_rev
        && cached_safe_threshold_ == safe_threshold_ && blocked_cache_.size() == n;
    if (!cache_hit) {
        blocked_cache_.assign(n, 0); // ESDF/阈值/尺寸变化时清空占用缓存
        cached_esdf_revision_ = esdf_rev;
        cached_safe_threshold_ = safe_threshold_;
        cache_valid_ = (esdf_rev != 0);
    }
    return true;
}

auto AStar::original_astar_search(
    const grid_map::GridMap& map,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    int timeout_ms
) -> std::vector<Eigen::Vector2d> {
    auto start_time = std::chrono::steady_clock::now();

    if (!sync_map_cache(map)) {
        logger::error(logger::planning, "A*: map is not set!");
        return {};
    }

    // Check start/goal validity
    if (check_collision(map, start)) {
        logger::error(logger::planning, "start point in collision!");
        return {};
    }
    if (check_collision(map, goal)) {
        logger::error(logger::planning, "Goal point in collision!");
        return {};
    }

    // 起点/终点吸附到栅格,并限制在地图范围内
    Eigen::Vector2i start_idx;
    Eigen::Vector2i goal_idx;
    map.posToIndex(start, start_idx);
    map.posToIndex(goal, goal_idx);
    start_idx.x() = std::clamp(start_idx.x(), 0, nx_ - 1);
    start_idx.y() = std::clamp(start_idx.y(), 0, ny_ - 1);
    goal_idx.x() = std::clamp(goal_idx.x(), 0, nx_ - 1);
    goal_idx.y() = std::clamp(goal_idx.y(), 0, ny_ - 1);

    // ---- 单次搜索状态(代际计数,免清零)----
    if (++search_gen_ <= 0) { // int32 溢出保护
        std::fill(open_gen_.begin(), open_gen_.end(), 0);
        std::fill(closed_gen_.begin(), closed_gen_.end(), 0);
        search_gen_ = 1;
    }
    const int32_t gen = search_gen_;

    const int gx = goal_idx.x();
    const int gy = goal_idx.y();
    const int start_lin = start_idx.x() * ny_ + start_idx.y();
    const int goal_lin = gx * ny_ + gy;

    // 堆条目:(f, h, idx)。f 相同时 h 小者优先,减少等 f 平台上的展开数。
    using HeapEntry = std::tuple<double, double, int>;
    std::priority_queue<HeapEntry, std::vector<HeapEntry>, std::greater<HeapEntry>> open_set;

    const double h0 = heuristic_cells(gx - start_idx.x(), gy - start_idx.y());
    g_score_[start_lin] = 0.0;
    parent_[start_lin] = -1;
    open_gen_[start_lin] = gen;
    open_set.emplace(h0, h0, start_lin);

    int iter = 0;
    while (!open_set.empty()) {
        // Check timeout(降频检查)
        if (++iter % K_TIMEOUT_CHECK_INTERVAL == 0
            && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time)
                    .count()
                > timeout_ms)
        {
            logger::error(logger::planning,"A* timeout!");
            return {};
        }

        const int lin = std::get<2>(open_set.top());
        open_set.pop();

        // 惰性删除:已展开过的过期条目直接跳过
        if (closed_gen_[lin] == gen) continue;
        closed_gen_[lin] = gen;

        // Reached goal
        if (lin == goal_lin) {
            auto path = reconstruct_path(goal_lin);
            path.front() = start; // 与原实现一致:首点为精确起点
            if (path.size() > 1) path.back() = goal;
            else path.push_back(goal);
            return path;
        }

        const int x = lin / ny_;
        const int y = lin % ny_;
        const double g_cur = g_score_[lin];

        // Expand neighbors (8 directions)
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || nx >= nx_ || ny < 0 || ny >= ny_) continue;

                const int nlin = nx * ny_ + ny;
                if (closed_gen_[nlin] == gen) continue;

                // 终点栅格即使中心落在膨胀区内也允许进入(精确终点已在入口校验)
                if (nlin != goal_lin && blocked(map, nx, ny)) continue;

                const double step_cost = (dx != 0 && dy != 0) ? K_SQRT2 : 1.0; // Orthogonal/diagonal cost
                const double tentative_g = g_cur + step_cost;

                if (open_gen_[nlin] != gen || tentative_g < g_score_[nlin]) {
                    // New node or found better path
                    g_score_[nlin] = tentative_g;
                    parent_[nlin] = lin;
                    open_gen_[nlin] = gen;
                    const double h = heuristic_cells(gx - nx, gy - ny);
                    open_set.emplace(tentative_g + h, h, nlin);
                }
            }
        }
    }

    logger::error(logger::planning, "A*: no path!");
    return {}; // Open set empty, no path found
}

auto AStar::blocked(const grid_map::GridMap& map, int x, int y) const -> bool {
    if (x < 0 || x >= nx_ || y < 0 || y >= ny_) return true;

    const int a = x * ny_ + y;
    const int8_t c = blocked_cache_[a];
    if (c != 0) return c < 0;

    // 在栅格中心求值 ESDF
    const Eigen::Vector2d center((x + 0.5) * resolution_ + origin_.x(), (y + 0.5) * resolution_ + origin_.y());
    blocked_cache_[a] = (map.getDistance(center) < safe_threshold_) ? -1 : 1;
    return blocked_cache_[a] < 0;
}

// 八叉距离(栅格单位),8 连通下的一致启发函数
auto AStar::heuristic_cells(int dx, int dy) -> double {
    dx = std::abs(dx);
    dy = std::abs(dy);
    const int mn = std::min(dx, dy);
    const int mx = std::max(dx, dy);
    return static_cast<double>(mx) + (K_SQRT2 - 1.0) * static_cast<double>(mn);
}

// Reconstruct path(栅格中心坐标)
auto AStar::reconstruct_path(int goal_lin) const -> std::vector<Eigen::Vector2d> {
    std::vector<Eigen::Vector2d> path;
    int cur = goal_lin;
    while (cur >= 0) {
        const int x = cur / ny_;
        const int y = cur % ny_;
        path.emplace_back((x + 0.5) * resolution_ + origin_.x(), (y + 0.5) * resolution_ + origin_.y());
        cur = parent_[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

auto AStar::check_collision(const grid_map::GridMap& map, const Eigen::Vector2d& pos) const -> bool {
    // Points outside map are considered collision-free
    if (!map.isInsideMap(pos)) return false;

    // Points inside map use safety distance check
    return map.getDistance(pos) < safe_threshold_;
}
}
