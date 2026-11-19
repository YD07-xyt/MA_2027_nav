#include "path_planning/search/jps.h"
#include "utils/logger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace path_planning {

namespace {
constexpr double K_SQRT2 = 1.4142135623730951; // 对角步长(栅格单位)
}

auto JPS::sync_map_cache(const grid_map::GridMap& map) -> bool {
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
        f_score_.assign(n, 0.0);
        parent_.assign(n, -1);
        search_gen_ = 0; // 数组刚被整体清零,代际可以安全复位
    }
    // 注意:尺寸不变时不能重置 search_gen_。代际计数器必须保持单调递增:一旦被重置,
    // 下一次搜索会复用 gen=1,上一轮搜索写入的 closed_gen_/open_gen_ 会被误认为
    // 是当前轮的记录,导致起点节点弹出时被当作"过期条目"跳过,搜索立刻以 expanded=0 失败。

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

auto JPS::blocked(const grid_map::GridMap& map, int x, int y) const -> bool {
    // 越界栅格一律视为被占用(射线不能离开栅格地图)。
    if (!in_map(x, y)) return true;

    const int a = addr(x, y);
    const int8_t c = blocked_cache_[a];
    if (c != 0) return c < 0;

    // 与 AStar::check_collision 相同的碰撞语义,在栅格中心求值
    // (对栅格中心而言,该值就等于 ESDF 栅格的原始值)。
    const Eigen::Vector2d center((x + 0.5) * resolution_ + origin_.x(), (y + 0.5) * resolution_ + origin_.y());
    blocked_cache_[a] = (map.getDistance(center) < safe_threshold_) ? -1 : 1;
    return blocked_cache_[a] < 0;
}

auto JPS::heuristic_cells(int dx, int dy) -> double {
    dx = std::abs(dx);
    dy = std::abs(dy);
    const int mn = std::min(dx, dy);
    const int mx = std::max(dx, dy);
    return static_cast<double>(mx) + (K_SQRT2 - 1.0) * static_cast<double>(mn);
}

auto JPS::has_forced_neighbor(const grid_map::GridMap& map, int x, int y, int dx, int dy) const -> bool {
    if (dx != 0 && dy != 0) {
        // 对角来向:强制邻居为 (x-dx, y+dy) 或 (x+dx, y-dy)
        if (blocked(map, x - dx, y) && !blocked(map, x - dx, y + dy)) return true;
        if (blocked(map, x, y - dy) && !blocked(map, x + dx, y - dy)) return true;
    } else if (dx != 0) {
        // 水平来向:强制邻居为 (x+dx, y+1) 或 (x+dx, y-1)
        if (blocked(map, x, y + 1) && !blocked(map, x + dx, y + 1)) return true;
        if (blocked(map, x, y - 1) && !blocked(map, x + dx, y - 1)) return true;
    } else {
        // 垂直来向:强制邻居为 (x+1, y+dy) 或 (x-1, y+dy)
        if (blocked(map, x + 1, y) && !blocked(map, x + 1, y + dy)) return true;
        if (blocked(map, x - 1, y) && !blocked(map, x - 1, y + dy)) return true;
    }
    return false;
}

auto JPS::jump_ray(const grid_map::GridMap& map, int x, int y, int dx, int dy, Eigen::Vector2i& jp) const -> bool {
    int cx = x;
    int cy = y;
    while (true) {
        const int nx = cx + dx;
        const int ny = cy + dy;

        // 终点所在栅格可以直接终止射线,即使其栅格中心落在膨胀区内
        // (精确终点坐标已在入口处校验为无碰撞)。
        if (nx == goal_x_ && ny == goal_y_) {
            jp << nx, ny;
            return true;
        }
        if (blocked(map, nx, ny)) return false; // 撞到障碍物或地图边界

        cx = nx;
        cy = ny;

        // 存在强制邻居 -> (cx, cy) 是跳点。
        if (has_forced_neighbor(map, cx, cy, dx, dy)) {
            jp << cx, cy;
            return true;
        }
        // 对角移动:若两个正交分量射线中任意一条能找到跳点
        // (前方存在必然转向),则 (cx, cy) 是跳点。
        if (dx != 0 && dy != 0) {
            Eigen::Vector2i tmp;
            if (jump_ray(map, cx, cy, dx, 0, tmp) || jump_ray(map, cx, cy, 0, dy, tmp)) {
                jp << cx, cy;
                return true;
            }
        }
    }
}

auto JPS::neighbor_directions(const grid_map::GridMap& map, int x, int y, int in_dx, int in_dy, int dirs[8][2]) const
    -> int {
    int n = 0;
    const auto add = [&](int dx, int dy) {
        for (int i = 0; i < n; ++i) {
            if (dirs[i][0] == dx && dirs[i][1] == dy) return;
        }
        dirs[n][0] = dx;
        dirs[n][1] = dy;
        ++n;
    };

    if (in_dx == 0 && in_dy == 0) {
        // 起点节点:没有父节点,扫描全部 8 个方向。
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                add(dx, dy);
            }
        }
        return n;
    }

    if (in_dx != 0 && in_dy != 0) {
        // 对角来向:自然方向 = 继续沿对角 + 两个正交分量,
        // 再加上两个强制邻居方向。
        add(in_dx, in_dy);
        add(in_dx, 0);
        add(0, in_dy);
        if (blocked(map, x - in_dx, y) && !blocked(map, x - in_dx, y + in_dy)) add(-in_dx, in_dy);
        if (blocked(map, x, y - in_dy) && !blocked(map, x + in_dx, y - in_dy)) add(in_dx, -in_dy);
    } else if (in_dx != 0) {
        // 水平来向。
        add(in_dx, 0);
        add(in_dx, 1);
        add(in_dx, -1);
        if (blocked(map, x, y + 1) && !blocked(map, x + in_dx, y + 1)) add(in_dx, 1);
        if (blocked(map, x, y - 1) && !blocked(map, x + in_dx, y - 1)) add(in_dx, -1);
    } else {
        // 垂直来向。
        add(0, in_dy);
        add(1, in_dy);
        add(-1, in_dy);
        if (blocked(map, x + 1, y) && !blocked(map, x + 1, y + in_dy)) add(1, in_dy);
        if (blocked(map, x - 1, y) && !blocked(map, x - 1, y + in_dy)) add(-1, in_dy);
    }
    return n;
}

auto JPS::check_collision(const grid_map::GridMap& map, const Eigen::Vector2d& pos) const -> bool {
    // 地图外的点视为无碰撞(AStar 的约定)。
    if (!map.isInsideMap(pos)) return false;
    return map.getDistance(pos) < safe_threshold_;
}

auto JPS::reconstruct_path(int goal_lin) const -> std::vector<Eigen::Vector2d> {
    std::vector<Eigen::Vector2d> path;
    int cur = goal_lin;
    while (cur >= 0) {
        const int x = cur / ny_;
        const int y = cur % ny_;
        path.emplace_back(
            (x + 0.5) * resolution_ + origin_.x(),
            (y + 0.5) * resolution_ + origin_.y());
        cur = parent_[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

auto JPS::jps_search(
    const grid_map::GridMap& map,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    int timeout_ms
) -> std::vector<Eigen::Vector2d> {
    const auto start_time = std::chrono::steady_clock::now();

    if (!sync_map_cache(map)) {
        logger::error(logger::planning, "JPS: map is not set!");
        return {};
    }

    // 与 AStar::original_astar_search 相同的起终点有效性检查。
    if (check_collision(map, start)) {
        logger::error(logger::planning, "JPS: start point in collision!");
        return {};
    }
    if (check_collision(map, goal)) {
        logger::error(logger::planning, "JPS: goal point in collision!");
        return {};
    }

    // 把起点/终点吸附到栅格,并限制在地图范围内。
    Eigen::Vector2i start_idx;
    Eigen::Vector2i goal_idx;
    map.posToIndex(start, start_idx);
    map.posToIndex(goal, goal_idx);
    start_idx.x() = std::clamp(start_idx.x(), 0, nx_ - 1);
    start_idx.y() = std::clamp(start_idx.y(), 0, ny_ - 1);
    goal_idx.x() = std::clamp(goal_idx.x(), 0, nx_ - 1);
    goal_idx.y() = std::clamp(goal_idx.y(), 0, ny_ - 1);

    // 起点所在栅格可能被占用 / 孤立(无人机贴地图边界或处于狭窄凹坑中)。
    // A* 只检查离散点所以能容忍这种情况,而 JPS 需要一个自由栅格作为
    // 扩展起点——因此吸附到最近的可通行自由栅格(最多 1.5 m),保持行为一致。
    const int max_snap_radius = std::max(1, static_cast<int>(1.5 / resolution_ + 0.5));
    Eigen::Vector2i root_idx;
    if (!find_navigable_cell(map, start_idx.x(), start_idx.y(), max_snap_radius, root_idx)) {
        logger::error(
            logger::planning,
            "JPS: no navigable cell within {:.1f} m of start cell ({},{}), start=({:.3f},{:.3f})",
            1.5,
            start_idx.x(),
            start_idx.y(),
            start.x(),
            start.y()
        );
        return {};
    }
    const bool start_snapped = (root_idx != start_idx);
    if (start_snapped) {
        logger::warn(
            logger::planning,
            "JPS: start not navigable, snapped ({:.3f},{:.3f}) -> ({:.3f},{:.3f})",
            start.x(),
            start.y(),
            (root_idx.x() + 0.5) * resolution_ + origin_.x(),
            (root_idx.y() + 0.5) * resolution_ + origin_.y()
        );
    }

    // ---- 单次搜索的状态 ----
    if (++search_gen_ <= 0) { // int32 溢出保护
        std::fill(open_gen_.begin(), open_gen_.end(), 0);
        std::fill(closed_gen_.begin(), closed_gen_.end(), 0);
        search_gen_ = 1;
    }
    const int32_t gen = search_gen_;

    goal_x_ = goal_idx.x();
    goal_y_ = goal_idx.y();

    const int start_lin = addr(root_idx.x(), root_idx.y());
    const int goal_lin = addr(goal_idx.x(), goal_idx.y());

    using HeapEntry = std::pair<double, int>;
    std::priority_queue<HeapEntry, std::vector<HeapEntry>, std::greater<HeapEntry>> open;

    g_score_[start_lin] = 0.0;
    f_score_[start_lin] = heuristic_cells(goal_idx.x() - root_idx.x(), goal_idx.y() - root_idx.y());
    parent_[start_lin] = -1;
    open_gen_[start_lin] = gen;
    open.emplace(f_score_[start_lin], start_lin);

    int expanded = 0;

    while (!open.empty()) {
        // 超时检查(与 AStar 的粒度一致)。
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count()
            > timeout_ms)
        {
            logger::error(logger::planning, "JPS timeout!");
            return {};
        }

        const auto [f, lin] = open.top();
        open.pop();

        // 跳过过期堆条目(已展开,或被更小的 f 取代)。
        if (closed_gen_[lin] == gen || f_score_[lin] != f) continue;

        if (lin == goal_lin) {
            auto path = reconstruct_path(goal_lin);
            // 与 AStar 约定一致:raw path 以精确起点开始
            // (仅当搜索根就是真实起点栅格时),并始终以精确终点结束。
            if (!start_snapped) path.front() = start;
            path.back() = goal;
            return path;
        }

        closed_gen_[lin] = gen;
        ++expanded;

        const int x = lin / ny_;
        const int y = lin % ny_;

        // 来向(父节点 -> 当前节点的方向),用于剪枝。
        int in_dx = 0;
        int in_dy = 0;
        const int p_lin = parent_[lin];
        if (p_lin >= 0) {
            in_dx = std::clamp(x - p_lin / ny_, -1, 1);
            in_dy = std::clamp(y - p_lin % ny_, -1, 1);
        }

        int dirs[8][2];
        const int ndirs = neighbor_directions(map, x, y, in_dx, in_dy, dirs);

        for (int i = 0; i < ndirs; ++i) {
            const int dx = dirs[i][0];
            const int dy = dirs[i][1];

            Eigen::Vector2i jp;
            if (!jump_ray(map, x, y, dx, dy, jp)) continue;

            const int jlin = addr(jp.x(), jp.y());
            if (closed_gen_[jlin] == gen) continue; // 一致启发 -> 已展开节点不可能再被改进

            const int steps = std::max(std::abs(jp.x() - x), std::abs(jp.y() - y));
            const double step_cost = (dx != 0 && dy != 0) ? K_SQRT2 : 1.0;
            const double tentative_g = g_score_[lin] + static_cast<double>(steps) * step_cost;

            const double old_g = (open_gen_[jlin] == gen)
                                     ? g_score_[jlin]
                                     : std::numeric_limits<double>::infinity();
            if (tentative_g < old_g) {
                g_score_[jlin] = tentative_g;
                f_score_[jlin] = tentative_g + heuristic_cells(goal_idx.x() - jp.x(), goal_idx.y() - jp.y());
                parent_[jlin] = lin;
                open_gen_[jlin] = gen;
                open.emplace(f_score_[jlin], jlin);
            }
        }
    }

    // open set 耗尽 -> 无路径。记录足够的上下文以便诊断失败原因
    // (例如起点被 clamp 到地图边界且其邻居全部被占用)。
    logger::error(
        logger::planning,
        "JPS: no path! start=({:.3f},{:.3f}) goal=({:.3f},{:.3f}) root_cell=({},{}) goal_cell=({},{}) "
        "map={}x{} res={:.3f} snapped={} root_blocked={} n_blocked_neighbors={} expanded={}",
        start.x(),
        start.y(),
        goal.x(),
        goal.y(),
        root_idx.x(),
        root_idx.y(),
        goal_idx.x(),
        goal_idx.y(),
        nx_,
        ny_,
        resolution_,
        start_snapped,
        blocked(map, root_idx.x(), root_idx.y()),
        blocked_neighbor_count(map, root_idx.x(), root_idx.y()),
        expanded
    );
    return {};
}

// 8 个邻居中是否至少有一个自由栅格。
auto JPS::has_free_neighbor(const grid_map::GridMap& map, int x, int y) const -> bool {
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            if (!blocked(map, x + dx, y + dy)) return true;
        }
    }
    return false;
}

// 8 个邻居中被占用栅格的个数(用于诊断日志)。
auto JPS::blocked_neighbor_count(const grid_map::GridMap& map, int x, int y) const -> int {
    int cnt = 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            if (blocked(map, x + dx, y + dy)) ++cnt;
        }
    }
    return cnt;
}

// 环形搜索最近的一个自由且至少有一个自由邻居的栅格。
auto JPS::find_navigable_cell(const grid_map::GridMap& map, int x, int y, int max_radius, Eigen::Vector2i& out) const
    -> bool {
    for (int r = 0; r <= max_radius; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
                const int nx = x + dx;
                const int ny = y + dy;
                if (!in_map(nx, ny)) continue;
                if (blocked(map, nx, ny)) continue;
                if (!has_free_neighbor(map, nx, ny)) continue;
                out << nx, ny;
                return true;
            }
        }
    }
    return false;
}

} // namespace path_planning
