#include "map/layered/map_layers.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>

namespace ma_map {

namespace {

    template<typename T>
    auto shiftGridBuffer(
        const std::vector<T>& src,
        int old_nx,
        int old_ny,
        int new_nx,
        int new_ny,
        const Eigen::Vector2i& shift,
        const T& fill_val
    ) -> std::vector<T> {
        std::vector<T> dst(static_cast<size_t>(new_nx) * static_cast<size_t>(new_ny), fill_val);
        if (old_nx <= 0 || old_ny <= 0 || new_nx <= 0 || new_ny <= 0
            || src.size() != static_cast<size_t>(old_nx) * static_cast<size_t>(old_ny))
        {
            return dst;
        }

        const int x_begin = std::max(0, -shift.x());
        const int x_end = std::min(old_nx, new_nx - shift.x());
        const int y_begin = std::max(0, -shift.y());
        const int y_end = std::min(old_ny, new_ny - shift.y());
        if (x_begin >= x_end || y_begin >= y_end) {
            return dst;
        }

        const size_t copy_len = static_cast<size_t>(y_end - y_begin);
        for (int ox = x_begin; ox < x_end; ++ox) {
            const size_t src_off = static_cast<size_t>(ox) * static_cast<size_t>(old_ny) + static_cast<size_t>(y_begin);
            const size_t dst_off = static_cast<size_t>(ox + shift.x()) * static_cast<size_t>(new_ny)
                + static_cast<size_t>(y_begin + shift.y());
            std::copy_n(src.data() + src_off, copy_len, dst.data() + dst_off);
        }
        return dst;
    }

    // 把 terrain 格中心映射到 2D 细格：向内收缩 1/4 细格避免边界浮点截断跳格，
    // 并直接跳过完全落在地图外的格，防止 OOB 点被 clamp 到边界产生幽灵障碍或误清边界。
    template<typename F>
    auto forEachTerrainCell(
        const grid_map::GridGeometry& geom,
        const utils::vec_E<utils::Vec3f>& cells,
        double terrain_res,
        F&& fn
    ) -> void {
        if (!geom.isValid() || cells.empty()) {
            return;
        }
        const double half_span = std::max(0.0, 0.5 * terrain_res - 0.25 * geom.resolution);
        const Eigen::Vector2i max_idx(geom.voxel_num.x() - 1, geom.voxel_num.y() - 1);
        for (const auto& c: cells) {
            if (!std::isfinite(c.x()) || !std::isfinite(c.y())) {
                continue;
            }
            if (c.x() + half_span < geom.min_boundary.x() || c.y() + half_span < geom.min_boundary.y()
                || c.x() - half_span >= geom.max_boundary.x() || c.y() - half_span >= geom.max_boundary.y())
            {
                continue;
            }
            Eigen::Vector2i lo = geom.posToIndex(Eigen::Vector2d(c.x() - half_span, c.y() - half_span));
            Eigen::Vector2i hi = geom.posToIndex(Eigen::Vector2d(c.x() + half_span, c.y() + half_span));
            if (hi.x() < 0 || hi.y() < 0 || lo.x() > max_idx.x() || lo.y() > max_idx.y()) {
                continue;
            }
            lo = lo.cwiseMax(Eigen::Vector2i::Zero());
            hi = hi.cwiseMin(max_idx);
            for (int x = lo.x(); x <= hi.x(); ++x) {
                for (int y = lo.y(); y <= hi.y(); ++y) {
                    fn(x, y);
                }
            }
        }
    }

} // namespace

// ==================== StaticMapLayer ====================

auto StaticMapLayer::resize(const grid_map::GridGeometry& new_geom, const Eigen::Vector2i& shift, bool preserve_data)
    -> void {
    if (!new_geom.isValid()) {
        geom_ = grid_map::GridGeometry {};
        occ_.clear();
        cells_.clear();
        terrain_prior_.reset();
        has_loaded_cells_ = false;
        from_binary_matrix_ = false;
        return;
    }

    if (preserve_data && geom_.isValid()) {
        occ_ = shiftGridBuffer<uint8_t>(
            occ_,
            geom_.voxel_num.x(),
            geom_.voxel_num.y(),
            new_geom.voxel_num.x(),
            new_geom.voxel_num.y(),
            shift,
            0
        );
        cells_ = shiftGridBuffer<CellState>(
            cells_,
            geom_.voxel_num.x(),
            geom_.voxel_num.y(),
            new_geom.voxel_num.x(),
            new_geom.voxel_num.y(),
            shift,
            CellState::UNKNOWN
        );
        terrain_prior_.reset();
        from_binary_matrix_ = false;
    } else {
        occ_.assign(new_geom.cellCount(), 0);
        cells_.assign(new_geom.cellCount(), CellState::UNKNOWN);
        terrain_prior_.reset();
        has_loaded_cells_ = false;
        from_binary_matrix_ = false;
    }
    geom_ = new_geom;
}

auto StaticMapLayer::reset() -> void {
    std::fill(occ_.begin(), occ_.end(), 0);
    std::fill(cells_.begin(), cells_.end(), CellState::UNKNOWN);
    terrain_prior_.reset();
    has_loaded_cells_ = false;
    from_binary_matrix_ = false;
}

auto StaticMapLayer::load(const LoadedMap& loaded) -> void {
    geom_ = loaded.geometry;
    occ_ = loaded.occupancy;
    cells_ = loaded.cells;
    if (loaded.terrain_prior.has_value() && loaded.terrain_prior->isValid()
        && loaded.terrain_prior->geometry == loaded.geometry)
    {
        terrain_prior_ = std::make_shared<StaticTerrainPrior>(*loaded.terrain_prior);
    } else {
        terrain_prior_.reset();
    }
    has_loaded_cells_ = geom_.isValid();
    from_binary_matrix_ = false;
}

auto StaticMapLayer::setOccupancyMatrix(const grid_map::RowMatrixXi& mat, bool* changed) -> bool {
    if (!geom_.isValid() || mat.rows() != geom_.voxel_num.x() || mat.cols() != geom_.voxel_num.y()) {
        logger::MaMap->error(
            "StaticMapLayer::setOccupancyMatrix size mismatch: got {}x{}, expected {}x{}",
            mat.rows(),
            mat.cols(),
            geom_.voxel_num.x(),
            geom_.voxel_num.y()
        );
        return false;
    }
    const size_t n = geom_.cellCount();
    const int* src = mat.data();

    if (has_loaded_cells_ && from_binary_matrix_ && occ_.size() == n && cells_.size() == n) {
        const uint8_t* dst = occ_.data();
        size_t i = 0;
        bool equal = true;
        for (; i + 64 <= n; i += 64) {
            uint8_t diff = 0;
            for (size_t k = 0; k < 64; ++k) {
                diff |= static_cast<uint8_t>((src[i + k] != 0) != (dst[i + k] != 0));
            }
            if (diff != 0) {
                equal = false;
                break;
            }
        }
        if (equal) {
            for (; i < n; ++i) {
                if ((src[i] != 0) != (dst[i] != 0)) {
                    equal = false;
                    break;
                }
            }
        }
        if (equal) {
            if (changed) {
                *changed = false;
            }
            return true;
        }
        for (; i < n; ++i) {
            const uint8_t v = src[i] != 0 ? 1 : 0;
            occ_[i] = v;
            cells_[i] = v != 0 ? CellState::OCCUPIED : CellState::FREE;
        }
        if (changed) {
            *changed = true;
        }
        return true;
    }

    occ_.resize(n);
    cells_.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const uint8_t v = src[i] != 0 ? 1 : 0;
        occ_[i] = v;
        cells_[i] = v != 0 ? CellState::OCCUPIED : CellState::FREE;
    }
    terrain_prior_.reset();
    has_loaded_cells_ = true;
    from_binary_matrix_ = true;
    if (changed) {
        *changed = true;
    }
    return true;
}

auto StaticMapLayer::toOccupancyMatrix() const -> grid_map::RowMatrixXi {
    if (!geom_.isValid()) {
        return {};
    }
    grid_map::RowMatrixXi out(geom_.voxel_num.x(), geom_.voxel_num.y());
    int* dst = out.data();
    const size_t n = occ_.size();
    for (size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<int>(occ_[i]);
    }
    return out;
}

// ==================== PersistentMapLayer ====================

auto PersistentMapLayer::resize(
    const grid_map::GridGeometry& new_geom,
    const Eigen::Vector2i& shift,
    bool preserve_data
) -> void {
    if (!new_geom.isValid()) {
        geom_ = grid_map::GridGeometry {};
        cells_.clear();
        occ_.clear();
        free_streak_.clear();
        free_last_frame_.clear();
        frame_id_ = 0;
        return;
    }

    if (preserve_data && geom_.isValid()) {
        const int old_nx = geom_.voxel_num.x();
        const int old_ny = geom_.voxel_num.y();
        const int new_nx = new_geom.voxel_num.x();
        const int new_ny = new_geom.voxel_num.y();
        cells_ = shiftGridBuffer<CellState>(cells_, old_nx, old_ny, new_nx, new_ny, shift, CellState::UNKNOWN);
        occ_ = shiftGridBuffer<uint8_t>(occ_, old_nx, old_ny, new_nx, new_ny, shift, 0);
        free_streak_ = shiftGridBuffer<int>(free_streak_, old_nx, old_ny, new_nx, new_ny, shift, 0);
        free_last_frame_ = shiftGridBuffer<int>(free_last_frame_, old_nx, old_ny, new_nx, new_ny, shift, -10);
    } else {
        const size_t n = new_geom.cellCount();
        cells_.assign(n, CellState::UNKNOWN);
        occ_.assign(n, 0);
        free_streak_.assign(n, 0);
        free_last_frame_.assign(n, -10);
        frame_id_ = 0;
    }
    geom_ = new_geom;
}

auto PersistentMapLayer::reset() -> void {
    std::fill(cells_.begin(), cells_.end(), CellState::UNKNOWN);
    std::fill(occ_.begin(), occ_.end(), 0);
    std::fill(free_streak_.begin(), free_streak_.end(), 0);
    std::fill(free_last_frame_.begin(), free_last_frame_.end(), -10);
    frame_id_ = 0;
}

auto PersistentMapLayer::update(const LocalTerrainFrame& frame) -> void {
    if (!geom_.isValid()) {
        return;
    }
    ++frame_id_;

    // 1. 障碍格优先：立即写 OCCUPIED，清零空闲连击并用 -frame_id_ 标记本帧已被障碍命中，
    //    防止同一帧内 obstacle_cells 与 free_cells 重叠映射时误累加空闲计数。
    forEachTerrainCell(geom_, frame.obstacle_cells, frame.terrain_resolution, [&](int x, int y) {
        const size_t idx = static_cast<size_t>(geom_.toAddress(x, y));
        cells_[idx] = CellState::OCCUPIED;
        occ_[idx] = 1;
        free_streak_[idx] = 0;
        free_last_frame_[idx] = -frame_id_;
    });

    // 2. 空闲证据：
    //    - UNKNOWN  -> 立即标记 FREE（已探索）
    //    - OCCUPIED -> authoritative_free 时单帧清为 FREE（3D 静态建图器已完成跨帧累计）；
    //                  否则连续 kFreeFramesToClear 帧空闲证据才清为 FREE（防细杆/网格单帧漏检）
    //    - FREE     -> 保持 FREE
    forEachTerrainCell(geom_, frame.free_cells, frame.terrain_resolution, [&](int x, int y) {
        const size_t idx = static_cast<size_t>(geom_.toAddress(x, y));
        const CellState st = cells_[idx];
        if (st == CellState::UNKNOWN) {
            cells_[idx] = CellState::FREE;
            occ_[idx] = 0;
            return;
        }
        if (st == CellState::FREE) {
            return;
        }
        if (free_last_frame_[idx] == -frame_id_ || free_last_frame_[idx] == frame_id_) {
            return;
        }
        if (frame.authoritative_free) {
            cells_[idx] = CellState::FREE;
            occ_[idx] = 0;
            free_streak_[idx] = 0;
            free_last_frame_[idx] = frame_id_;
            return;
        }
        free_streak_[idx] = (free_last_frame_[idx] == frame_id_ - 1) ? (free_streak_[idx] + 1) : 1;
        free_last_frame_[idx] = frame_id_;
        if (free_streak_[idx] >= kFreeFramesToClear) {
            cells_[idx] = CellState::FREE;
            occ_[idx] = 0;
            free_streak_[idx] = 0;
        }
    });

    // 2.5. 3D 静态擦除同步：当某列已无任何有效静态占据且本帧未被 obstacle_cells 命中时，
    //      将旧 OCCUPIED 退回 UNKNOWN（不覆盖 FREE，避免抹除已探索地面）。
    if (!frame.cleared_cells.empty()) {
        forEachTerrainCell(geom_, frame.cleared_cells, frame.terrain_resolution, [&](int x, int y) {
            const size_t idx = static_cast<size_t>(geom_.toAddress(x, y));
            if (cells_[idx] == CellState::OCCUPIED && free_last_frame_[idx] != -frame_id_) {
                cells_[idx] = CellState::UNKNOWN;
                occ_[idx] = 0;
                free_streak_[idx] = 0;
                free_last_frame_[idx] = frame_id_;
            }
        });
    }

    // 3. 机器人足迹覆盖：仅把车底盲区 UNKNOWN 标为 FREE，绝不擦除已确认 OCCUPIED。
    if (frame.has_robot_pos && frame.robot_footprint_radius > 0.0 && frame.robot_pos.allFinite()) {
        const double r = frame.robot_footprint_radius;
        if (!(frame.robot_pos.x() + r < geom_.min_boundary.x() || frame.robot_pos.y() + r < geom_.min_boundary.y()
              || frame.robot_pos.x() - r >= geom_.max_boundary.x()
              || frame.robot_pos.y() - r >= geom_.max_boundary.y()))
        {
            const double r2 = r * r;
            const Eigen::Vector2i max_idx(geom_.voxel_num.x() - 1, geom_.voxel_num.y() - 1);
            Eigen::Vector2i lo = geom_.posToIndex(frame.robot_pos - Eigen::Vector2d(r, r));
            Eigen::Vector2i hi = geom_.posToIndex(frame.robot_pos + Eigen::Vector2d(r, r));
            if (!(hi.x() < 0 || hi.y() < 0 || lo.x() > max_idx.x() || lo.y() > max_idx.y())) {
                lo = lo.cwiseMax(Eigen::Vector2i::Zero());
                hi = hi.cwiseMin(max_idx);
                for (int x = lo.x(); x <= hi.x(); ++x) {
                    for (int y = lo.y(); y <= hi.y(); ++y) {
                        const Eigen::Vector2d center = geom_.indexToPos(Eigen::Vector2i(x, y));
                        if ((center - frame.robot_pos).squaredNorm() <= r2) {
                            const size_t idx = static_cast<size_t>(geom_.toAddress(x, y));
                            if (cells_[idx] == CellState::UNKNOWN) {
                                cells_[idx] = CellState::FREE;
                                occ_[idx] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
}

// ==================== DynamicObstacleLayer ====================

auto DynamicObstacleLayer::resize(
    const grid_map::GridGeometry& new_geom,
    const Eigen::Vector2i& shift,
    bool preserve_data
) -> void {
    if (!new_geom.isValid()) {
        geom_ = grid_map::GridGeometry {};
        occ_.clear();
        return;
    }
    if (preserve_data && geom_.isValid()) {
        occ_ = shiftGridBuffer<uint8_t>(
            occ_,
            geom_.voxel_num.x(),
            geom_.voxel_num.y(),
            new_geom.voxel_num.x(),
            new_geom.voxel_num.y(),
            shift,
            0
        );
    } else {
        occ_.assign(new_geom.cellCount(), 0);
    }
    geom_ = new_geom;
}

auto DynamicObstacleLayer::reset() -> void {
    std::fill(occ_.begin(), occ_.end(), 0);
}

auto DynamicObstacleLayer::update(const LocalTerrainFrame& frame) -> void {
    if (!geom_.isValid()) {
        return;
    }
    std::fill(occ_.begin(), occ_.end(), 0);
    forEachTerrainCell(geom_, frame.obstacle_cells, frame.terrain_resolution, [&](int x, int y) {
        occ_[static_cast<size_t>(geom_.toAddress(x, y))] = 1;
    });
}

// ==================== SemanticsLayer ====================

auto SemanticsLayer::computeAabb(
    const std::vector<Eigen::Vector2d>& polygon,
    Eigen::Vector2d& min_p,
    Eigen::Vector2d& max_p
) -> void {
    min_p = polygon.front();
    max_p = polygon.front();
    for (const auto& pt: polygon) {
        min_p.x() = std::min(min_p.x(), pt.x());
        min_p.y() = std::min(min_p.y(), pt.y());
        max_p.x() = std::max(max_p.x(), pt.x());
        max_p.y() = std::max(max_p.y(), pt.y());
    }
}

auto SemanticsLayer::pointInPolygon(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& polygon) -> bool {
    bool inside = false;
    const int n = static_cast<int>(polygon.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Eigen::Vector2d& pi = polygon[static_cast<size_t>(i)];
        const Eigen::Vector2d& pj = polygon[static_cast<size_t>(j)];
        const bool intersect = ((pi.y() > p.y()) != (pj.y() > p.y()))
            && (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x());
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

auto SemanticsLayer::polygonsEqual(const std::vector<Eigen::Vector2d>& a, const std::vector<Eigen::Vector2d>& b)
    -> bool {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if ((a[i] - b[i]).squaredNorm() > 1e-18) {
            return false;
        }
    }
    return true;
}

auto SemanticsLayer::rasterizeEntry(
    const std::vector<Eigen::Vector2d>& polygon,
    const Eigen::Vector2d& min_p,
    const Eigen::Vector2d& max_p,
    uint8_t value,
    const Eigen::Vector2i& clip_lo,
    const Eigen::Vector2i& clip_hi
) -> bool {
    if (!geom_.isValid() || polygon.size() < 3) {
        return false;
    }
    if (max_p.x() < geom_.min_boundary.x() || max_p.y() < geom_.min_boundary.y() || min_p.x() >= geom_.max_boundary.x()
        || min_p.y() >= geom_.max_boundary.y())
    {
        return false;
    }
    const Eigen::Vector2d clamped_min = min_p.cwiseMax(geom_.min_boundary);
    const Eigen::Vector2d clamped_max = max_p.cwiseMin(geom_.max_boundary);
    const Eigen::Vector2i lo = (geom_.posToIndex(clamped_min) - Eigen::Vector2i(1, 1)).cwiseMax(clip_lo);
    const Eigen::Vector2i hi = (geom_.posToIndex(clamped_max) + Eigen::Vector2i(1, 1)).cwiseMin(clip_hi);
    if (lo.x() > hi.x() || lo.y() > hi.y()) {
        return false;
    }

    bool changed = false;
    for (int x = lo.x(); x <= hi.x(); ++x) {
        for (int y = lo.y(); y <= hi.y(); ++y) {
            const Eigen::Vector2d pos = geom_.indexToPos(Eigen::Vector2i(x, y));
            if (pointInPolygon(pos, polygon)) {
                const size_t idx = static_cast<size_t>(geom_.toAddress(x, y));
                if (buffer_[idx] != value) {
                    buffer_[idx] = value;
                    changed = true;
                }
            }
        }
    }
    return changed;
}

auto SemanticsLayer::resize(
    const grid_map::GridGeometry& new_geom,
    const Eigen::Vector2i& /*shift*/,
    bool /*preserve_data*/
) -> void {
    geom_ = new_geom;
    if (!geom_.isValid()) {
        buffer_.clear();
        ++version_;
        return;
    }
    buffer_.assign(geom_.cellCount(), static_cast<uint8_t>(grid_map::GridMap::Semantics::NONE));
    const Eigen::Vector2i clip_lo = Eigen::Vector2i::Zero();
    const Eigen::Vector2i clip_hi(geom_.voxel_num.x() - 1, geom_.voxel_num.y() - 1);
    for (const auto& entry: polygons_) {
        rasterizeEntry(
            entry.polygon,
            entry.min_p,
            entry.max_p,
            static_cast<uint8_t>(entry.semantics),
            clip_lo,
            clip_hi
        );
    }
    ++version_;
}

auto SemanticsLayer::reset() -> void {
    polygons_.clear();
    std::fill(buffer_.begin(), buffer_.end(), static_cast<uint8_t>(grid_map::GridMap::Semantics::NONE));
    ++version_;
}

auto SemanticsLayer::setPolygon(const std::vector<Eigen::Vector2d>& polygon, grid_map::GridMap::Semantics semantics)
    -> bool {
    if (polygon.size() < 3) {
        return false;
    }
    for (const auto& pt: polygon) {
        if (!pt.allFinite()) {
            return false;
        }
    }

    Eigen::Vector2d min_p = Eigen::Vector2d::Zero();
    Eigen::Vector2d max_p = Eigen::Vector2d::Zero();
    computeAabb(polygon, min_p, max_p);

    // 对世界系多边形列表去重维护：写 NONE 移除对应记录，非 NONE 原地更新或追加。
    auto it = std::find_if(polygons_.begin(), polygons_.end(), [&](const PolygonEntry& e) {
        return polygonsEqual(e.polygon, polygon);
    });
    if (semantics == grid_map::GridMap::Semantics::NONE) {
        if (it != polygons_.end()) {
            polygons_.erase(it);
        }
    } else if (it != polygons_.end()) {
        it->semantics = semantics;
    } else {
        polygons_.push_back(PolygonEntry {polygon, semantics, min_p, max_p});
    }

    if (!geom_.isValid()) {
        return false;
    }

    if (max_p.x() < geom_.min_boundary.x() || max_p.y() < geom_.min_boundary.y() || min_p.x() >= geom_.max_boundary.x()
        || min_p.y() >= geom_.max_boundary.y())
    {
        return false;
    }
    const Eigen::Vector2i grid_lo = Eigen::Vector2i::Zero();
    const Eigen::Vector2i grid_hi(geom_.voxel_num.x() - 1, geom_.voxel_num.y() - 1);
    const Eigen::Vector2d clamped_min = min_p.cwiseMax(geom_.min_boundary);
    const Eigen::Vector2d clamped_max = max_p.cwiseMin(geom_.max_boundary);
    const Eigen::Vector2i aabb_lo = (geom_.posToIndex(clamped_min) - Eigen::Vector2i(1, 1)).cwiseMax(grid_lo);
    const Eigen::Vector2i aabb_hi = (geom_.posToIndex(clamped_max) + Eigen::Vector2i(1, 1)).cwiseMin(grid_hi);
    if (aabb_lo.x() > aabb_hi.x() || aabb_lo.y() > aabb_hi.y()) {
        return false;
    }

    bool changed = false;
    if (semantics == grid_map::GridMap::Semantics::NONE && !polygons_.empty()) {
        // 清除多边形且仍存在其他多边形时，先记录 AABB 旧值再在局部重栅格重叠多边形，保证交集不被误清。
        const int sx = aabb_hi.x() - aabb_lo.x() + 1;
        const int sy = aabb_hi.y() - aabb_lo.y() + 1;
        std::vector<uint8_t> before(static_cast<size_t>(sx) * static_cast<size_t>(sy));
        for (int dx = 0; dx < sx; ++dx) {
            const size_t src = static_cast<size_t>(aabb_lo.x() + dx) * static_cast<size_t>(geom_.voxel_num.y())
                + static_cast<size_t>(aabb_lo.y());
            std::copy_n(buffer_.data() + src, static_cast<size_t>(sy), before.data() + static_cast<size_t>(dx * sy));
        }

        rasterizeEntry(polygon, min_p, max_p, static_cast<uint8_t>(semantics), aabb_lo, aabb_hi);
        for (const auto& entry: polygons_) {
            if (entry.max_p.x() < min_p.x() || entry.min_p.x() > max_p.x() || entry.max_p.y() < min_p.y()
                || entry.min_p.y() > max_p.y())
            {
                continue;
            }
            rasterizeEntry(
                entry.polygon,
                entry.min_p,
                entry.max_p,
                static_cast<uint8_t>(entry.semantics),
                aabb_lo,
                aabb_hi
            );
        }

        for (int dx = 0; dx < sx && !changed; ++dx) {
            const size_t src = static_cast<size_t>(aabb_lo.x() + dx) * static_cast<size_t>(geom_.voxel_num.y())
                + static_cast<size_t>(aabb_lo.y());
            for (int dy = 0; dy < sy; ++dy) {
                if (buffer_[src + static_cast<size_t>(dy)] != before[static_cast<size_t>(dx * sy + dy)]) {
                    changed = true;
                    break;
                }
            }
        }
    } else {
        changed = rasterizeEntry(polygon, min_p, max_p, static_cast<uint8_t>(semantics), aabb_lo, aabb_hi);
    }

    if (changed) {
        ++version_;
    }
    return changed;
}

} // namespace ma_map
