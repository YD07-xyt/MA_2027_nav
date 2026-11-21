#include "map/layered/layered_map.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ma_map {

namespace {

    auto roundUpToChunk(int need, int chunk_cells) -> int {
        if (need <= 0) {
            return 0;
        }
        const int c = std::max(1, chunk_cells);
        return ((need + c - 1) / c) * c;
    }

} // namespace

LayeredMap::LayeredMap(const Config& config): config_(config) {
    if (!std::isfinite(config_.resolution) || config_.resolution <= 0.0) {
        config_.resolution = 0.05;
    }
    if (!std::isfinite(config_.expand_chunk_m) || config_.expand_chunk_m <= 0.0) {
        config_.expand_chunk_m = 4.0;
    }
    if (!std::isfinite(config_.max_map_size_x) || config_.max_map_size_x <= 0.0) {
        config_.max_map_size_x = 80.0;
    }
    if (!std::isfinite(config_.max_map_size_y) || config_.max_map_size_y <= 0.0) {
        config_.max_map_size_y = 80.0;
    }
    if (config_.save_margin_cells < 0) {
        config_.save_margin_cells = 0;
    }

    geometry_ = grid_map::GridGeometry::fromBounds(
        config_.x_left,
        config_.x_right,
        config_.y_left,
        config_.y_right,
        config_.resolution
    );
    mode_ = MapMode::MAPPING;
    oob_policy_ = grid_map::OutOfBoundsPolicy::FREE;

    static_layer_.resize(geometry_, Eigen::Vector2i::Zero(), false);
    persistent_layer_.resize(geometry_, Eigen::Vector2i::Zero(), false);
    dynamic_layer_.resize(geometry_, Eigen::Vector2i::Zero(), false);
    semantics_layer_.resize(geometry_, Eigen::Vector2i::Zero(), false);

    composed_occ_.assign(geometry_.cellCount(), 0);
    snapshot_pool_ = std::make_shared<SnapshotPool>(geometry_, oob_policy_);

    auto initial_slot = snapshot_pool_->acquire();
    initial_slot.map->setOutOfBoundsPolicy(oob_policy_);
    initial_slot.map->overwriteOccAndUpdateEsdf(composed_occ_, &esdf_workspace_);
    initial_slot.map->copySemanticsFrom(semantics_layer_.data());
    initial_slot.semantics_version = semantics_layer_.version();
    publishSnapshot(snapshot_pool_->makeSnapshot(std::move(initial_slot)));
}

auto LayeredMap::setMode(MapMode mode) -> void {
    std::lock_guard<std::mutex> lock(back_mtx_);
    if (mode_ == mode) {
        return;
    }
    mode_ = mode;
    oob_policy_ =
        (mode_ == MapMode::NAVIGATION) ? grid_map::OutOfBoundsPolicy::OCCUPIED : grid_map::OutOfBoundsPolicy::FREE;
    if (snapshot_pool_) {
        snapshot_pool_->setDefaultOutOfBoundsPolicy(oob_policy_);
    }
    if (mode_ == MapMode::NAVIGATION) {
        dynamic_layer_.reset();
    }
    composeOccupancyLocked();
    publishComposedLocked(/*force_publish=*/true);
}

auto LayeredMap::mode() const -> MapMode {
    std::lock_guard<std::mutex> lock(back_mtx_);
    return mode_;
}

auto LayeredMap::updateFromPerception(const LocalTerrainFrame& frame) -> void {
    std::lock_guard<std::mutex> lock(back_mtx_);
    if (!geometry_.isValid()) {
        return;
    }

    bool expanded = false;
    if (mode_ == MapMode::MAPPING) {
        Eigen::Vector2d req_min(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        Eigen::Vector2d req_max(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
        bool has_any = false;

        auto includeBox = [&](double x_lo, double y_lo, double x_hi, double y_hi) {
            if (!std::isfinite(x_lo) || !std::isfinite(y_lo) || !std::isfinite(x_hi) || !std::isfinite(y_hi)) {
                return;
            }
            req_min.x() = std::min(req_min.x(), x_lo);
            req_min.y() = std::min(req_min.y(), y_lo);
            req_max.x() = std::max(req_max.x(), x_hi);
            req_max.y() = std::max(req_max.y(), y_hi);
            has_any = true;
        };

        if (frame.has_robot_pos && frame.robot_pos.allFinite()) {
            const double r = std::max(0.0, frame.robot_footprint_radius);
            includeBox(
                frame.robot_pos.x() - r,
                frame.robot_pos.y() - r,
                frame.robot_pos.x() + r,
                frame.robot_pos.y() + r
            );
        }
        const double half_cell = std::max(0.0, 0.5 * frame.terrain_resolution);
        for (const auto& c: frame.obstacle_cells) {
            includeBox(c.x() - half_cell, c.y() - half_cell, c.x() + half_cell, c.y() + half_cell);
        }
        for (const auto& c: frame.free_cells) {
            includeBox(c.x() - half_cell, c.y() - half_cell, c.x() + half_cell, c.y() + half_cell);
        }

        if (has_any) {
            const grid_map::GridGeometry prev_geom = geometry_;
            expandToCoverBoxLocked(req_min, req_max, /*strict_cover=*/false);
            expanded = (geometry_ != prev_geom);
        }
        persistent_layer_.update(frame);
    } else {
        dynamic_layer_.update(frame);
    }

    composeOccupancyLocked();
    publishComposedLocked(/*force_publish=*/expanded);
}

auto LayeredMap::ensureCovers(const Eigen::Vector2d& world_pt) -> bool {
    if (!world_pt.allFinite()) {
        return false;
    }
    std::lock_guard<std::mutex> lock(back_mtx_);
    if (!geometry_.isValid()) {
        return false;
    }
    if (geometry_.isInside(world_pt)) {
        return true;
    }
    if (mode_ != MapMode::MAPPING) {
        return false;
    }
    const grid_map::GridGeometry prev_geom = geometry_;
    if (!expandToCoverBoxLocked(world_pt, world_pt, /*strict_cover=*/true)) {
        return false;
    }
    if (geometry_ != prev_geom) {
        composeOccupancyLocked();
        publishComposedLocked(/*force_publish=*/true);
    }
    return geometry_.isInside(world_pt);
}

auto LayeredMap::loadStaticMap(std::string_view map_path) -> tl::expected<LoadReport, std::string> {
    LoadOptions opts;
    opts.unknown_as_occupied = config_.unknown_as_occupied;
    auto loaded = MapIO::load(map_path, opts);
    if (!loaded) {
        return tl::make_unexpected(loaded.error());
    }

    LoadReport report;
    report.geometry = loaded->geometry;
    report.occupied_count = loaded->occupied_count;
    report.free_count = loaded->free_count;
    report.unknown_count = loaded->unknown_count;
    report.has_terrain_prior = loaded->terrain_prior.has_value();
    report.terrain_prior_path = loaded->terrain_prior_path;

    {
        std::lock_guard<std::mutex> lock(back_mtx_);
        geometry_ = loaded->geometry;
        max_size_warned_ = false;
        static_layer_.load(*loaded);
        persistent_layer_.resize(geometry_, Eigen::Vector2i::Zero(), false);
        dynamic_layer_.resize(geometry_, Eigen::Vector2i::Zero(), false);
        semantics_layer_.resize(geometry_, Eigen::Vector2i::Zero(), false);

        composed_occ_.assign(geometry_.cellCount(), 0);
        snapshot_pool_ = std::make_shared<SnapshotPool>(geometry_, oob_policy_);
        composeOccupancyLocked();
        publishComposedLocked(/*force_publish=*/true);
        rebuildStaticEsdfLocked();
    }

    return report;
}

auto LayeredMap::saveExploredMap(std::string_view output_path, const std::optional<SaveOptions>& options) const
    -> tl::expected<SaveReport, std::string> {
    grid_map::GridGeometry save_geom;
    std::vector<CellState> save_cells;
    std::shared_ptr<const StaticTerrainPrior> save_prior;
    SaveOptions effective_opts;

    {
        std::lock_guard<std::mutex> lock(back_mtx_);
        if (!geometry_.isValid()) {
            return tl::make_unexpected("Cannot save map: invalid map geometry");
        }
        save_geom = geometry_;
        const size_t n = geometry_.cellCount();
        save_cells.assign(n, CellState::UNKNOWN);

        if (mode_ == MapMode::MAPPING) {
            const auto pers = persistent_layer_.cells();
            std::copy(pers.begin(), pers.end(), save_cells.begin());
            if (static_layer_.hasLoadedCells()) {
                const auto st_cells = static_layer_.cells();
                const auto st_occ = static_layer_.occupancy();
                for (size_t i = 0; i < n; ++i) {
                    if (save_cells[i] == CellState::OCCUPIED || st_occ[i] != 0) {
                        save_cells[i] = CellState::OCCUPIED;
                    } else if (save_cells[i] == CellState::UNKNOWN && st_cells[i] != CellState::UNKNOWN) {
                        save_cells[i] = st_cells[i];
                    }
                }
            }
        } else {
            if (static_layer_.hasLoadedCells()) {
                const auto st_cells = static_layer_.cells();
                const auto dyn_occ = dynamic_layer_.occupancy();
                std::copy(st_cells.begin(), st_cells.end(), save_cells.begin());
                for (size_t i = 0; i < n; ++i) {
                    if (dyn_occ[i] != 0) {
                        save_cells[i] = CellState::OCCUPIED;
                    }
                }
                auto st_prior = static_layer_.terrainPrior();
                if (st_prior && st_prior->isValid() && st_prior->geometry == save_geom) {
                    save_prior = std::move(st_prior);
                }
            } else {
                for (size_t i = 0; i < n; ++i) {
                    save_cells[i] = composed_occ_[i] != 0 ? CellState::OCCUPIED : CellState::FREE;
                }
            }
        }

        if (options.has_value()) {
            effective_opts = *options;
        } else {
            effective_opts = defaultSaveOptions();
        }
    }

    return MapIO::save(output_path, save_geom, save_cells, effective_opts, save_prior.get());
}

auto LayeredMap::setGlobalOccupancy(const grid_map::RowMatrixXi& global_occ) -> bool {
    std::lock_guard<std::mutex> lock(back_mtx_);
    bool changed = true;
    if (!static_layer_.setOccupancyMatrix(global_occ, &changed)) {
        return false;
    }
    if (!changed) {
        return true;
    }
    static_esdf_dirty_.store(true, std::memory_order_release);
    composeOccupancyLocked();
    publishComposedLocked(/*force_publish=*/false);
    return true;
}

auto LayeredMap::setSemanticsPolygon(
    const std::vector<Eigen::Vector2d>& polygon,
    grid_map::GridMap::Semantics semantics
) -> bool {
    std::lock_guard<std::mutex> lock(back_mtx_);
    if (!semantics_layer_.setPolygon(polygon, semantics)) {
        return false;
    }

    auto slot = snapshot_pool_->acquire();
    auto current_front = getSnapshot();
    if (current_front && current_front->geometry() == geometry_ && slot.map->copyOccAndEsdfFrom(*current_front)) {
        slot.map->setOutOfBoundsPolicy(oob_policy_);
    } else {
        slot.map->setOutOfBoundsPolicy(oob_policy_);
        slot.map->overwriteOccAndUpdateEsdf(composed_occ_, &esdf_workspace_);
    }
    current_front.reset();

    slot.map->copySemanticsFrom(semantics_layer_.data());
    slot.semantics_version = semantics_layer_.version();

    publishSnapshot(snapshot_pool_->makeSnapshot(std::move(slot)));
    return true;
}

auto LayeredMap::getSnapshot() const -> grid_map::GridMap::ConstPtr {
    std::lock_guard<std::mutex> lock(front_mtx_);
    return front_snapshot_;
}

auto LayeredMap::getStaticNavContext() const -> StaticNavContext {
    if (static_esdf_dirty_.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> back_lock(back_mtx_);
        if (static_esdf_dirty_.load(std::memory_order_relaxed)) {
            rebuildStaticEsdfLocked();
        }
        // 在持有 back_mtx_ 的同一临界区内捕获三个指针，避免 rebuild 释放 back_mtx_ 后、
        // 重新获取 front_mtx_ 之前被写线程插入发布新快照导致上下文错位。
        // 锁序保持 back_mtx_ -> front_mtx_。
        std::lock_guard<std::mutex> front_lock(front_mtx_);
        StaticNavContext ctx;
        ctx.static_esdf = front_static_esdf_;
        ctx.terrain_prior = front_terrain_prior_;
        ctx.current_snapshot = front_snapshot_;
        return ctx;
    }
    std::lock_guard<std::mutex> lock(front_mtx_);
    StaticNavContext ctx;
    ctx.static_esdf = front_static_esdf_;
    ctx.terrain_prior = front_terrain_prior_;
    ctx.current_snapshot = front_snapshot_;
    return ctx;
}

auto LayeredMap::getStaticTerrainPrior(uint64_t* revision) const -> std::shared_ptr<const StaticTerrainPrior> {
    std::lock_guard<std::mutex> lock(front_mtx_);
    if (revision != nullptr) {
        *revision = front_terrain_prior_rev_;
    }
    return front_terrain_prior_;
}

auto LayeredMap::defaultSaveOptions() const -> SaveOptions {
    SaveOptions opts;
    opts.crop_to_explored = true;
    opts.margin_cells = config_.save_margin_cells;
    opts.cleanup_free_pinholes = true;
    opts.max_pinhole_cells = 4;
    opts.save_terrain_prior = true;
    return opts;
}

auto LayeredMap::getOccupancyMatrix() const -> grid_map::RowMatrixXi {
    std::lock_guard<std::mutex> lock(back_mtx_);
    if (!geometry_.isValid()) {
        return {};
    }
    grid_map::RowMatrixXi out(geometry_.voxel_num.x(), geometry_.voxel_num.y());
    int* dst = out.data();
    const size_t n = composed_occ_.size();
    for (size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<int>(composed_occ_[i]);
    }
    return out;
}

auto LayeredMap::getStaticOccupancyMatrix() const -> grid_map::RowMatrixXi {
    std::lock_guard<std::mutex> lock(back_mtx_);
    return static_layer_.toOccupancyMatrix();
}

auto LayeredMap::expandToCoverBoxLocked(
    const Eigen::Vector2d& req_min,
    const Eigen::Vector2d& req_max,
    bool strict_cover
) -> bool {
    const double res = geometry_.resolution;
    const int max_voxels_x =
        std::max(geometry_.voxel_num.x(), static_cast<int>(std::floor(config_.max_map_size_x / res + 1e-9)));
    const int max_voxels_y =
        std::max(geometry_.voxel_num.y(), static_cast<int>(std::floor(config_.max_map_size_y / res + 1e-9)));
    const int budget_x = std::max(0, max_voxels_x - geometry_.voxel_num.x());
    const int budget_y = std::max(0, max_voxels_y - geometry_.voxel_num.y());

    auto calcNeedNeg = [&](double val, double min_bound, int cap) -> int {
        if (val >= min_bound) {
            return 0;
        }
        const double raw = std::ceil((min_bound - val) * geometry_.resolution_inv - 1e-9);
        if (!std::isfinite(raw) || raw > static_cast<double>(cap)) {
            return cap;
        }
        int need = std::max(1, static_cast<int>(raw));
        if (val < min_bound - static_cast<double>(need) * res && need < cap) {
            ++need;
        }
        return need;
    };
    auto calcNeedPos = [&](double val, double max_bound, int cap) -> int {
        if (val < max_bound) {
            return 0;
        }
        const double raw = std::floor((val - max_bound) * geometry_.resolution_inv) + 1.0;
        if (!std::isfinite(raw) || raw > static_cast<double>(cap)) {
            return cap;
        }
        int need = std::max(1, static_cast<int>(raw));
        if (val >= max_bound + static_cast<double>(need) * res && need < cap) {
            ++need;
        }
        return need;
    };

    const int need_x_neg = calcNeedNeg(req_min.x(), geometry_.min_boundary.x(), budget_x + 1);
    const int need_x_pos = calcNeedPos(req_max.x(), geometry_.max_boundary.x(), budget_x + 1);
    const int need_y_neg = calcNeedNeg(req_min.y(), geometry_.min_boundary.y(), budget_y + 1);
    const int need_y_pos = calcNeedPos(req_max.y(), geometry_.max_boundary.y(), budget_y + 1);

    if (need_x_neg == 0 && need_x_pos == 0 && need_y_neg == 0 && need_y_pos == 0) {
        return true;
    }

    const int chunk_cells = std::max(1, static_cast<int>(std::ceil(config_.expand_chunk_m / res - 1e-9)));

    auto planAxis = [&](int need_neg, int need_pos, int budget, int& add_neg, int& add_pos) -> bool {
        if (strict_cover && need_neg + need_pos > budget) {
            return false;
        }
        int want_neg = roundUpToChunk(need_neg, chunk_cells);
        int want_pos = roundUpToChunk(need_pos, chunk_cells);
        if (want_neg + want_pos <= budget) {
            add_neg = want_neg;
            add_pos = want_pos;
            return true;
        }
        if (strict_cover) {
            const int extra_budget = budget - (need_neg + need_pos);
            const int extra_neg = std::min(want_neg - need_neg, extra_budget);
            const int extra_pos = std::min(want_pos - need_pos, extra_budget - extra_neg);
            add_neg = need_neg + extra_neg;
            add_pos = need_pos + extra_pos;
            return true;
        }
        // 非严格模式（感知扩图）：夹紧到可用配额内，优先覆盖双侧需求。
        if (want_neg > 0 && want_pos > 0) {
            add_neg = std::min(want_neg, budget / 2);
            add_pos = std::min(want_pos, budget - add_neg);
            const int rem = budget - add_neg - add_pos;
            if (rem > 0 && add_neg < want_neg) {
                add_neg += std::min(rem, want_neg - add_neg);
            }
        } else {
            add_neg = std::min(want_neg, budget);
            add_pos = std::min(want_pos, budget - add_neg);
        }
        return true;
    };

    int dx_neg = 0;
    int dx_pos = 0;
    int dy_neg = 0;
    int dy_pos = 0;
    if (!planAxis(need_x_neg, need_x_pos, budget_x, dx_neg, dx_pos)
        || !planAxis(need_y_neg, need_y_pos, budget_y, dy_neg, dy_pos))
    {
        logger::MaMap->warn(
            "Map expansion rejected: target box [{:.2f},{:.2f}]x[{:.2f},{:.2f}] exceeds max_map_size [{:.1f},{:.1f}]",
            req_min.x(),
            req_max.x(),
            req_min.y(),
            req_max.y(),
            config_.max_map_size_x,
            config_.max_map_size_y
        );
        return false;
    }

    if (dx_neg == 0 && dx_pos == 0 && dy_neg == 0 && dy_pos == 0) {
        if (!max_size_warned_) {
            logger::MaMap->warn(
                "Map reached max_map_size ({:.1f}x{:.1f}m); further perception outside bounds will be clipped",
                geometry_.map_size.x(),
                geometry_.map_size.y()
            );
            max_size_warned_ = true;
        }
        return !strict_cover;
    }

    expandByCellsLocked(dx_neg, dx_pos, dy_neg, dy_pos);
    return true;
}

auto LayeredMap::expandByCellsLocked(int dx_neg, int dx_pos, int dy_neg, int dy_pos) -> void {
    const grid_map::GridGeometry old_geom = geometry_;
    const double res = old_geom.resolution;
    const Eigen::Vector2d new_origin(
        old_geom.origin.x() - static_cast<double>(dx_neg) * res,
        old_geom.origin.y() - static_cast<double>(dy_neg) * res
    );
    const Eigen::Vector2i new_voxels(
        old_geom.voxel_num.x() + dx_neg + dx_pos,
        old_geom.voxel_num.y() + dy_neg + dy_pos
    );
    const grid_map::GridGeometry new_geom = grid_map::GridGeometry::fromOriginAndSize(new_origin, new_voxels, res);
    const Eigen::Vector2i shift(dx_neg, dy_neg);

    static_layer_.resize(new_geom, shift, true);
    persistent_layer_.resize(new_geom, shift, true);
    dynamic_layer_.resize(new_geom, shift, true);
    semantics_layer_.resize(new_geom, shift, true);

    geometry_ = new_geom;
    composed_occ_.assign(geometry_.cellCount(), 0);
    snapshot_pool_ = std::make_shared<SnapshotPool>(geometry_, oob_policy_);
    if (static_layer_.hasLoadedCells()) {
        static_esdf_dirty_.store(true, std::memory_order_release);
    }

    logger::MaMap->info(
        "Expanded 2D map: {}x{} -> {}x{} (origin ({:.2f},{:.2f}) -> ({:.2f},{:.2f}), size {:.1f}x{:.1f}m)",
        old_geom.voxel_num.x(),
        old_geom.voxel_num.y(),
        new_geom.voxel_num.x(),
        new_geom.voxel_num.y(),
        old_geom.origin.x(),
        old_geom.origin.y(),
        new_geom.origin.x(),
        new_geom.origin.y(),
        new_geom.map_size.x(),
        new_geom.map_size.y()
    );
}

auto LayeredMap::composeOccupancyLocked() -> void {
    const size_t n = geometry_.cellCount();
    composed_occ_.resize(n);
    const auto st_occ = static_layer_.occupancy();
    const auto active_occ = (mode_ == MapMode::MAPPING) ? persistent_layer_.occupancy() : dynamic_layer_.occupancy();
    for (size_t i = 0; i < n; ++i) {
        composed_occ_[i] = static_cast<uint8_t>((st_occ[i] | active_occ[i]) != 0 ? 1 : 0);
    }
}

auto LayeredMap::publishComposedLocked(bool force_publish) -> void {
    auto current_front = getSnapshot();
    const bool same_geom = current_front && current_front->geometry() == geometry_;
    const bool same_occ = same_geom && current_front->equalsOccupancy(composed_occ_);
    const bool same_oob = same_geom && current_front->outOfBoundsPolicy() == oob_policy_;

    if (!force_publish && same_occ && same_oob) {
        return;
    }

    auto slot = snapshot_pool_->acquire();
    // 几何与占据完全一致时（无论 oob 是否变化、是否 force_publish）直接复用已算好的 ESDF，
    // 避免对相同 composed_occ_ 反复做全图 2D ESDF 重建；几何或占据变化时才必须重建。
    if (same_geom && same_occ && slot.map->copyOccAndEsdfFrom(*current_front)) {
        if (!same_oob) {
            slot.map->setOutOfBoundsPolicy(oob_policy_);
        }
    } else {
        current_front.reset();
        slot.map->setOutOfBoundsPolicy(oob_policy_);
        if (!slot.map->overwriteOccAndUpdateEsdf(composed_occ_, &esdf_workspace_)) {
            snapshot_pool_->release(std::move(slot));
            return;
        }
    }
    current_front.reset();

    if (slot.semantics_version != semantics_layer_.version()) {
        slot.map->copySemanticsFrom(semantics_layer_.data());
        slot.semantics_version = semantics_layer_.version();
    }
    publishSnapshot(snapshot_pool_->makeSnapshot(std::move(slot)));
}

auto LayeredMap::publishSnapshot(grid_map::GridMap::ConstPtr next_front) -> void {
    grid_map::GridMap::ConstPtr old_front;
    {
        std::lock_guard<std::mutex> lock(front_mtx_);
        old_front = std::move(front_snapshot_);
        front_snapshot_ = std::move(next_front);
    }
}

auto LayeredMap::rebuildStaticEsdfLocked() const -> void {
    grid_map::GridMap::ConstPtr next_static;
    std::shared_ptr<const StaticTerrainPrior> next_prior;
    if (static_layer_.hasLoadedCells() && geometry_.isValid()) {
        auto gm = std::make_shared<grid_map::GridMap>();
        gm->init(geometry_, grid_map::OutOfBoundsPolicy::OCCUPIED);
        gm->overwriteOccAndUpdateEsdf(static_layer_.occupancy(), &esdf_workspace_);
        next_static = std::move(gm);
        next_prior = static_layer_.terrainPrior();
    }

    grid_map::GridMap::ConstPtr old_static;
    std::shared_ptr<const StaticTerrainPrior> old_prior;
    {
        std::lock_guard<std::mutex> lock(front_mtx_);
        const bool prior_changed = (front_terrain_prior_.get() != next_prior.get());
        old_static = std::move(front_static_esdf_);
        old_prior = std::move(front_terrain_prior_);
        front_static_esdf_ = std::move(next_static);
        front_terrain_prior_ = std::move(next_prior);
        if (prior_changed) {
            ++front_terrain_prior_rev_;
        }
    }
    static_esdf_dirty_.store(false, std::memory_order_release);
}

} // namespace ma_map
