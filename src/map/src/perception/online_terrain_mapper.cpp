#include "map/perception/online_terrain_mapper.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ma_map {

namespace {

    constexpr int32_t kChunkSize = 16;
    constexpr int kWallBleedMinClears = 3;
    constexpr float kWallBleedClearRatio = 0.8f;
    constexpr int kWallBleedMinNeighborHits = 10;
    constexpr float kWallBleedNeighborClearRatio = 0.25f;
    constexpr int32_t kWallBleedMinNeighborVertSpanMm = 65; // 邻格垂向延续 >= 6.5 cm 才认定为竖墙，保护斜坡/平地边界
    constexpr int kPreviewEmitHalo = 6; // 预览边界内缩 halo（格），与 overhang_ground_radius=6 对齐
    constexpr int32_t kZMinQuantileStepDownMm = 9; // ~10% 分位下移步长 (mm)
    constexpr int32_t kZMinQuantileStepUpMm = 1; // ~10% 分位上移步长 (mm)

    inline auto floorDiv16(int32_t v) -> int32_t {
        return (v >= 0) ? (v / kChunkSize) : -((-v + kChunkSize - 1) / kChunkSize);
    }

    inline auto mod16(int32_t v) -> int32_t {
        return v - floorDiv16(v) * kChunkSize;
    }

    inline auto localVoxelIndex(int32_t lx, int32_t ly, int32_t lz) -> size_t {
        return (static_cast<size_t>(lx) << 8U) | (static_cast<size_t>(ly) << 4U) | static_cast<size_t>(lz);
    }

    inline auto packXY(int32_t vx, int32_t vy) -> uint64_t {
        return (static_cast<uint64_t>(static_cast<uint32_t>(vx)) << 32U)
            | static_cast<uint64_t>(static_cast<uint32_t>(vy));
    }

    inline auto unpackX(uint64_t key) -> int32_t {
        return static_cast<int32_t>(key >> 32U);
    }

    inline auto unpackY(uint64_t key) -> int32_t {
        return static_cast<int32_t>(key & 0xffffffffULL);
    }

    inline auto toMillimeters(double z) -> int16_t {
        const long mm = std::lround(z * 1000.0);
        return static_cast<int16_t>(std::clamp(mm, -32000L, 32000L));
    }

} // namespace

OnlineTerrainMapper::OnlineTerrainMapper() {
    setConfig(Config {});
}

OnlineTerrainMapper::OnlineTerrainMapper(const Config& cfg) {
    setConfig(cfg);
}

auto OnlineTerrainMapper::setConfig(const Config& cfg) -> void {
    std::lock_guard<std::mutex> lock(mtx_);
    cfg_ = cfg;
    if (!std::isfinite(cfg_.resolution) || cfg_.resolution <= 0.0) {
        cfg_.resolution = 0.05;
    }
    inv_res_ = 1.0 / cfg_.resolution;
    builder_.setConfig(cfg_.terrain);
    if (cfg_.max_clear_rays_per_frame > 0) {
        clear_targets_scratch_.reserve(std::min<size_t>(static_cast<size_t>(cfg_.max_clear_rays_per_frame), 4096));
    }
}

auto OnlineTerrainMapper::reset() -> void {
    std::lock_guard<std::mutex> lock(mtx_);
    chunks_.clear();
    tile_max_cz_.clear();
    footprint_ground_.clear();
    clear_targets_scratch_.clear();
    erased_cols_scratch_.clear();
    frame_seq_ = 0;
    has_prev_pose_ = false;
    prev_q_base_w_ = Eigen::Quaterniond::Identity();
    tilt_hold_left_ = 0;
    chunk_limit_warned_ = false;
    revision_.fetch_add(1, std::memory_order_release);
}

auto OnlineTerrainMapper::getOrCreateChunkLocked(const ChunkKey& key) -> Chunk* {
    const auto it = chunks_.find(key);
    if (it != chunks_.end()) {
        return it->second.get();
    }
    if (chunks_.size() >= cfg_.max_chunks) {
        if (!chunk_limit_warned_) {
            chunk_limit_warned_ = true;
            logger::MaMap->warn(
                "[OnlineTerrainMapper] Sparse 3D chunk limit reached ({} chunks); refusing new chunks to bound memory.",
                cfg_.max_chunks
            );
        }
        return nullptr;
    }
    auto chunk = std::make_unique<Chunk>();
    Chunk* raw = chunk.get();
    chunks_.emplace(key, std::move(chunk));
    return raw;
}

auto OnlineTerrainMapper::findChunkLocked(const ChunkKey& key) const -> const Chunk* {
    const auto it = chunks_.find(key);
    return (it != chunks_.end()) ? it->second.get() : nullptr;
}

auto OnlineTerrainMapper::findChunkMutableLocked(const ChunkKey& key) -> Chunk* {
    const auto it = chunks_.find(key);
    return (it != chunks_.end()) ? it->second.get() : nullptr;
}

auto OnlineTerrainMapper::isRawStaticOccupied(const VoxelCell& cell) const -> bool {
    return static_cast<int>(cell.hits) >= cfg_.static_min_hits
        && static_cast<float>(cell.clears) <= cfg_.static_clear_ratio * static_cast<float>(cell.hits)
        && cell.z_min_mm <= cell.z_max_mm;
}

auto OnlineTerrainMapper::findVoxelLocked(int32_t vx, int32_t vy, int32_t vz) const -> const VoxelCell* {
    const ChunkKey ck {floorDiv16(vx), floorDiv16(vy), floorDiv16(vz)};
    const Chunk* ch = findChunkLocked(ck);
    if (ch == nullptr) {
        return nullptr;
    }
    return &ch->cells[localVoxelIndex(mod16(vx), mod16(vy), mod16(vz))];
}

auto OnlineTerrainMapper::isWallBleedLocked(
    int32_t vx,
    int32_t vy,
    int32_t vz,
    const VoxelCell& cell,
    const ChunkKey& ck,
    const Chunk& chunk
) const -> bool {
    if (static_cast<int>(cell.clears) < kWallBleedMinClears
        || static_cast<float>(cell.clears) < kWallBleedClearRatio * static_cast<float>(cell.hits))
    {
        return false;
    }

    const int min_nb_hits = std::max(kWallBleedMinNeighborHits, (3 * static_cast<int>(cell.hits) + 1) / 2);
    const int32_t base_x = ck.cx * kChunkSize;
    const int32_t base_y = ck.cy * kChunkSize;
    const int32_t base_z = ck.cz * kChunkSize;

    auto fetchCell = [&](int32_t nx, int32_t ny, int32_t nz) -> const VoxelCell* {
        const int32_t nlx = nx - base_x;
        const int32_t nly = ny - base_y;
        const int32_t nlz = nz - base_z;
        if (nlx >= 0 && nlx < kChunkSize && nly >= 0 && nly < kChunkSize && nlz >= 0 && nlz < kChunkSize) {
            return &chunk.cells[localVoxelIndex(nlx, nly, nlz)];
        }
        return findVoxelLocked(nx, ny, nz);
    };

    constexpr int32_t kDirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& d: kDirs) {
        const int32_t nvx = vx + d[0];
        const int32_t nvy = vy + d[1];
        const VoxelCell* nb = fetchCell(nvx, nvy, vz);
        if (nb == nullptr || static_cast<int>(nb->hits) < min_nb_hits
            || static_cast<float>(nb->clears) > kWallBleedNeighborClearRatio * static_cast<float>(nb->hits)
            || nb->z_min_mm > nb->z_max_mm)
        {
            continue;
        }
        // 要求邻格沿竖直方向延续至少 6.5 cm，确认真实竖墙而非跨体素 Z 边界的平地/斜坡
        const VoxelCell* nb_up = fetchCell(nvx, nvy, vz + 1);
        if (nb_up != nullptr && isRawStaticOccupied(*nb_up)
            && (static_cast<int32_t>(nb_up->z_max_mm) - static_cast<int32_t>(nb->z_min_mm))
                >= kWallBleedMinNeighborVertSpanMm)
        {
            return true;
        }
        const VoxelCell* nb_dn = fetchCell(nvx, nvy, vz - 1);
        if (nb_dn != nullptr && isRawStaticOccupied(*nb_dn)
            && (static_cast<int32_t>(nb->z_max_mm) - static_cast<int32_t>(nb_dn->z_min_mm))
                >= kWallBleedMinNeighborVertSpanMm)
        {
            return true;
        }
    }
    return false;
}

auto OnlineTerrainMapper::isStaticOccupiedLocked(
    int32_t vx,
    int32_t vy,
    int32_t vz,
    const VoxelCell& cell,
    const ChunkKey& ck,
    const Chunk& chunk
) const -> bool {
    if (!isRawStaticOccupied(cell)) {
        return false;
    }
    return !isWallBleedLocked(vx, vy, vz, cell, ck, chunk);
}

auto OnlineTerrainMapper::columnHasStaticOccupiedLocked(int32_t vx, int32_t vy) const -> bool {
    const int32_t cx = floorDiv16(vx);
    const int32_t cy = floorDiv16(vy);
    const auto max_it = tile_max_cz_.find(packXY(cx, cy));
    if (max_it == tile_max_cz_.end()) {
        return false;
    }
    const double min_z_bound = std::min(cfg_.min_z, static_cast<double>(cfg_.terrain.min_ground_height));
    const int32_t min_cz = floorDiv16(static_cast<int32_t>(std::floor(min_z_bound * inv_res_))) - 1;
    const int32_t max_cz = max_it->second;
    const int32_t lx = mod16(vx);
    const int32_t ly = mod16(vy);
    for (int32_t cz = min_cz; cz <= max_cz; ++cz) {
        const ChunkKey ck {cx, cy, cz};
        const Chunk* ch = findChunkLocked(ck);
        if (ch == nullptr) {
            continue;
        }
        const int32_t base_z = cz * kChunkSize;
        for (int32_t lz = 0; lz < kChunkSize; ++lz) {
            const VoxelCell& cell = ch->cells[localVoxelIndex(lx, ly, lz)];
            if (isStaticOccupiedLocked(vx, vy, base_z + lz, cell, ck, *ch)) {
                return true;
            }
        }
    }
    return false;
}

auto OnlineTerrainMapper::castClearRayLocked(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& target,
    uint32_t frame_seq,
    std::vector<uint64_t>* erased_cols
) -> void {
    if (!origin.allFinite() || !target.allFinite() || !std::isfinite(cfg_.clear_range)
        || !std::isfinite(cfg_.clear_stop_margin) || !std::isfinite(cfg_.resolution))
    {
        return;
    }
    const Eigen::Vector3d delta = target - origin;
    const double dist = delta.norm();
    if (dist <= 1e-6) {
        return;
    }
    const double max_dist = std::min(dist - cfg_.clear_stop_margin, cfg_.clear_range);
    const double start_dist = std::max(0.25, cfg_.resolution);
    if (!std::isfinite(max_dist) || max_dist <= start_dist) {
        return;
    }

    const Eigen::Vector3d dir = delta / dist;
    // 任何向下倾斜的射线（含高台顶面浅掠射）在接近落点上方 3 cm 时停止扫空，
    // 避免在首次命中前后误擦除高台/地面薄表面；严格水平射线（dir.z() == 0）仍允许穿透清除动态障碍。
    const bool downward_ray = (dir.z() < -1e-6);
    const double min_clear_z = target.z() + 0.03;
    const double trust_r_sq = cfg_.trust_range * cfg_.trust_range;
    const double chunk_z_size = static_cast<double>(kChunkSize) * cfg_.resolution;

    const Eigen::Vector3d p_start = origin + dir * start_dist;
    int32_t vx = static_cast<int32_t>(std::floor(p_start.x() * inv_res_));
    int32_t vy = static_cast<int32_t>(std::floor(p_start.y() * inv_res_));
    int32_t vz = static_cast<int32_t>(std::floor(p_start.z() * inv_res_));

    const double kInf = std::numeric_limits<double>::infinity();
    const int32_t step_x = (dir.x() > 0.0) ? 1 : ((dir.x() < 0.0) ? -1 : 0);
    const int32_t step_y = (dir.y() > 0.0) ? 1 : ((dir.y() < 0.0) ? -1 : 0);
    const int32_t step_z = (dir.z() > 0.0) ? 1 : ((dir.z() < 0.0) ? -1 : 0);

    const double t_delta_x = (step_x != 0) ? (cfg_.resolution / std::abs(dir.x())) : kInf;
    const double t_delta_y = (step_y != 0) ? (cfg_.resolution / std::abs(dir.y())) : kInf;
    const double t_delta_z = (step_z != 0) ? (cfg_.resolution / std::abs(dir.z())) : kInf;

    auto nextBoundary = [&](int32_t v, int32_t step, double p_coord, double d_coord) -> double {
        if (step == 0) {
            return kInf;
        }
        const double border = static_cast<double>(step > 0 ? (v + 1) : v) * cfg_.resolution;
        return (border - p_coord) / d_coord;
    };

    double t_max_x = nextBoundary(vx, step_x, p_start.x(), dir.x());
    double t_max_y = nextBoundary(vy, step_y, p_start.y(), dir.y());
    double t_max_z = nextBoundary(vz, step_z, p_start.z(), dir.z());

    const double travel_budget = max_dist - start_dist;
    double t_curr = 0.0;

    bool has_cached = false;
    ChunkKey cached_ck {};
    Chunk* cached_chunk = nullptr;

    const int max_steps = static_cast<int>(travel_budget * inv_res_ * 3.0) + 8;
    for (int step = 0; step < max_steps && t_curr <= travel_budget; ++step) {
        const double t_next = std::min({t_max_x, t_max_y, t_max_z, travel_budget});
        const double t_mid = 0.5 * (t_curr + t_next);
        const double z_ray = p_start.z() + dir.z() * t_mid;

        if (downward_ray && z_ray < min_clear_z) {
            break;
        }

        const ChunkKey ck {floorDiv16(vx), floorDiv16(vy), floorDiv16(vz)};
        if (!has_cached || !(ck == cached_ck)) {
            cached_ck = ck;
            cached_chunk = findChunkMutableLocked(ck);
            if (cached_chunk == nullptr
                && static_cast<double>(ck.cz + 1) * chunk_z_size > static_cast<double>(cfg_.terrain.min_ground_height))
            {
                const double rx = p_start.x() + dir.x() * t_mid - origin.x();
                const double ry = p_start.y() + dir.y() * t_mid - origin.y();
                if (rx * rx + ry * ry <= trust_r_sq) {
                    const auto max_it = tile_max_cz_.find(packXY(ck.cx, ck.cy));
                    if (max_it != tile_max_cz_.end() && ck.cz < max_it->second) {
                        cached_chunk = getOrCreateChunkLocked(ck);
                    }
                }
            }
            has_cached = true;
        }

        if (cached_chunk != nullptr) {
            VoxelCell& cell = cached_chunk->cells[localVoxelIndex(mod16(vx), mod16(vy), mod16(vz))];
            if (cell.last_hit_frame != frame_seq && cell.last_clear_frame != frame_seq) {
                bool allow_clear = true;
                if (cell.hits > 0) {
                    const double cell_z_lo = static_cast<double>(cell.z_min_mm) * 0.001;
                    const double cell_z_hi = static_cast<double>(cell.z_max_mm) * 0.001;
                    // 若射线完全从已建表面上方或下方掠过，不误伤表面体素
                    if (z_ray < cell_z_lo - 0.015 || z_ray > cell_z_hi + 0.015) {
                        allow_clear = false;
                    }
                }
                if (allow_clear) {
                    const uint16_t prev_clears = cell.clears;
                    if (cell.clears < std::numeric_limits<uint16_t>::max()) {
                        ++cell.clears;
                    }
                    cell.last_clear_frame = frame_seq;
                    if (erased_cols != nullptr && static_cast<int>(cell.hits) >= cfg_.static_min_hits
                        && cell.z_min_mm <= cell.z_max_mm)
                    {
                        const float max_clr = cfg_.static_clear_ratio * static_cast<float>(cell.hits);
                        const bool crossed_ratio =
                            (static_cast<float>(prev_clears) <= max_clr) && (static_cast<float>(cell.clears) > max_clr);
                        bool crossed_wall_bleed = false;
                        if (!crossed_ratio && static_cast<float>(cell.clears) <= max_clr) {
                            const float wb_thr = kWallBleedClearRatio * static_cast<float>(cell.hits);
                            const bool was_wb_cand = (static_cast<int>(prev_clears) >= kWallBleedMinClears)
                                && (static_cast<float>(prev_clears) >= wb_thr);
                            const bool now_wb_cand = (static_cast<int>(cell.clears) >= kWallBleedMinClears)
                                && (static_cast<float>(cell.clears) >= wb_thr);
                            if (!was_wb_cand && now_wb_cand
                                && isWallBleedLocked(vx, vy, vz, cell, cached_ck, *cached_chunk))
                            {
                                crossed_wall_bleed = true;
                            }
                        }
                        if (crossed_ratio || crossed_wall_bleed) {
                            erased_cols->push_back(packXY(vx, vy));
                        }
                    }
                }
            }
        }

        if (t_max_x <= t_max_y && t_max_x <= t_max_z) {
            t_curr = t_max_x;
            t_max_x += t_delta_x;
            vx += step_x;
        } else if (t_max_y <= t_max_z) {
            t_curr = t_max_y;
            t_max_y += t_delta_y;
            vy += step_y;
        } else {
            t_curr = t_max_z;
            t_max_z += t_delta_z;
            vz += step_z;
        }
    }
}

auto OnlineTerrainMapper::extractColumnGridLocked(
    int32_t min_vx,
    int32_t max_vx,
    int32_t min_vy,
    int32_t max_vy,
    std::vector<uint8_t>* erased_cols
) const -> StaticTerrainBuilder::VoxelColumnGrid {
    if (erased_cols != nullptr) {
        erased_cols->clear();
    }
    if (max_vx < min_vx || max_vy < min_vy) {
        return {};
    }

    const int nx = static_cast<int>(max_vx - min_vx + 1);
    const int ny = static_cast<int>(max_vy - min_vy + 1);
    const Eigen::Vector2d origin(
        static_cast<double>(min_vx) * cfg_.resolution,
        static_cast<double>(min_vy) * cfg_.resolution
    );
    const grid_map::GridGeometry geom =
        grid_map::GridGeometry::fromOriginAndSize(origin, Eigen::Vector2i(nx, ny), cfg_.resolution);
    if (!geom.isValid()) {
        return {};
    }

    auto grid = StaticTerrainBuilder::VoxelColumnGrid::create(geom, /*solid_mesh=*/false, /*ray_clearing=*/true);
    if (erased_cols != nullptr) {
        erased_cols->assign(static_cast<size_t>(nx) * static_cast<size_t>(ny), 0);
    }
    const float res_f = static_cast<float>(cfg_.resolution);

    auto processChunk = [&](const ChunkKey& ck, const Chunk& chunk) {
        const int32_t base_x = ck.cx * kChunkSize;
        const int32_t base_y = ck.cy * kChunkSize;
        const int32_t base_z = ck.cz * kChunkSize;

        const int32_t x_lo = std::max(base_x, min_vx);
        const int32_t x_hi = std::min(base_x + kChunkSize - 1, max_vx);
        const int32_t y_lo = std::max(base_y, min_vy);
        const int32_t y_hi = std::min(base_y + kChunkSize - 1, max_vy);

        for (int32_t vx = x_lo; vx <= x_hi; ++vx) {
            const int32_t lx = vx - base_x;
            const size_t row_off = static_cast<size_t>(vx - min_vx) * static_cast<size_t>(ny);
            for (int32_t vy = y_lo; vy <= y_hi; ++vy) {
                const int32_t ly = vy - base_y;
                const size_t col_idx = row_off + static_cast<size_t>(vy - min_vy);
                for (int32_t lz = 0; lz < kChunkSize; ++lz) {
                    const VoxelCell& cell = chunk.cells[localVoxelIndex(lx, ly, lz)];
                    if (cell.hits == 0 && cell.clears == 0) {
                        continue;
                    }
                    if (isStaticOccupiedLocked(vx, vy, base_z + lz, cell, ck, chunk)) {
                        grid.occ_spans[col_idx].push_back(StaticTerrainBuilder::ZSpan {
                            static_cast<float>(cell.z_min_mm) * 0.001f,
                            static_cast<float>(cell.z_max_mm) * 0.001f,
                        });
                    } else {
                        if (erased_cols != nullptr && static_cast<int>(cell.hits) >= cfg_.static_min_hits) {
                            (*erased_cols)[col_idx] = 1;
                        }
                        if (static_cast<int>(cell.clears) >= cfg_.ray_clear_min_clears) {
                            const float vz_lo = static_cast<float>(base_z + lz) * res_f;
                            grid.cleared_spans[col_idx].push_back(StaticTerrainBuilder::ZSpan {vz_lo, vz_lo + res_f});
                        }
                    }
                }
            }
        }
    };

    const int32_t min_cx = floorDiv16(min_vx);
    const int32_t max_cx = floorDiv16(max_vx);
    const int32_t min_cy = floorDiv16(min_vy);
    const int32_t max_cy = floorDiv16(max_vy);
    const int64_t tile_span = static_cast<int64_t>(max_cx - min_cx + 1) * static_cast<int64_t>(max_cy - min_cy + 1);
    const double min_z_bound = std::min(cfg_.min_z, static_cast<double>(cfg_.terrain.min_ground_height));
    const int32_t min_cz = floorDiv16(static_cast<int32_t>(std::floor(min_z_bound * inv_res_))) - 1;

    // 极小局部补丁走 tile_max_cz_ 定向查询，避免在大地图中为单个补丁遍历全量 chunks_
    if (tile_span <= 4 && chunks_.size() > 32) {
        for (int32_t cx = min_cx; cx <= max_cx; ++cx) {
            for (int32_t cy = min_cy; cy <= max_cy; ++cy) {
                const auto max_it = tile_max_cz_.find(packXY(cx, cy));
                if (max_it == tile_max_cz_.end()) {
                    continue;
                }
                const int32_t max_cz = max_it->second;
                for (int32_t cz = min_cz; cz <= max_cz; ++cz) {
                    const ChunkKey ck {cx, cy, cz};
                    const Chunk* ch = findChunkLocked(ck);
                    if (ch != nullptr) {
                        processChunk(ck, *ch);
                    }
                }
            }
        }
    } else {
        for (const auto& kv: chunks_) {
            const ChunkKey& ck = kv.first;
            const int32_t base_x = ck.cx * kChunkSize;
            const int32_t base_y = ck.cy * kChunkSize;
            // 先做 chunk 级 AABB 拒绝，完全跳过 preview bbox 外的稀疏块（不做任何逐格计算）。
            if (base_x + kChunkSize - 1 < min_vx || base_x > max_vx || base_y + kChunkSize - 1 < min_vy
                || base_y > max_vy)
            {
                continue;
            }
            processChunk(ck, *kv.second);
        }
    }

    const size_t total_cols = static_cast<size_t>(nx) * static_cast<size_t>(ny);
    if (total_cols <= 256 && footprint_ground_.size() > total_cols) {
        for (int32_t vx = min_vx; vx <= max_vx; ++vx) {
            const size_t row_off = static_cast<size_t>(vx - min_vx) * static_cast<size_t>(ny);
            for (int32_t vy = min_vy; vy <= max_vy; ++vy) {
                const auto it = footprint_ground_.find(packXY(vx, vy));
                if (it != footprint_ground_.end()) {
                    grid.footprint_ground_z[row_off + static_cast<size_t>(vy - min_vy)] = it->second;
                }
            }
        }
    } else {
        for (const auto& kv: footprint_ground_) {
            const int32_t vx = unpackX(kv.first);
            const int32_t vy = unpackY(kv.first);
            if (vx < min_vx || vx > max_vx || vy < min_vy || vy > max_vy) {
                continue;
            }
            const size_t col_idx =
                static_cast<size_t>(vx - min_vx) * static_cast<size_t>(ny) + static_cast<size_t>(vy - min_vy);
            grid.footprint_ground_z[col_idx] = kv.second;
        }
    }

    return grid;
}

auto OnlineTerrainMapper::integrateFrame(
    std::span<const Eigen::Vector3d> pts_world,
    const SensorPose& pose,
    const Eigen::Quaterniond& q_base_w,
    const SensorModel& sensor,
    bool build_local_preview,
    std::vector<Eigen::Vector3f>* preview_cloud,
    std::vector<Eigen::Vector3f>* filtered_cloud
) -> std::optional<LocalTerrainFrame> {
    if (filtered_cloud != nullptr) {
        filtered_cloud->clear();
    }
    if (!pose.p_base_w.allFinite() || !pose.p_lidar_w.allFinite()) {
        return std::nullopt;
    }

    sensor.checkExtrinsicsOnce(pts_world, pose);

    StaticTerrainBuilder::VoxelColumnGrid local_grid;
    bool run_preview = false;
    struct OutsidePatch {
        StaticTerrainBuilder::VoxelColumnGrid grid;
        int32_t vx = 0;
        int32_t vy = 0;
        int center_ix = 0;
        int center_iy = 0;
    };
    std::vector<uint8_t> local_erased_cols;
    std::vector<std::pair<int32_t, int32_t>> outside_cleared_cols;
    std::vector<OutsidePatch> outside_patches;
    int emit_halo = 0;

    {
        std::lock_guard<std::mutex> lock(mtx_);

        // 1. 机身倾角跳变门控（防上下坡撞坎或配准抖动向 3D 静态图写入歪斜重影）
        if (has_prev_pose_) {
            const double tilt_deg = SensorModel::tiltAngleDeg(prev_q_base_w_, q_base_w);
            if (tilt_deg > cfg_.tilt_jump_deg) {
                tilt_hold_left_ = std::max(tilt_hold_left_, cfg_.tilt_hold_frames);
            }
        }
        prev_q_base_w_ = (q_base_w.squaredNorm() > 1e-12) ? q_base_w.normalized() : Eigen::Quaterniond::Identity();
        has_prev_pose_ = true;

        if (tilt_hold_left_ > 0) {
            --tilt_hold_left_;
            return std::nullopt;
        }

        ++frame_seq_;
        if (frame_seq_ == 0) {
            frame_seq_ = 1;
        }
        const uint32_t cur_frame = frame_seq_;

        // 2. 更新车底脚印地面高度（弥合 Mid-360 上仰安装导致的车底盲区）
        const float fp_z = static_cast<float>(pose.p_base_w.z() - cfg_.footprint_ground_offset);
        if (std::isfinite(fp_z) && cfg_.footprint_radius > 0.0) {
            const int r_cells = std::max(0, static_cast<int>(std::ceil(cfg_.footprint_radius * inv_res_)));
            const double r_sq = cfg_.footprint_radius * cfg_.footprint_radius;
            const int32_t cx = static_cast<int32_t>(std::floor(pose.p_base_w.x() * inv_res_));
            const int32_t cy = static_cast<int32_t>(std::floor(pose.p_base_w.y() * inv_res_));
            for (int dx = -r_cells; dx <= r_cells; ++dx) {
                for (int dy = -r_cells; dy <= r_cells; ++dy) {
                    const int32_t vx = cx + dx;
                    const int32_t vy = cy + dy;
                    const double wx = (static_cast<double>(vx) + 0.5) * cfg_.resolution;
                    const double wy = (static_cast<double>(vy) + 0.5) * cfg_.resolution;
                    const double ddx = wx - pose.p_base_w.x();
                    const double ddy = wy - pose.p_base_w.y();
                    if (ddx * ddx + ddy * ddy > r_sq) {
                        continue;
                    }
                    const uint64_t key = packXY(vx, vy);
                    const auto it = footprint_ground_.find(key);
                    if (it == footprint_ground_.end()) {
                        footprint_ground_.emplace(key, fp_z);
                    } else if (std::abs(fp_z - it->second) > 0.15f) {
                        it->second = fp_z;
                    } else {
                        it->second = 0.7f * it->second + 0.3f * fp_z;
                    }
                }
            }
        }

        // 3. 写入静态占据命中（严格截断在 trust_range 内）并均匀采样扫空射线端点（最远到 clear_range）
        const double trust_r_sq = cfg_.trust_range * cfg_.trust_range;
        const double min_r_sq = sensor.config().min_range * sensor.config().min_range;
        const double clear_r_sq = cfg_.clear_range * cfg_.clear_range;
        const size_t ray_stride = (cfg_.max_clear_rays_per_frame > 0)
            ? std::max<size_t>(1, pts_world.size() / static_cast<size_t>(cfg_.max_clear_rays_per_frame))
            : pts_world.size() + 1;

        clear_targets_scratch_.clear();
        std::vector<Eigen::Vector3d>& clear_targets = clear_targets_scratch_;
        if (cfg_.max_clear_rays_per_frame > 0) {
            const size_t desired =
                std::min<size_t>(pts_world.size(), static_cast<size_t>(cfg_.max_clear_rays_per_frame) + 1);
            if (desired > clear_targets.capacity()) {
                clear_targets.reserve(desired);
            }
        }

        const bool collect_filtered = (filtered_cloud != nullptr);
        if (collect_filtered && filtered_cloud->capacity() < pts_world.size()) {
            filtered_cloud->reserve(pts_world.size());
        }

        bool has_cached = false;
        ChunkKey cached_ck {};
        Chunk* cached_chunk = nullptr;

        for (size_t i = 0; i < pts_world.size(); ++i) {
            const Eigen::Vector3d& pw = pts_world[i];
            if (!pw.allFinite() || pw.z() < cfg_.min_z || pw.z() > cfg_.max_z) {
                continue;
            }
            if (sensor.isSelfHit(pw, pose)) {
                continue;
            }
            const Eigen::Vector3d d = pw - pose.p_lidar_w;
            const double r_sq = d.squaredNorm();
            if (r_sq < min_r_sq) {
                continue;
            }

            if (ray_stride <= pts_world.size() && (i % ray_stride == 0) && r_sq <= clear_r_sq) {
                clear_targets.push_back(pw);
            }

            const double r_xy_sq = d.x() * d.x() + d.y() * d.y();
            if (r_xy_sq > trust_r_sq) {
                continue;
            }

            if (collect_filtered) {
                filtered_cloud
                    ->emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()), static_cast<float>(pw.z()));
            }

            const int32_t vx = static_cast<int32_t>(std::floor(pw.x() * inv_res_));
            const int32_t vy = static_cast<int32_t>(std::floor(pw.y() * inv_res_));
            const int32_t vz = static_cast<int32_t>(std::floor(pw.z() * inv_res_));

            const ChunkKey ck {floorDiv16(vx), floorDiv16(vy), floorDiv16(vz)};
            if (!has_cached || !(ck == cached_ck)) {
                cached_ck = ck;
                cached_chunk = getOrCreateChunkLocked(ck);
                has_cached = true;
                if (cached_chunk != nullptr) {
                    const uint64_t tile_key = packXY(ck.cx, ck.cy);
                    const auto [it, inserted] = tile_max_cz_.emplace(tile_key, ck.cz);
                    if (!inserted && ck.cz > it->second) {
                        it->second = ck.cz;
                    }
                }
            }
            if (cached_chunk == nullptr) {
                continue;
            }

            VoxelCell& cell = cached_chunk->cells[localVoxelIndex(mod16(vx), mod16(vy), mod16(vz))];
            const int16_t z_mm = toMillimeters(pw.z());
            if (cell.hits == 0) {
                cell.z_min_mm = z_mm;
                cell.z_max_mm = z_mm;
            } else {
                // 非对称分位追踪（~10% 分位下界）：防止偶发向下漂移/墙沿漏点永久拉低体素 z_min_mm，
                // 同时保留精确 z_max_mm（确保台阶/高台顶面高度零偏差）。
                if (z_mm < cell.z_min_mm) {
                    const int32_t next_min = std::max<int32_t>(
                        static_cast<int32_t>(z_mm),
                        static_cast<int32_t>(cell.z_min_mm) - kZMinQuantileStepDownMm
                    );
                    cell.z_min_mm = static_cast<int16_t>(next_min);
                } else if (z_mm > cell.z_min_mm) {
                    const int32_t next_min = std::min<int32_t>(
                        static_cast<int32_t>(z_mm),
                        static_cast<int32_t>(cell.z_min_mm) + kZMinQuantileStepUpMm
                    );
                    cell.z_min_mm = static_cast<int16_t>(next_min);
                }
                cell.z_max_mm = std::max(cell.z_max_mm, z_mm);
            }
            if (cell.last_hit_frame != cur_frame) {
                if (cell.hits < std::numeric_limits<uint16_t>::max()) {
                    ++cell.hits;
                }
                cell.last_hit_frame = cur_frame;
            }
        }

        // 4. 3D DDA 射线扫空（仅遍历已分配 chunk，清除穿过信任区的动态过客并积累悬空下沿净空证据）
        const bool track_erased = build_local_preview && cfg_.preview_radius > 0.0;
        if (track_erased) {
            erased_cols_scratch_.clear();
        }
        std::vector<uint64_t>* erased_sink = track_erased ? &erased_cols_scratch_ : nullptr;
        for (const auto& target: clear_targets) {
            castClearRayLocked(pose.p_lidar_w, target, cur_frame, erased_sink);
        }

        revision_.fetch_add(1, std::memory_order_release);

        // 5. 若需要实时 2D 预览，短锁提取车周 preview_radius 内的稀疏列后在锁外运行 StaticTerrainBuilder
        if (build_local_preview && cfg_.preview_radius > 0.0) {
            const int32_t min_vx =
                static_cast<int32_t>(std::floor((pose.p_base_w.x() - cfg_.preview_radius) * inv_res_));
            const int32_t max_vx =
                static_cast<int32_t>(std::floor((pose.p_base_w.x() + cfg_.preview_radius) * inv_res_));
            const int32_t min_vy =
                static_cast<int32_t>(std::floor((pose.p_base_w.y() - cfg_.preview_radius) * inv_res_));
            const int32_t max_vy =
                static_cast<int32_t>(std::floor((pose.p_base_w.y() + cfg_.preview_radius) * inv_res_));
            local_grid = extractColumnGridLocked(min_vx, max_vx, min_vy, max_vy, &local_erased_cols);
            run_preview = local_grid.geometry.isValid();
            if (run_preview) {
                const int nx = local_grid.geometry.voxel_num.x();
                const int ny = local_grid.geometry.voxel_num.y();
                const double margin_m = cfg_.preview_radius - cfg_.trust_range;
                if (margin_m > 0.0 && nx > 2 && ny > 2) {
                    const int max_halo_by_margin =
                        std::max(0, static_cast<int>(std::floor(margin_m * inv_res_ + 1e-6)));
                    const int max_halo_by_size = std::min((nx - 1) / 2, (ny - 1) / 2);
                    emit_halo = std::min({kPreviewEmitHalo, max_halo_by_margin, max_halo_by_size});
                }

                // 将本帧被射线擦除且落在发射窗口之外（含 halo 外缘与 preview_radius..clear_range）的列同步到 2D
                if (!erased_cols_scratch_.empty()) {
                    std::sort(erased_cols_scratch_.begin(), erased_cols_scratch_.end());
                    erased_cols_scratch_.erase(
                        std::unique(erased_cols_scratch_.begin(), erased_cols_scratch_.end()),
                        erased_cols_scratch_.end()
                    );
                    const int32_t emit_min_vx = min_vx + emit_halo;
                    const int32_t emit_max_vx = max_vx - emit_halo;
                    const int32_t emit_min_vy = min_vy + emit_halo;
                    const int32_t emit_max_vy = max_vy - emit_halo;
                    constexpr size_t kMaxOutsidePatchesPerFrame = 32;
                    const int32_t patch_r = std::max(kPreviewEmitHalo, cfg_.terrain.overhang_ground_radius);
                    for (const uint64_t key: erased_cols_scratch_) {
                        const int32_t evx = unpackX(key);
                        const int32_t evy = unpackY(key);
                        if (evx >= emit_min_vx && evx <= emit_max_vx && evy >= emit_min_vy && evy <= emit_max_vy) {
                            continue;
                        }
                        if (!columnHasStaticOccupiedLocked(evx, evy)) {
                            outside_cleared_cols.emplace_back(evx, evy);
                        } else if (outside_patches.size() < kMaxOutsidePatchesPerFrame) {
                            OutsidePatch patch;
                            patch.vx = evx;
                            patch.vy = evy;
                            patch.grid =
                                extractColumnGridLocked(evx - patch_r, evx + patch_r, evy - patch_r, evy + patch_r);
                            if (patch.grid.geometry.isValid()) {
                                patch.center_ix = static_cast<int>(patch_r);
                                patch.center_iy = static_cast<int>(patch_r);
                                outside_patches.push_back(std::move(patch));
                            }
                        }
                    }
                }
            }
        }
    }

    if (!run_preview) {
        return std::nullopt;
    }

    const StaticTerrainBuilder::BuildResult built = builder_.build(local_grid);
    LocalTerrainFrame frame;
    frame.terrain_resolution = cfg_.resolution;
    frame.robot_pos = Eigen::Vector2d(pose.p_base_w.x(), pose.p_base_w.y());
    frame.robot_footprint_radius = cfg_.footprint_radius;
    frame.has_robot_pos = true;
    frame.authoritative_free = true;

    const int nx = built.geometry.voxel_num.x();
    const int ny = built.geometry.voxel_num.y();
    frame.obstacle_cells.reserve(static_cast<size_t>(built.occupied_count));
    frame.free_cells.reserve(static_cast<size_t>(built.free_count));
    if (!outside_cleared_cols.empty()) {
        frame.cleared_cells.reserve(outside_cleared_cols.size() + 32);
    }
    if (preview_cloud != nullptr) {
        preview_cloud->clear();
        preview_cloud->reserve(static_cast<size_t>(built.occupied_count));
    }

    const int ix_lo = emit_halo;
    const int ix_hi = nx - emit_halo;
    const int iy_lo = emit_halo;
    const int iy_hi = ny - emit_halo;
    for (int ix = ix_lo; ix < ix_hi; ++ix) {
        const size_t row_off = static_cast<size_t>(ix) * static_cast<size_t>(ny);
        for (int iy = iy_lo; iy < iy_hi; ++iy) {
            const size_t idx = row_off + static_cast<size_t>(iy);
            const CellState st = built.cells[idx];
            if (st == CellState::UNKNOWN) {
                if (idx < local_erased_cols.size() && local_erased_cols[idx] != 0) {
                    const Eigen::Vector2d wp = built.geometry.indexToPos(Eigen::Vector2i(ix, iy));
                    frame.cleared_cells.emplace_back(
                        static_cast<float>(wp.x()),
                        static_cast<float>(wp.y()),
                        static_cast<float>(pose.p_base_w.z())
                    );
                }
                continue;
            }
            const Eigen::Vector2d wp = built.geometry.indexToPos(Eigen::Vector2i(ix, iy));
            const float gz = built.terrain_prior.ground_z[idx];
            const double z_vis = std::isfinite(gz) ? static_cast<double>(gz) : (pose.p_base_w.z() + 0.30);
            if (st == CellState::OCCUPIED) {
                frame.obstacle_cells.emplace_back(wp.x(), wp.y(), z_vis);
                if (preview_cloud != nullptr) {
                    preview_cloud->emplace_back(
                        static_cast<float>(wp.x()),
                        static_cast<float>(wp.y()),
                        static_cast<float>(z_vis)
                    );
                }
            } else {
                frame.free_cells.emplace_back(wp.x(), wp.y(), z_vis);
            }
        }
    }

    for (const auto& [evx, evy]: outside_cleared_cols) {
        const float wx = static_cast<float>((static_cast<double>(evx) + 0.5) * cfg_.resolution);
        const float wy = static_cast<float>((static_cast<double>(evy) + 0.5) * cfg_.resolution);
        frame.cleared_cells.emplace_back(wx, wy, static_cast<float>(pose.p_base_w.z()));
    }

    for (const auto& patch: outside_patches) {
        const StaticTerrainBuilder::BuildResult pb = builder_.build(patch.grid);
        if (!pb.geometry.isValid()) {
            continue;
        }
        const size_t cidx = static_cast<size_t>(patch.center_ix) * static_cast<size_t>(pb.geometry.voxel_num.y())
            + static_cast<size_t>(patch.center_iy);
        if (cidx >= pb.cells.size()) {
            continue;
        }
        const CellState st = pb.cells[cidx];
        const float wx = static_cast<float>((static_cast<double>(patch.vx) + 0.5) * cfg_.resolution);
        const float wy = static_cast<float>((static_cast<double>(patch.vy) + 0.5) * cfg_.resolution);
        if (st == CellState::FREE) {
            const float gz = pb.terrain_prior.ground_z[cidx];
            const float wz = std::isfinite(gz) ? gz : static_cast<float>(pose.p_base_w.z());
            frame.free_cells.emplace_back(wx, wy, wz);
        } else if (st == CellState::UNKNOWN) {
            frame.cleared_cells.emplace_back(wx, wy, static_cast<float>(pose.p_base_w.z()));
        }
    }

    return frame;
}

auto OnlineTerrainMapper::extractStaticVoxelCloud(
    const Eigen::Vector3d& box_min,
    const Eigen::Vector3d& box_max,
    std::vector<Eigen::Vector3f>& out,
    size_t max_points
) const -> uint64_t {
    out.clear();
    if (!box_min.allFinite() || !box_max.allFinite() || box_max.x() < box_min.x() || box_max.y() < box_min.y()
        || box_max.z() < box_min.z())
    {
        return revision_.load(std::memory_order_acquire);
    }

    std::lock_guard<std::mutex> lock(mtx_);
    const uint64_t rev = revision_.load(std::memory_order_acquire);
    if (chunks_.empty()) {
        return rev;
    }

    const int32_t min_vx = static_cast<int32_t>(std::floor(box_min.x() * inv_res_));
    const int32_t max_vx = static_cast<int32_t>(std::floor(box_max.x() * inv_res_));
    const int32_t min_vy = static_cast<int32_t>(std::floor(box_min.y() * inv_res_));
    const int32_t max_vy = static_cast<int32_t>(std::floor(box_max.y() * inv_res_));
    const int32_t min_vz = static_cast<int32_t>(std::floor(box_min.z() * inv_res_));
    const int32_t max_vz = static_cast<int32_t>(std::floor(box_max.z() * inv_res_));

    for (const auto& kv: chunks_) {
        const ChunkKey& ck = kv.first;
        const int32_t base_x = ck.cx * kChunkSize;
        const int32_t base_y = ck.cy * kChunkSize;
        const int32_t base_z = ck.cz * kChunkSize;
        if (base_x + kChunkSize - 1 < min_vx || base_x > max_vx || base_y + kChunkSize - 1 < min_vy || base_y > max_vy
            || base_z + kChunkSize - 1 < min_vz || base_z > max_vz)
        {
            continue;
        }

        const int32_t x_lo = std::max(base_x, min_vx);
        const int32_t x_hi = std::min(base_x + kChunkSize - 1, max_vx);
        const int32_t y_lo = std::max(base_y, min_vy);
        const int32_t y_hi = std::min(base_y + kChunkSize - 1, max_vy);
        const int32_t z_lo = std::max(base_z, min_vz);
        const int32_t z_hi = std::min(base_z + kChunkSize - 1, max_vz);

        const Chunk& chunk = *kv.second;
        for (int32_t vx = x_lo; vx <= x_hi; ++vx) {
            const int32_t lx = vx - base_x;
            const float wx = static_cast<float>((static_cast<double>(vx) + 0.5) * cfg_.resolution);
            for (int32_t vy = y_lo; vy <= y_hi; ++vy) {
                const int32_t ly = vy - base_y;
                const float wy = static_cast<float>((static_cast<double>(vy) + 0.5) * cfg_.resolution);
                for (int32_t vz = z_lo; vz <= z_hi; ++vz) {
                    const int32_t lz = vz - base_z;
                    const VoxelCell& cell = chunk.cells[localVoxelIndex(lx, ly, lz)];
                    if (!isStaticOccupiedLocked(vx, vy, vz, cell, ck, chunk)) {
                        continue;
                    }
                    const float wz = 0.0005f
                        * static_cast<float>(static_cast<int32_t>(cell.z_min_mm) + static_cast<int32_t>(cell.z_max_mm));
                    out.emplace_back(wx, wy, wz);
                    if (max_points > 0 && out.size() >= max_points) {
                        return rev;
                    }
                }
            }
        }
    }

    return rev;
}

auto OnlineTerrainMapper::buildGlobalStaticMap() const -> std::optional<StaticTerrainBuilder::BuildResult> {
    StaticTerrainBuilder::VoxelColumnGrid global_grid;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        int32_t min_vx = std::numeric_limits<int32_t>::max();
        int32_t max_vx = std::numeric_limits<int32_t>::min();
        int32_t min_vy = std::numeric_limits<int32_t>::max();
        int32_t max_vy = std::numeric_limits<int32_t>::min();
        bool has_any = false;

        for (const auto& kv: chunks_) {
            const ChunkKey& ck = kv.first;
            const int32_t base_x = ck.cx * kChunkSize;
            const int32_t base_y = ck.cy * kChunkSize;
            const int32_t base_z = ck.cz * kChunkSize;
            const Chunk& chunk = *kv.second;
            for (int32_t lx = 0; lx < kChunkSize; ++lx) {
                for (int32_t ly = 0; ly < kChunkSize; ++ly) {
                    bool col_static = false;
                    for (int32_t lz = 0; lz < kChunkSize; ++lz) {
                        const VoxelCell& cell = chunk.cells[localVoxelIndex(lx, ly, lz)];
                        if (isStaticOccupiedLocked(base_x + lx, base_y + ly, base_z + lz, cell, ck, chunk)) {
                            col_static = true;
                            break;
                        }
                    }
                    if (col_static) {
                        const int32_t vx = base_x + lx;
                        const int32_t vy = base_y + ly;
                        min_vx = std::min(min_vx, vx);
                        max_vx = std::max(max_vx, vx);
                        min_vy = std::min(min_vy, vy);
                        max_vy = std::max(max_vy, vy);
                        has_any = true;
                    }
                }
            }
        }

        for (const auto& kv: footprint_ground_) {
            const int32_t vx = unpackX(kv.first);
            const int32_t vy = unpackY(kv.first);
            min_vx = std::min(min_vx, vx);
            max_vx = std::max(max_vx, vx);
            min_vy = std::min(min_vy, vy);
            max_vy = std::max(max_vy, vy);
            has_any = true;
        }

        if (!has_any) {
            return std::nullopt;
        }

        global_grid = extractColumnGridLocked(min_vx, max_vx, min_vy, max_vy);
    }

    if (!global_grid.geometry.isValid()) {
        return std::nullopt;
    }
    return builder_.build(global_grid);
}

auto OnlineTerrainMapper::hasData() const -> bool {
    std::lock_guard<std::mutex> lock(mtx_);
    return !chunks_.empty() || !footprint_ground_.empty();
}

auto OnlineTerrainMapper::stats() const -> Stats {
    std::lock_guard<std::mutex> lock(mtx_);
    Stats s;
    s.chunk_count = chunks_.size();
    s.footprint_cells = footprint_ground_.size();
    s.frame_count = frame_seq_;
    s.chunk_limit_reached = chunk_limit_warned_;
    return s;
}

} // namespace ma_map
