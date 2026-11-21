#include "map/perception/online_terrain_mapper.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ma_map {

namespace {

    constexpr int32_t kChunkSize = 16;

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
    footprint_ground_.clear();
    clear_targets_scratch_.clear();
    frame_seq_ = 0;
    has_prev_pose_ = false;
    prev_q_base_w_ = Eigen::Quaterniond::Identity();
    tilt_hold_left_ = 0;
    chunk_limit_warned_ = false;
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

auto OnlineTerrainMapper::castClearRayLocked(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& target,
    uint32_t frame_seq
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
    const bool downward_ray = (target.z() < origin.z() - 0.05);
    const double min_clear_z = target.z() + 0.03;

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
                    if (cell.clears < std::numeric_limits<uint16_t>::max()) {
                        ++cell.clears;
                    }
                    cell.last_clear_frame = frame_seq;
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

auto OnlineTerrainMapper::extractColumnGridLocked(int32_t min_vx, int32_t max_vx, int32_t min_vy, int32_t max_vy) const
    -> StaticTerrainBuilder::VoxelColumnGrid {
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
    const float res_f = static_cast<float>(cfg_.resolution);

    for (const auto& kv: chunks_) {
        const ChunkKey& ck = kv.first;
        const int32_t base_x = ck.cx * kChunkSize;
        const int32_t base_y = ck.cy * kChunkSize;
        // 先做 chunk 级 AABB 拒绝，完全跳过 preview bbox 外的稀疏块（不做任何逐格计算）。
        if (base_x + kChunkSize - 1 < min_vx || base_x > max_vx || base_y + kChunkSize - 1 < min_vy || base_y > max_vy)
        {
            continue;
        }
        const int32_t base_z = ck.cz * kChunkSize;

        const int32_t x_lo = std::max(base_x, min_vx);
        const int32_t x_hi = std::min(base_x + kChunkSize - 1, max_vx);
        const int32_t y_lo = std::max(base_y, min_vy);
        const int32_t y_hi = std::min(base_y + kChunkSize - 1, max_vy);

        const Chunk& chunk = *kv.second;
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
                    const bool is_static_occ = (static_cast<int>(cell.hits) >= cfg_.static_min_hits)
                        && (static_cast<float>(cell.clears) <= cfg_.static_clear_ratio * static_cast<float>(cell.hits))
                        && (cell.z_min_mm <= cell.z_max_mm);
                    if (is_static_occ) {
                        grid.occ_spans[col_idx].push_back(StaticTerrainBuilder::ZSpan {
                            static_cast<float>(cell.z_min_mm) * 0.001f,
                            static_cast<float>(cell.z_max_mm) * 0.001f,
                        });
                    } else if (static_cast<int>(cell.clears) >= cfg_.ray_clear_min_clears) {
                        const float vz_lo = static_cast<float>(base_z + lz) * res_f;
                        grid.cleared_spans[col_idx].push_back(StaticTerrainBuilder::ZSpan {vz_lo, vz_lo + res_f});
                    }
                }
            }
        }
    }

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

    return grid;
}

auto OnlineTerrainMapper::integrateFrame(
    std::span<const Eigen::Vector3d> pts_world,
    const SensorPose& pose,
    const Eigen::Quaterniond& q_base_w,
    const SensorModel& sensor,
    bool build_local_preview,
    std::vector<Eigen::Vector3f>* preview_cloud
) -> std::optional<LocalTerrainFrame> {
    if (!pose.p_base_w.allFinite() || !pose.p_lidar_w.allFinite()) {
        return std::nullopt;
    }

    sensor.checkExtrinsicsOnce(pts_world, pose);

    StaticTerrainBuilder::VoxelColumnGrid local_grid;
    bool run_preview = false;

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

            const int32_t vx = static_cast<int32_t>(std::floor(pw.x() * inv_res_));
            const int32_t vy = static_cast<int32_t>(std::floor(pw.y() * inv_res_));
            const int32_t vz = static_cast<int32_t>(std::floor(pw.z() * inv_res_));

            const ChunkKey ck {floorDiv16(vx), floorDiv16(vy), floorDiv16(vz)};
            if (!has_cached || !(ck == cached_ck)) {
                cached_ck = ck;
                cached_chunk = getOrCreateChunkLocked(ck);
                has_cached = true;
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
                cell.z_min_mm = std::min(cell.z_min_mm, z_mm);
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
        for (const auto& target: clear_targets) {
            castClearRayLocked(pose.p_lidar_w, target, cur_frame);
        }

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
            local_grid = extractColumnGridLocked(min_vx, max_vx, min_vy, max_vy);
            run_preview = local_grid.geometry.isValid();
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

    const int nx = built.geometry.voxel_num.x();
    const int ny = built.geometry.voxel_num.y();
    frame.obstacle_cells.reserve(static_cast<size_t>(built.occupied_count));
    frame.free_cells.reserve(static_cast<size_t>(built.free_count));
    if (preview_cloud != nullptr) {
        preview_cloud->clear();
        preview_cloud->reserve(static_cast<size_t>(built.occupied_count));
    }

    for (int ix = 0; ix < nx; ++ix) {
        const size_t row_off = static_cast<size_t>(ix) * static_cast<size_t>(ny);
        for (int iy = 0; iy < ny; ++iy) {
            const size_t idx = row_off + static_cast<size_t>(iy);
            const CellState st = built.cells[idx];
            if (st == CellState::UNKNOWN) {
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

    return frame;
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
            const Chunk& chunk = *kv.second;
            for (int32_t lx = 0; lx < kChunkSize; ++lx) {
                for (int32_t ly = 0; ly < kChunkSize; ++ly) {
                    bool col_static = false;
                    for (int32_t lz = 0; lz < kChunkSize; ++lz) {
                        const VoxelCell& cell = chunk.cells[localVoxelIndex(lx, ly, lz)];
                        if (static_cast<int>(cell.hits) >= cfg_.static_min_hits
                            && static_cast<float>(cell.clears)
                                <= cfg_.static_clear_ratio * static_cast<float>(cell.hits)
                            && cell.z_min_mm <= cell.z_max_mm)
                        {
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
