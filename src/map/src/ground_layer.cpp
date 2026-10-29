#include "map/ground_layer.hpp"
#include <algorithm>
#include <cmath>

namespace Terrain {

void GroundLayer::setConfig(const Config& cfg) {
    cfg_ = cfg;
    n_ = std::max(1, static_cast<int>(std::ceil(cfg_.size / cfg_.resolution)));
    cells_.assign(static_cast<size_t>(n_) * n_, Cell {});
    scratch_.assign(static_cast<size_t>(n_) * n_, Scratch {});
    touched_.clear();
    frame_ = 0;
    seeded_ = false;
}

auto GroundLayer::ringIndex(int kx, int ky) const -> int {
    const int ix = ((kx % n_) + n_) % n_;
    const int iy = ((ky % n_) + n_) % n_;
    return ix * n_ + iy;
}

auto GroundLayer::slot(int kx, int ky) -> Cell& {
    Cell& c = cells_[ringIndex(kx, ky)];
    if (c.kx != kx || c.ky != ky) {
        c = Cell {};
        c.kx = kx;
        c.ky = ky;
    }
    return c;
}

auto GroundLayer::cell(int kx, int ky) const -> const Cell* {
    if (n_ == 0) {
        return nullptr;
    }
    const Cell& c = cells_[ringIndex(kx, ky)];
    return (c.kx == kx && c.ky == ky) ? &c : nullptr;
}

void GroundLayer::seedDisk(const utils::Pose& base_pose, float radius, bool traversed) {
    const float res = cfg_.resolution;
    const int r = static_cast<int>(std::ceil(radius / res));
    const int cx = static_cast<int>(std::floor(base_pose.first.x() / res));
    const int cy = static_cast<int>(std::floor(base_pose.first.y() / res));
    const float h = static_cast<float>(base_pose.first.z()) + cfg_.footprint_ground_offset;
    for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
            if (dx * dx + dy * dy > r * r) {
                continue;
            }
            Cell& c = slot(cx + dx, cy + dy);
            if (traversed) {
                c.flags |= TRAV;
            }
            if (!(c.flags & OBS)) {
                c.h = h;
                c.var = cfg_.seed_sigma * cfg_.seed_sigma;
                c.flags |= SEED;
            }
        }
    }
}

void GroundLayer::update(
    const rog_map::PointCloud& cloud,
    const utils::Pose& base_pose,
    const utils::Vec3f& sensor,
    const VoxelQuery& voxel_query
) {
    if (n_ == 0) {
        return;
    }
    ++frame_;
    const float res = cfg_.resolution;
    const float sx = static_cast<float>(sensor.x()), sy = static_cast<float>(sensor.y());
    const float sz = static_cast<float>(sensor.z());
    const float max_r2 = cfg_.max_range * cfg_.max_range;
    const float min_r2 = cfg_.min_range * cfg_.min_range;
    constexpr float kInf = std::numeric_limits<float>::infinity();

    if (!seeded_) {
        // 启动盲区：雷达看不到车周近处地面，先按机体平面给低置信地面
        seedDisk(base_pose, cfg_.seed_radius, false);
        seeded_ = true;
    }

    // 1) 每格本帧最低点
    touched_.clear();
    for (const auto& p: cloud) {
        const float dx = p.x - sx, dy = p.y - sy;
        const float r2 = dx * dx + dy * dy;
        if (r2 > max_r2 || r2 < min_r2 || !std::isfinite(p.z)) {
            continue;
        }
        const int kx = static_cast<int>(std::floor(p.x / res));
        const int ky = static_cast<int>(std::floor(p.y / res));
        const int idx = ringIndex(kx, ky);
        Scratch& s = scratch_[idx];
        if (s.stamp != frame_ || s.kx != kx || s.ky != ky) {
            if (s.stamp == frame_) {
                continue;  // 环形冲突（距离超过 size 的点），丢弃
            }
            s.stamp = frame_;
            s.kx = kx;
            s.ky = ky;
            s.m1 = s.m2 = kInf;
            s.cnt = 0;
            touched_.push_back(idx);
        }
        ++s.cnt;
        if (p.z < s.m1) {
            s.m2 = s.m1;
            s.m1 = p.z;
            s.range = std::sqrt(r2);
        } else if (p.z < s.m2) {
            s.m2 = p.z;
        }
    }

    // 2) 量测融合：方差加权，易降难升
    for (const int idx: touched_) {
        const Scratch& s = scratch_[idx];
        const float z = s.m1;
        // 最低点正下方已被射线证实为空闲 → 悬空物下沿，不是地面
        if (voxel_query && cfg_.float_free_voxels > 0) {
            const utils::Vec3f center((s.kx + 0.5) * res, (s.ky + 0.5) * res, z);
            bool floating = true;
            for (int i = 1; i <= cfg_.float_free_voxels; ++i) {
                if (voxel_query(center - utils::Vec3f(0, 0, i * res)) != utils::KNOWN_FREE) {
                    floating = false;
                    break;
                }
            }
            if (floating) {
                continue;
            }
        }
        const float sigma = cfg_.meas_sigma + cfg_.meas_sigma_per_m * s.range;
        const float var_m = sigma * sigma;
        Cell& c = slot(s.kx, s.ky);
        if (!(c.flags & OBS)) {
            if ((c.flags & SEED) && z - c.h > cfg_.rise_max) {
                continue;  // 种子格上方的悬空物
            }
            c.h = z;
            c.var = var_m;
            c.flags = static_cast<uint8_t>((c.flags | OBS) & ~SEED);
            c.up = c.down = 0;
        } else {
            const float d = z - c.h;
            const float gate = std::max(cfg_.gate_min, 3.0f * std::sqrt(c.var + var_m));
            if (std::fabs(d) <= gate) {
                const float k = c.var / (c.var + var_m);
                c.h += k * d;
                c.var = std::max((1.0f - k) * c.var, 1e-6f);
                c.up = c.down = 0;
            } else if (d < 0) {
                // 下降：连续 drop_votes 帧给出相近的更低值
                if (c.down > 0 && std::fabs(z - c.vote_h) <= 2 * cfg_.gate_min) {
                    ++c.down;
                    c.vote_h = std::min(c.vote_h, z);
                } else {
                    c.down = 1;
                    c.vote_h = z;
                }
                c.up = 0;
                if (c.down >= cfg_.drop_votes) {
                    c.h = c.vote_h;
                    c.var = var_m;
                    c.down = 0;
                }
            } else {
                if (d > cfg_.rise_max) {
                    continue;  // 远高于地面：悬空物/车体，不计命中
                }
                if (c.up > 0 && std::fabs(z - c.vote_h) <= 2 * cfg_.gate_min) {
                    ++c.up;
                    c.vote_h = std::max(c.vote_h, z);
                } else {
                    c.up = 1;
                    c.vote_h = z;
                }
                c.down = 0;
                if (c.up >= cfg_.rise_votes) {
                    c.h = c.vote_h;
                    c.var = var_m;
                    c.up = 0;
                }
            }
        }
        c.last_hit = frame_;
        c.ub = kInf;  // 有量测的格以量测为准，上界只对未观测格有意义
    }

    // 3) 射线上界：下行射线经过的格，地面不可能高于射线
    for (const auto& p: cloud) {
        const float dz = p.z - sz;
        if (!(dz < 0.0f)) {
            continue;
        }
        const float dx = p.x - sx, dy = p.y - sy;
        const float rxy = std::sqrt(dx * dx + dy * dy);
        const float r_end = std::min(rxy, cfg_.max_range) - cfg_.ub_end_margin;
        if (r_end <= res) {
            continue;
        }
        const float ux = dx / rxy, uy = dy / rxy, slope = dz / rxy;
        int last_kx = INT_MIN, last_ky = INT_MIN;
        for (float r = res; r < r_end; r += res) {
            const float x = sx + ux * r, y = sy + uy * r;
            const int kx = static_cast<int>(std::floor(x / res));
            const int ky = static_cast<int>(std::floor(y / res));
            if (kx == last_kx && ky == last_ky) {
                continue;
            }
            last_kx = kx;
            last_ky = ky;
            const float zr = sz + slope * r;
            Cell& c = slot(kx, ky);
            if ((c.flags & OBS) && c.last_hit != frame_) {
                // 射线从已记地面下方穿过且该格久未命中 → 地面已不存在（被移走/之前是误记）
                if (zr < c.h - cfg_.ub_margin && frame_ - c.last_hit > cfg_.ub_stale_frames && !(c.flags & TRAV)) {
                    c.flags = static_cast<uint8_t>(c.flags & ~OBS);
                    c.up = c.down = 0;
                    c.ub = zr;
                }
            } else if (!(c.flags & OBS)) {
                c.ub = std::min(c.ub, zr);
            }
        }
    }

    // 4) 足迹：走过的格必是地面
    seedDisk(base_pose, cfg_.footprint_radius, true);
}

}  // namespace Terrain
