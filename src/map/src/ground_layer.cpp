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
    // 种子高度：优先取足迹外一圈（r..r+3 格）真实地面的中位数，没有才用 base 高度 + 偏移
    float h = static_cast<float>(base_pose.first.z()) + cfg_.footprint_ground_offset;
    std::vector<float> ring;
    for (int dx = -r - 3; dx <= r + 3; ++dx) {
        for (int dy = -r - 3; dy <= r + 3; ++dy) {
            const int d2 = dx * dx + dy * dy;
            if (d2 <= r * r || d2 > (r + 3) * (r + 3)) {
                continue;
            }
            const Cell* c = cell(cx + dx, cy + dy);
            if (c != nullptr && (c->flags & OBS)) {
                ring.push_back(c->h);
            }
        }
    }
    if (ring.size() >= 8) {
        auto mid = ring.begin() + ring.size() / 2;
        std::nth_element(ring.begin(), mid, ring.end());
        h = *mid;
    }
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

    // 0) 姿态跳变门控：机体 z 轴帧间转角过大时，点云与位姿多半没对齐（压过凸起/台阶瞬间），
    //    错配的远处地面会被记成低洼坑，坑沿再被判成上沿障碍。本帧及其后几帧只更新足迹。
    {
        const utils::Vec3f up = base_pose.second * utils::Vec3f::UnitZ();
        if (has_last_up_ && cfg_.tilt_jump_deg > 0.0f) {
            const float c = std::clamp(static_cast<float>(up.dot(last_up_)), -1.0f, 1.0f);
            const float deg = std::acos(c) * 180.0f / static_cast<float>(M_PI);
            if (deg > cfg_.tilt_jump_deg) {
                tilt_hold_ = cfg_.tilt_hold_frames + 1;
            }
        }
        last_up_ = up;
        has_last_up_ = true;
        last_gated_ = tilt_hold_ > 0;
        if (tilt_hold_ > 0) {
            --tilt_hold_;
            seedDisk(base_pose, cfg_.footprint_radius, true);
            return;
        }
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
    drop_cand_.clear();
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
                    c.down = static_cast<int8_t>(cfg_.drop_votes);  // 封顶，待邻域确认期间不溢出
                    if (cfg_.drop_min_support > 0) {
                        drop_cand_.push_back({s.kx, s.ky, var_m});  // 本帧结束后做邻域一致性确认
                    } else {
                        c.h = c.vote_h;
                        c.var = var_m;
                        c.down = 0;
                    }
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

    // 2b) 下降的空间一致性：真实的下降（下坡、台阶下沿、移走的物体）总是成片出现，
    //     孤立单格下降多为远处噪声或错配，确认前保留原高度与投票，后续帧继续投票。
    //     先统一判定再统一写入，判定只看本帧写入前的状态。
    if (!drop_cand_.empty()) {
        const float tol = 2.0f * cfg_.gate_min;
        drop_ok_.assign(drop_cand_.size(), 0);
        for (size_t i = 0; i < drop_cand_.size(); ++i) {
            const auto& dc = drop_cand_[i];
            const Cell* c = cell(dc.kx, dc.ky);
            int support = 0;
            for (int dx = -1; dx <= 1 && support < cfg_.drop_min_support; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const Cell* n = cell(dc.kx + dx, dc.ky + dy);
                    if (n == nullptr || !(n->flags & (OBS | SEED))) {
                        continue;
                    }
                    if (std::fabs(n->h - c->vote_h) <= tol || (n->down > 0 && std::fabs(n->vote_h - c->vote_h) <= tol)) {
                        ++support;
                    }
                }
            }
            drop_ok_[i] = support >= cfg_.drop_min_support;
        }
        for (size_t i = 0; i < drop_cand_.size(); ++i) {
            if (!drop_ok_[i]) {
                continue;
            }
            Cell& c = slot(drop_cand_[i].kx, drop_cand_[i].ky);
            c.h = c.vote_h;
            c.var = drop_cand_[i].var;
            c.down = 0;
        }
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
