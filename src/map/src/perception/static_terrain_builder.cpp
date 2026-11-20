#include "map/perception/static_terrain_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace ma_map {

namespace {

    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

    enum class ColKind : uint8_t {
        UNOBSERVED = 0,
        HARD_OCC = 1,
        CANDIDATE_GROUND = 2,
        MAYBE_OVERHANG = 3,
    };

    auto mergeSpans(std::vector<StaticTerrainBuilder::ZSpan>& spans, float max_gap) -> void {
        if (spans.empty()) {
            return;
        }
        std::sort(spans.begin(), spans.end(), [](const auto& a, const auto& b) {
            if (a.z_lo != b.z_lo) {
                return a.z_lo < b.z_lo;
            }
            return a.z_hi < b.z_hi;
        });
        size_t write = 0;
        for (size_t i = 1; i < spans.size(); ++i) {
            if (spans[i].z_lo <= spans[write].z_hi + max_gap) {
                spans[write].z_hi = std::max(spans[write].z_hi, spans[i].z_hi);
            } else {
                ++write;
                spans[write] = spans[i];
            }
        }
        spans.resize(write + 1);
    }

    auto medianOf(std::vector<float>& vals) -> float {
        const size_t n = vals.size();
        const size_t mid = n / 2;
        std::nth_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(mid), vals.end());
        const float hi = vals[mid];
        if ((n & 1U) != 0U) {
            return hi;
        }
        const float lo = *std::max_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(mid));
        return 0.5f * (lo + hi);
    }

    // 计算 [z_bot, z_top] 区间内被射线扫空的总有效覆盖厚度
    auto clearedCoverage(
        const std::vector<StaticTerrainBuilder::ZSpan>& cleared,
        float z_bot,
        float z_top
    ) -> float {
        if (cleared.empty() || z_top <= z_bot) {
            return 0.0f;
        }
        float total = 0.0f;
        for (const auto& s: cleared) {
            const float lo = std::max(s.z_lo, z_bot);
            const float hi = std::min(s.z_hi, z_top);
            if (hi > lo) {
                total += (hi - lo);
            }
        }
        return total;
    }

} // namespace

auto StaticTerrainBuilder::build(const VoxelColumnGrid& grid) const -> BuildResult {
    BuildResult out;
    if (!grid.geometry.isValid()) {
        return out;
    }

    const int nx = grid.geometry.voxel_num.x();
    const int ny = grid.geometry.voxel_num.y();
    const size_t n_cells = grid.geometry.cellCount();
    if (grid.occ_spans.size() != n_cells) {
        return out;
    }

    const bool has_cleared = grid.has_ray_clearing && grid.cleared_spans.size() == n_cells;
    const bool has_fp_vec = grid.footprint_ground_z.size() == n_cells;
    const float res = static_cast<float>(grid.geometry.resolution);
    const float kNan = std::numeric_limits<float>::quiet_NaN();
    const float kInf = std::numeric_limits<float>::infinity();

    out.geometry = grid.geometry;
    out.cells.assign(n_cells, CellState::UNKNOWN);
    out.terrain_prior = StaticTerrainPrior::createEmpty(grid.geometry);

    std::vector<ColKind> col_kind(n_cells, ColKind::UNOBSERVED);
    std::vector<float> raw_ground(n_cells, kNan);
    std::vector<float> overhang_z(n_cells, kInf);
    std::vector<float> lowest_occ_z(n_cells, kInf);
    std::vector<uint8_t> has_cleared_below(n_cells, 0);

    std::vector<ZSpan> merged_occ;
    std::vector<ZSpan> merged_clr;
    merged_occ.reserve(16);
    merged_clr.reserve(16);

    // 阶段 1：逐列合并连续 Z 占据段，确定初始地面、硬墙体与悬空候选。
    for (size_t idx = 0; idx < n_cells; ++idx) {
        const float fp_z = has_fp_vec ? grid.footprint_ground_z[idx] : kNan;
        const bool has_fp = std::isfinite(fp_z);

        const auto& src_occ = grid.occ_spans[idx];
        if (src_occ.empty()) {
            if (has_fp) {
                col_kind[idx] = ColKind::CANDIDATE_GROUND;
                raw_ground[idx] = fp_z;
            }
            continue;
        }

        merged_occ = src_occ;
        mergeSpans(merged_occ, cfg_.gap_fill_distance);

        // 过滤低于 min_ground_height 的地下杂点段
        size_t first_valid = 0;
        while (first_valid < merged_occ.size() && merged_occ[first_valid].z_hi < cfg_.min_ground_height) {
            ++first_valid;
        }
        if (first_valid >= merged_occ.size()) {
            if (has_fp) {
                col_kind[idx] = ColKind::CANDIDATE_GROUND;
                raw_ground[idx] = fp_z;
            }
            continue;
        }

        const float first_z_lo = merged_occ[first_valid].z_lo;
        lowest_occ_z[idx] = first_z_lo;

        // 若机器人脚印直接碾过此列，且所有静态占据都在脚印地面之上 >= min_overhang_clearance，直接确认悬空可通行
        if (has_fp && (first_z_lo - fp_z >= cfg_.min_overhang_clearance)) {
            col_kind[idx] = ColKind::CANDIDATE_GROUND;
            raw_ground[idx] = fp_z;
            overhang_z[idx] = first_z_lo;
            continue;
        }

        // 检查射线扫空证据（是否存在明显位于最低占据段下方的净空）
        if (has_cleared && !grid.cleared_spans[idx].empty()) {
            merged_clr = grid.cleared_spans[idx];
            mergeSpans(merged_clr, 0.01f);
            const float clr_below =
                clearedCoverage(merged_clr, cfg_.min_ground_height, first_z_lo - 0.02f);
            if (clr_below >= cfg_.min_cleared_overhang_span) {
                has_cleared_below[idx] = 1;
            }
        }

        bool found_ground = false;
        bool hit_vertical_wall = false;
        for (size_t k = first_valid; k < merged_occ.size(); ++k) {
            const ZSpan& span = merged_occ[k];
            if (span.z_hi > cfg_.max_ground_height) {
                break;
            }
            const float thickness = span.z_hi - span.z_lo;
            if (!grid.is_solid_mesh && thickness > cfg_.max_surface_thickness) {
                hit_vertical_wall = true;
                break;
            }

            const float g_cand = grid.is_solid_mesh ? span.z_hi : (0.5f * (span.z_lo + span.z_hi));
            if (k + 1 < merged_occ.size()) {
                const float next_lo = merged_occ[k + 1].z_lo;
                const float clearance = next_lo - g_cand;
                if (clearance >= cfg_.min_overhang_clearance) {
                    col_kind[idx] = ColKind::CANDIDATE_GROUND;
                    raw_ground[idx] = g_cand;
                    overhang_z[idx] = next_lo;
                    found_ground = true;
                    break;
                }
                // 净空不足：机器人钻不过这段缝隙，继续检查上一段顶面（如高台/台阶面；若为低杆则由阶段 4 步高判障碍）
            } else {
                col_kind[idx] = ColKind::CANDIDATE_GROUND;
                raw_ground[idx] = g_cand;
                overhang_z[idx] = kInf;
                found_ground = true;
                break;
            }
        }

        if (!found_ground) {
            if (hit_vertical_wall) {
                col_kind[idx] = ColKind::HARD_OCC;
            } else {
                col_kind[idx] = ColKind::MAYBE_OVERHANG;
            }
        }
    }

    // 阶段 2：悬空列恢复（仅在有射线扫空证据或高空天花板夹于双侧地面之间时触发，不误放实心高台/矮墙）。
    if (cfg_.overhang_ground_radius > 0) {
        const std::vector<ColKind> kind_snap = col_kind;
        const std::vector<float> ground_snap = raw_ground;
        std::vector<float> ring_vals;
        ring_vals.reserve(static_cast<size_t>(8 * cfg_.overhang_ground_radius));

        for (int ix = 0; ix < nx; ++ix) {
            for (int iy = 0; iy < ny; ++iy) {
                const size_t idx = static_cast<size_t>(ix) * static_cast<size_t>(ny) + static_cast<size_t>(iy);
                const float z_lo = lowest_occ_z[idx];
                if (!std::isfinite(z_lo)) {
                    continue;
                }

                // 情况 A：射线直接扫空了最低占据下方（即使阶段 1 暂将杆顶当成了 CANDIDATE_GROUND，也纠正为悬空！）
                const bool ray_confirmed_overhang = (!grid.is_solid_mesh && has_cleared_below[idx] != 0
                                                     && (kind_snap[idx] == ColKind::MAYBE_OVERHANG
                                                         || kind_snap[idx] == ColKind::CANDIDATE_GROUND));
                // 情况 B：无射线模式（静态 PCD）中高于 max_ground_height 的高空屋顶/桥底，且紧邻双侧均有地面
                const bool high_arch_candidate =
                    (!has_cleared && kind_snap[idx] == ColKind::MAYBE_OVERHANG && z_lo > cfg_.max_ground_height);

                if (!ray_confirmed_overhang && !high_arch_candidate) {
                    continue;
                }

                const int max_r = ray_confirmed_overhang ? cfg_.overhang_ground_radius : 2;
                float ref_g = kNan;
                bool has_pos_x = false;
                bool has_neg_x = false;
                bool has_pos_y = false;
                bool has_neg_y = false;

                for (int r = 1; r <= max_r; ++r) {
                    ring_vals.clear();
                    const int x_lo = std::max(0, ix - r);
                    const int x_hi = std::min(nx - 1, ix + r);
                    const int y_lo = std::max(0, iy - r);
                    const int y_hi = std::min(ny - 1, iy + r);
                    for (int jx = x_lo; jx <= x_hi; ++jx) {
                        const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ny);
                        for (int jy = y_lo; jy <= y_hi; ++jy) {
                            if (std::max(std::abs(jx - ix), std::abs(jy - iy)) != r) {
                                continue;
                            }
                            const size_t jidx = jrow + static_cast<size_t>(jy);
                            if (kind_snap[jidx] != ColKind::CANDIDATE_GROUND || has_cleared_below[jidx] != 0) {
                                continue;
                            }
                            const float gz = ground_snap[jidx];
                            if (std::isfinite(gz) && (z_lo - gz >= cfg_.min_overhang_clearance)) {
                                ring_vals.push_back(gz);
                                if (jx > ix) {
                                    has_pos_x = true;
                                } else if (jx < ix) {
                                    has_neg_x = true;
                                }
                                if (jy > iy) {
                                    has_pos_y = true;
                                } else if (jy < iy) {
                                    has_neg_y = true;
                                }
                            }
                        }
                    }
                    if (!ring_vals.empty()) {
                        ref_g = medianOf(ring_vals);
                        break;
                    }
                }

                if (std::isfinite(ref_g)) {
                    if (ray_confirmed_overhang) {
                        // 进一步核验射线确实扫空了 [ref_g + 0.03, z_lo - 0.02] 这段真实净空，避免误将地下噪声当扫空
                        merged_clr = grid.cleared_spans[idx];
                        mergeSpans(merged_clr, 0.01f);
                        const float real_clr = clearedCoverage(merged_clr, ref_g + 0.03f, z_lo - 0.02f);
                        if (real_clr >= cfg_.min_cleared_overhang_span) {
                            col_kind[idx] = ColKind::CANDIDATE_GROUND;
                            raw_ground[idx] = ref_g;
                            overhang_z[idx] = z_lo;
                        }
                    } else if ((has_pos_x && has_neg_x) || (has_pos_y && has_neg_y)) {
                        col_kind[idx] = ColKind::CANDIDATE_GROUND;
                        raw_ground[idx] = ref_g;
                        overhang_z[idx] = z_lo;
                    }
                }
            }
        }
    }

    // 阶段 3：点云模式同面保边 3x3 中值平滑（仅混合高差 <= same_surface_dz 的邻格，杜绝高台边沿跨面糊边）。
    std::vector<float> smooth_ground = raw_ground;
    if (!grid.is_solid_mesh) {
        std::vector<float> win;
        win.reserve(9);
        for (int ix = 0; ix < nx; ++ix) {
            for (int iy = 0; iy < ny; ++iy) {
                const size_t idx = static_cast<size_t>(ix) * static_cast<size_t>(ny) + static_cast<size_t>(iy);
                if (col_kind[idx] != ColKind::CANDIDATE_GROUND || !std::isfinite(raw_ground[idx])) {
                    continue;
                }
                const float g0 = raw_ground[idx];
                win.clear();
                const int x_lo = std::max(0, ix - 1);
                const int x_hi = std::min(nx - 1, ix + 1);
                const int y_lo = std::max(0, iy - 1);
                const int y_hi = std::min(ny - 1, iy + 1);
                for (int jx = x_lo; jx <= x_hi; ++jx) {
                    const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ny);
                    for (int jy = y_lo; jy <= y_hi; ++jy) {
                        const size_t jidx = jrow + static_cast<size_t>(jy);
                        if (col_kind[jidx] != ColKind::CANDIDATE_GROUND) {
                            continue;
                        }
                        const float gn = raw_ground[jidx];
                        if (std::isfinite(gn) && std::abs(gn - g0) <= cfg_.same_surface_dz) {
                            win.push_back(gn);
                        }
                    }
                }
                if (!win.empty()) {
                    smooth_ground[idx] = medianOf(win);
                }
            }
        }
    }

    // 阶段 4：邻格步高判定 + 5x5 同面局部平面拟合陡坡判定。
    const float tan_max_slope = std::tan(cfg_.max_slope_deg * kDegToRad);
    const float tan_plane_include = std::tan(std::min(80.0f, cfg_.max_slope_deg + 12.0f) * kDegToRad);

    for (size_t idx = 0; idx < n_cells; ++idx) {
        if (col_kind[idx] == ColKind::UNOBSERVED) {
            out.cells[idx] = CellState::UNKNOWN;
        } else if (col_kind[idx] == ColKind::HARD_OCC || col_kind[idx] == ColKind::MAYBE_OVERHANG) {
            out.cells[idx] = CellState::OCCUPIED;
        } else {
            out.cells[idx] = CellState::FREE;
        }
    }

    struct SamplePt {
        float x;
        float y;
        float z;
    };
    std::vector<SamplePt> plane_pts;
    plane_pts.reserve(25);

    for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
            const size_t idx = static_cast<size_t>(ix) * static_cast<size_t>(ny) + static_cast<size_t>(iy);
            if (col_kind[idx] != ColKind::CANDIDATE_GROUND) {
                continue;
            }
            const float g0 = smooth_ground[idx];
            if (!std::isfinite(g0)) {
                out.cells[idx] = CellState::OCCUPIED;
                continue;
            }

            // 4.1 8 邻域直接步高检查
            bool is_step_occ = false;
            for (int dx = -1; dx <= 1 && !is_step_occ; ++dx) {
                const int jx = ix + dx;
                if (jx < 0 || jx >= nx) {
                    continue;
                }
                const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ny);
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const int jy = iy + dy;
                    if (jy < 0 || jy >= ny) {
                        continue;
                    }
                    const size_t jidx = jrow + static_cast<size_t>(jy);
                    if (col_kind[jidx] != ColKind::CANDIDATE_GROUND) {
                        continue;
                    }
                    const float gn = smooth_ground[jidx];
                    if (!std::isfinite(gn)) {
                        continue;
                    }
                    const float d_xy = ((dx != 0 && dy != 0) ? 1.41421356f : 1.0f) * res;
                    const float max_dz = d_xy * tan_max_slope + cfg_.step_tol;
                    if (std::abs(gn - g0) > max_dz) {
                        is_step_occ = true;
                        break;
                    }
                }
            }
            if (is_step_occ) {
                out.cells[idx] = CellState::OCCUPIED;
                continue;
            }

            // 4.2 5x5 同面局部平面拟合陡坡检查
            plane_pts.clear();
            for (int dx = -2; dx <= 2; ++dx) {
                const int jx = ix + dx;
                if (jx < 0 || jx >= nx) {
                    continue;
                }
                const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ny);
                for (int dy = -2; dy <= 2; ++dy) {
                    const int jy = iy + dy;
                    if (jy < 0 || jy >= ny) {
                        continue;
                    }
                    const size_t jidx = jrow + static_cast<size_t>(jy);
                    if (col_kind[jidx] != ColKind::CANDIDATE_GROUND) {
                        continue;
                    }
                    const float gn = smooth_ground[jidx];
                    if (!std::isfinite(gn)) {
                        continue;
                    }
                    // 若距离为 2 格，禁止跨过中间硬墙列或步高台阶采样，避免平面拟合跨台阶糊边
                    if (std::max(std::abs(dx), std::abs(dy)) == 2) {
                        const int step_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
                        const int step_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
                        const int mx = ix + step_x;
                        const int my = iy + step_y;
                        const size_t midx = static_cast<size_t>(mx) * static_cast<size_t>(ny) + static_cast<size_t>(my);
                        if (col_kind[midx] != ColKind::CANDIDATE_GROUND || !std::isfinite(smooth_ground[midx])) {
                            continue;
                        }
                        const float d_step =
                            std::hypot(static_cast<float>(dx - step_x), static_cast<float>(dy - step_y)) * res;
                        if (std::abs(gn - smooth_ground[midx]) > d_step * tan_max_slope + cfg_.step_tol) {
                            continue;
                        }
                    }
                    const float d_xy = std::hypot(static_cast<float>(dx), static_cast<float>(dy)) * res;
                    if (std::abs(gn - g0) <= d_xy * tan_plane_include + 0.04f) {
                        plane_pts.push_back(
                            SamplePt {static_cast<float>(dx) * res, static_cast<float>(dy) * res, gn}
                        );
                    }
                }
            }

            if (plane_pts.size() >= 6) {
                const float inv_n = 1.0f / static_cast<float>(plane_pts.size());
                float mx = 0.0f;
                float my = 0.0f;
                float mz = 0.0f;
                for (const auto& pt: plane_pts) {
                    mx += pt.x;
                    my += pt.y;
                    mz += pt.z;
                }
                mx *= inv_n;
                my *= inv_n;
                mz *= inv_n;

                double sxx = 0.0;
                double sxy = 0.0;
                double syy = 0.0;
                double sxz = 0.0;
                double syz = 0.0;
                for (const auto& pt: plane_pts) {
                    const double dx = static_cast<double>(pt.x - mx);
                    const double dy = static_cast<double>(pt.y - my);
                    const double dz = static_cast<double>(pt.z - mz);
                    sxx += dx * dx;
                    sxy += dx * dy;
                    syy += dy * dy;
                    sxz += dx * dz;
                    syz += dy * dz;
                }

                const double det = sxx * syy - sxy * sxy;
                if (det > 1e-10) {
                    const double a = (syy * sxz - sxy * syz) / det;
                    const double b = (sxx * syz - sxy * sxz) / det;
                    double err_sq = 0.0;
                    for (const auto& pt: plane_pts) {
                        const double dx = static_cast<double>(pt.x - mx);
                        const double dy = static_cast<double>(pt.y - my);
                        const double dz = static_cast<double>(pt.z - mz);
                        const double r = a * dx + b * dy - dz;
                        err_sq += r * r;
                    }
                    const double rms = std::sqrt(err_sq * static_cast<double>(inv_n));
                    const double slope_tan = std::hypot(a, b);
                    if (rms <= static_cast<double>(cfg_.plane_rms_max)
                        && slope_tan > static_cast<double>(tan_max_slope))
                    {
                        out.cells[idx] = CellState::OCCUPIED;
                    }
                }
            }
        }
    }

    // 阶段 5：点云模式孤立单格非硬墙障碍去噪（仅当 8 邻域全为 FREE 时消除，绝不抹除竖墙柱/孤立细柱）。
    if (!grid.is_solid_mesh && cfg_.despeckle_isolated_occ) {
        const std::vector<CellState> pre = out.cells;
        for (int ix = 1; ix + 1 < nx; ++ix) {
            for (int iy = 1; iy + 1 < ny; ++iy) {
                const size_t idx = static_cast<size_t>(ix) * static_cast<size_t>(ny) + static_cast<size_t>(iy);
                if (pre[idx] != CellState::OCCUPIED || col_kind[idx] != ColKind::CANDIDATE_GROUND) {
                    continue;
                }
                bool all_free = true;
                for (int dx = -1; dx <= 1 && all_free; ++dx) {
                    const size_t jrow = static_cast<size_t>(ix + dx) * static_cast<size_t>(ny);
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        if (pre[jrow + static_cast<size_t>(iy + dy)] != CellState::FREE) {
                            all_free = false;
                            break;
                        }
                    }
                }
                if (all_free) {
                    out.cells[idx] = CellState::FREE;
                }
            }
        }
    }

    // 阶段 6：写入 2.5D StaticTerrainPrior 与统计计数。
    for (size_t idx = 0; idx < n_cells; ++idx) {
        if (col_kind[idx] == ColKind::CANDIDATE_GROUND && std::isfinite(smooth_ground[idx])) {
            out.terrain_prior.ground_z[idx] = smooth_ground[idx];
            out.terrain_prior.overhang_min_z[idx] = overhang_z[idx];
            if (std::isfinite(overhang_z[idx])) {
                ++out.overhang_count;
            }
        }
        if (out.cells[idx] == CellState::FREE) {
            ++out.free_count;
        } else if (out.cells[idx] == CellState::OCCUPIED) {
            ++out.occupied_count;
        } else {
            ++out.unknown_count;
        }
    }

    return out;
}

} // namespace ma_map
