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
        // 过滤非有限段（NaN/Inf 会让 std::sort 违反严格弱序并触发 UB）并在排序前把倒置上下界归一化。
        size_t write_in = 0;
        for (size_t i = 0; i < spans.size(); ++i) {
            const float z_lo = spans[i].z_lo;
            const float z_hi = spans[i].z_hi;
            if (!std::isfinite(z_lo) || !std::isfinite(z_hi)) {
                continue;
            }
            spans[write_in++] = StaticTerrainBuilder::ZSpan {std::min(z_lo, z_hi), std::max(z_lo, z_hi)};
        }
        spans.resize(write_in);
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
    auto clearedCoverage(const std::vector<StaticTerrainBuilder::ZSpan>& cleared, float z_bot, float z_top) -> float {
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

    // 2-pass O(N) 切比雪夫 (L_inf) 距离变换：mask==0 处为 0，mask!=0 处返回到最近背景或边界的格距，并返回最大格距。
    auto chebyshevDt2d(const std::vector<uint8_t>& mask, int lx, int ly, std::vector<int>& dt) -> int {
        constexpr int kInfDist = 1 << 20;
        const size_t n = static_cast<size_t>(lx) * static_cast<size_t>(ly);
        dt.assign(n, 0);
        for (size_t i = 0; i < n; ++i) {
            if (mask[i] != 0) {
                dt[i] = kInfDist;
            }
        }
        for (int ix = 0; ix < lx; ++ix) {
            const size_t row = static_cast<size_t>(ix) * static_cast<size_t>(ly);
            for (int iy = 0; iy < ly; ++iy) {
                if (dt[row + static_cast<size_t>(iy)] == 0) {
                    continue;
                }
                int v = 1;
                if (ix > 0 && iy > 0) {
                    const size_t prow = static_cast<size_t>(ix - 1) * static_cast<size_t>(ly);
                    v = std::min({
                            dt[prow + static_cast<size_t>(iy)],
                            dt[row + static_cast<size_t>(iy - 1)],
                            dt[prow + static_cast<size_t>(iy - 1)],
                        })
                        + 1;
                }
                if (ix > 0 && iy + 1 < ly) {
                    const size_t prow = static_cast<size_t>(ix - 1) * static_cast<size_t>(ly);
                    v = std::min(v, dt[prow + static_cast<size_t>(iy + 1)] + 1);
                }
                dt[row + static_cast<size_t>(iy)] = v;
            }
        }
        int max_dt = 0;
        for (int ix = lx - 1; ix >= 0; --ix) {
            const size_t row = static_cast<size_t>(ix) * static_cast<size_t>(ly);
            for (int iy = ly - 1; iy >= 0; --iy) {
                if (dt[row + static_cast<size_t>(iy)] == 0) {
                    continue;
                }
                int v = dt[row + static_cast<size_t>(iy)];
                if (ix + 1 == lx || iy + 1 == ly) {
                    v = std::min(v, 1);
                } else {
                    const size_t nrow = static_cast<size_t>(ix + 1) * static_cast<size_t>(ly);
                    v = std::min({
                        v,
                        dt[nrow + static_cast<size_t>(iy)] + 1,
                        dt[row + static_cast<size_t>(iy + 1)] + 1,
                        dt[nrow + static_cast<size_t>(iy + 1)] + 1,
                    });
                }
                if (ix + 1 < lx && iy > 0) {
                    const size_t nrow = static_cast<size_t>(ix + 1) * static_cast<size_t>(ly);
                    v = std::min(v, dt[nrow + static_cast<size_t>(iy - 1)] + 1);
                }
                dt[row + static_cast<size_t>(iy)] = v;
                max_dt = std::max(max_dt, v);
            }
        }
        return max_dt;
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
    // 懒分配：仅在点云模式检测到双层叠置（低地漏点 + 上层薄水平面）时启用，常规场景零开销
    std::vector<float> stacked_upper_z;
    std::vector<float> stacked_overhang_z;
    size_t stacked_count = 0;
    bool any_cleared_below = false;

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
        bool has_merged_clr = false;
        if (has_cleared && !grid.cleared_spans[idx].empty()) {
            merged_clr = grid.cleared_spans[idx];
            mergeSpans(merged_clr, 0.01f);
            has_merged_clr = true;
            const float clr_below = clearedCoverage(merged_clr, cfg_.min_ground_height, first_z_lo - 0.02f);
            if (clr_below >= cfg_.min_cleared_overhang_span) {
                has_cleared_below[idx] = 1;
                any_cleared_below = true;
            }
        }

        bool found_ground = false;
        bool hit_vertical_wall = false;
        bool hard_blocked = false;
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
                // 点云模式下，若空心高台底缝透射产生低地面漏点且上方紧邻薄水平顶面，记录为宽平台候选：
                // 要求下方无悬空扫空、上段上方无低矮遮挡且两段夹层未被开放射线扫空（排除开放低矮限高杆）。
                if (!grid.is_solid_mesh && has_cleared_below[idx] == 0) {
                    const ZSpan& upper = merged_occ[k + 1];
                    const float upper_thick = upper.z_hi - upper.z_lo;
                    if (upper.z_hi <= cfg_.max_ground_height && upper_thick <= cfg_.max_surface_thickness) {
                        const float u_cand = 0.5f * (upper.z_lo + upper.z_hi);
                        bool upper_clear = true;
                        float u_ov = kInf;
                        if (k + 2 < merged_occ.size()) {
                            const float nn_lo = merged_occ[k + 2].z_lo;
                            if (nn_lo - u_cand >= cfg_.min_overhang_clearance) {
                                u_ov = nn_lo;
                            } else {
                                upper_clear = false;
                            }
                        }
                        if (upper_clear) {
                            const float gap = upper.z_lo - span.z_hi;
                            const float gap_clr = has_merged_clr
                                ? clearedCoverage(merged_clr, span.z_hi + 0.01f, upper.z_lo - 0.01f)
                                : 0.0f;
                            const bool gap_not_cleared = !has_cleared
                                || (gap_clr < cfg_.min_cleared_overhang_span && gap_clr < 0.5f * std::max(gap, 0.0f));
                            if (gap_not_cleared) {
                                if (stacked_upper_z.empty()) {
                                    stacked_upper_z.assign(n_cells, kNan);
                                    stacked_overhang_z.assign(n_cells, kInf);
                                }
                                stacked_upper_z[idx] = u_cand;
                                stacked_overhang_z[idx] = u_ov;
                                ++stacked_count;
                            }
                        }
                    }
                }
                // 净空不足：mergeSpans 已保证上下段之间存在 > gap_fill_distance 的真实空气间隙。
                // 机器人无法从下方钻过低矮悬空结构，也不能把其顶面当作可行驶地面（否则 3x3 均匀低杆顶会在阶段 4
                // 被误判为平整地面）。因此整列先判为硬障碍（宽平台候选交阶段 1.5B 统一裁决）。
                col_kind[idx] = ColKind::HARD_OCC;
                hard_blocked = true;
                break;
            } else {
                // 若射线扫空证据表明最低占据段下方悬空，不能直接将悬空结构顶面当成地面，
                // 留给阶段 2 核验净空 >= min_overhang_clearance；不足则保持 OCCUPIED（防止无地面低杆顶误放）。
                if (!grid.is_solid_mesh && has_cleared_below[idx] != 0) {
                    col_kind[idx] = ColKind::MAYBE_OVERHANG;
                    break;
                }
                col_kind[idx] = ColKind::CANDIDATE_GROUND;
                raw_ground[idx] = g_cand;
                overhang_z[idx] = kInf;
                found_ground = true;
                break;
            }
        }

        if (!found_ground) {
            if (hit_vertical_wall || hard_blocked) {
                col_kind[idx] = ColKind::HARD_OCC;
            } else {
                col_kind[idx] = ColKind::MAYBE_OVERHANG;
            }
        }
    }

    const float tan_max_slope = std::tan(cfg_.max_slope_deg * kDegToRad);

    // 阶段 1.5A：将下方已被射线扫空且底面 5x5 局部水平一致的 HARD_OCC 悬空列（如较厚或分层的无地面隧道限高杆）
    // 降级为 MAYBE_OVERHANG 交给阶段 2 核验净空；底面倾斜（斜坡背面/斜拱）或孤立墙柱保持 HARD_OCC。
    if (!grid.is_solid_mesh && has_cleared && any_cleared_below) {
        std::vector<size_t> promote_indices;
        for (int ix = 0; ix < nx; ++ix) {
            const size_t row = static_cast<size_t>(ix) * static_cast<size_t>(ny);
            for (int iy = 0; iy < ny; ++iy) {
                const size_t idx = row + static_cast<size_t>(iy);
                if (col_kind[idx] != ColKind::HARD_OCC || has_cleared_below[idx] == 0) {
                    continue;
                }
                const float lo = lowest_occ_z[idx];
                if (!std::isfinite(lo)) {
                    continue;
                }
                float min_lo = lo;
                float max_lo = lo;
                int same_underside_cnt = 0;
                bool flat_ok = true;
                const int x_lo = std::max(0, ix - 2);
                const int x_hi = std::min(nx - 1, ix + 2);
                const int y_lo = std::max(0, iy - 2);
                const int y_hi = std::min(ny - 1, iy + 2);
                for (int jx = x_lo; jx <= x_hi && flat_ok; ++jx) {
                    const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ny);
                    for (int jy = y_lo; jy <= y_hi; ++jy) {
                        const size_t jidx = jrow + static_cast<size_t>(jy);
                        const float zz = lowest_occ_z[jidx];
                        if (!std::isfinite(zz) || zz < lo - cfg_.max_surface_thickness) {
                            continue;
                        }
                        const float d_xy = std::hypot(static_cast<float>(jx - ix), static_cast<float>(jy - iy)) * res;
                        if (std::abs(zz - lo) > d_xy * tan_max_slope + cfg_.step_tol) {
                            flat_ok = false;
                            break;
                        }
                        min_lo = std::min(min_lo, zz);
                        max_lo = std::max(max_lo, zz);
                        if (std::abs(zz - lo) <= 0.06f) {
                            ++same_underside_cnt;
                        }
                    }
                }
                if (flat_ok && (max_lo - min_lo) <= 0.06f && same_underside_cnt >= 2) {
                    promote_indices.push_back(idx);
                }
            }
        }
        for (const size_t idx: promote_indices) {
            col_kind[idx] = ColKind::MAYBE_OVERHANG;
        }
    }

    // 阶段 1.5B：点云模式宽平高台恢复（解决空心高台底缝透射低地漏点导致整个台面被标成 HARD_OCC 的问题）。
    // 仅从 stacked_upper_z 候选播种 8 邻域同高连通分量，经 1 格闭运算 + 切比雪夫距离变换确认具备可站立宽度
    // （直径 >= 0.35m，排除 <= 0.25m 的矮限高杆与窄障碍）后，恢复为上层台面 CANDIDATE_GROUND，
    // 并修补台面内部被 4 向平台格包围的 1-2 格低地漏点/未观测扫描缝（绝不覆盖 HARD_OCC 墙/柱/台沿）。
    if (!grid.is_solid_mesh && stacked_count > 0) {
        const int min_dt = std::max(3, static_cast<int>(std::ceil(0.175f / res - 1e-4f)));
        const size_t min_comp_cells = static_cast<size_t>((2 * min_dt - 1) * (2 * min_dt - 1) / 2);
        const size_t min_stacked_cells = std::max<size_t>(4, min_comp_cells / 2);

        std::vector<uint8_t> visited(n_cells, 0);
        std::vector<size_t> comp_q;
        std::vector<size_t> comp_cells;
        std::vector<uint8_t> local_mask;
        std::vector<uint8_t> dil_mask;
        std::vector<uint8_t> closed_mask;
        std::vector<uint8_t> in_plat;
        std::vector<int> dt_buf;
        std::vector<std::pair<size_t, float>> hole_fills;
        std::vector<float> bracket_vals;
        bracket_vals.reserve(4);

        for (size_t seed = 0; seed < n_cells; ++seed) {
            if (!std::isfinite(stacked_upper_z[seed]) || visited[seed] != 0) {
                continue;
            }
            const float z_seed = stacked_upper_z[seed];
            comp_q.clear();
            comp_cells.clear();
            comp_q.push_back(seed);
            visited[seed] = 1;
            size_t q_head = 0;
            size_t n_stacked = 0;
            int min_x = static_cast<int>(seed / static_cast<size_t>(ny));
            int max_x = min_x;
            int min_y = static_cast<int>(seed % static_cast<size_t>(ny));
            int max_y = min_y;

            while (q_head < comp_q.size()) {
                const size_t u = comp_q[q_head++];
                const int ux = static_cast<int>(u / static_cast<size_t>(ny));
                const int uy = static_cast<int>(u % static_cast<size_t>(ny));
                const float zu = std::isfinite(stacked_upper_z[u]) ? stacked_upper_z[u] : raw_ground[u];
                comp_cells.push_back(u);
                if (std::isfinite(stacked_upper_z[u])) {
                    ++n_stacked;
                }
                min_x = std::min(min_x, ux);
                max_x = std::max(max_x, ux);
                min_y = std::min(min_y, uy);
                max_y = std::max(max_y, uy);

                const int jx_lo = std::max(0, ux - 1);
                const int jx_hi = std::min(nx - 1, ux + 1);
                const int jy_lo = std::max(0, uy - 1);
                const int jy_hi = std::min(ny - 1, uy + 1);
                for (int jx = jx_lo; jx <= jx_hi; ++jx) {
                    const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ny);
                    for (int jy = jy_lo; jy <= jy_hi; ++jy) {
                        if (jx == ux && jy == uy) {
                            continue;
                        }
                        const size_t v = jrow + static_cast<size_t>(jy);
                        if (visited[v] != 0) {
                            continue;
                        }
                        float zv = kNan;
                        if (std::isfinite(stacked_upper_z[v])) {
                            zv = stacked_upper_z[v];
                        } else if (col_kind[v] == ColKind::CANDIDATE_GROUND && has_cleared_below[v] == 0
                                   && std::isfinite(raw_ground[v]))
                        {
                            zv = raw_ground[v];
                        }
                        if (std::isfinite(zv) && std::abs(zv - zu) <= cfg_.same_surface_dz
                            && std::abs(zv - z_seed) <= 1.5f * cfg_.same_surface_dz)
                        {
                            visited[v] = 1;
                            comp_q.push_back(v);
                        }
                    }
                }
            }

            if (n_stacked < min_stacked_cells || comp_cells.size() < min_comp_cells) {
                continue;
            }

            const int bx0 = std::max(0, min_x - 2);
            const int bx1 = std::min(nx - 1, max_x + 2);
            const int by0 = std::max(0, min_y - 2);
            const int by1 = std::min(ny - 1, max_y + 2);
            const int lx = bx1 - bx0 + 1;
            const int ly = by1 - by0 + 1;
            const size_t ln = static_cast<size_t>(lx) * static_cast<size_t>(ly);
            local_mask.assign(ln, 0);
            for (const size_t u: comp_cells) {
                const int ux = static_cast<int>(u / static_cast<size_t>(ny)) - bx0;
                const int uy = static_cast<int>(u % static_cast<size_t>(ny)) - by0;
                local_mask[static_cast<size_t>(ux) * static_cast<size_t>(ly) + static_cast<size_t>(uy)] = 1;
            }

            // 1 格形态学闭运算（膨胀后腐蚀，保留原前景），弥合 1-2 格扫描线缝后再评估有效站立宽度
            dil_mask.assign(ln, 0);
            for (int ix = 0; ix < lx; ++ix) {
                const int x0 = std::max(0, ix - 1);
                const int x1 = std::min(lx - 1, ix + 1);
                for (int iy = 0; iy < ly; ++iy) {
                    const int y0 = std::max(0, iy - 1);
                    const int y1 = std::min(ly - 1, iy + 1);
                    uint8_t any_on = 0;
                    for (int jx = x0; jx <= x1 && any_on == 0; ++jx) {
                        const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ly);
                        for (int jy = y0; jy <= y1; ++jy) {
                            if (local_mask[jrow + static_cast<size_t>(jy)] != 0) {
                                any_on = 1;
                                break;
                            }
                        }
                    }
                    dil_mask[static_cast<size_t>(ix) * static_cast<size_t>(ly) + static_cast<size_t>(iy)] = any_on;
                }
            }
            closed_mask.assign(ln, 0);
            for (int ix = 0; ix < lx; ++ix) {
                const int x0 = std::max(0, ix - 1);
                const int x1 = std::min(lx - 1, ix + 1);
                for (int iy = 0; iy < ly; ++iy) {
                    const size_t lidx = static_cast<size_t>(ix) * static_cast<size_t>(ly) + static_cast<size_t>(iy);
                    if (local_mask[lidx] != 0) {
                        closed_mask[lidx] = 1;
                        continue;
                    }
                    const int y0 = std::max(0, iy - 1);
                    const int y1 = std::min(ly - 1, iy + 1);
                    uint8_t all_on = 1;
                    for (int jx = x0; jx <= x1 && all_on != 0; ++jx) {
                        const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ly);
                        for (int jy = y0; jy <= y1; ++jy) {
                            if (dil_mask[jrow + static_cast<size_t>(jy)] == 0) {
                                all_on = 0;
                                break;
                            }
                        }
                    }
                    closed_mask[lidx] = all_on;
                }
            }

            if (chebyshevDt2d(closed_mask, lx, ly, dt_buf) < min_dt) {
                continue;
            }

            in_plat.assign(ln, 0);
            for (const size_t u: comp_cells) {
                const int ux = static_cast<int>(u / static_cast<size_t>(ny)) - bx0;
                const int uy = static_cast<int>(u % static_cast<size_t>(ny)) - by0;
                in_plat[static_cast<size_t>(ux) * static_cast<size_t>(ly) + static_cast<size_t>(uy)] = 1;
                if (std::isfinite(stacked_upper_z[u])) {
                    col_kind[u] = ColKind::CANDIDATE_GROUND;
                    raw_ground[u] = stacked_upper_z[u];
                    overhang_z[u] = stacked_overhang_z[u];
                }
            }

            // 修补被确认平台 4 向包围（<= 2 格内）的极小内部低地漏点或未观测孔洞，不触碰 HARD_OCC / MAYBE_OVERHANG
            constexpr int kDirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            hole_fills.clear();
            for (int ix = min_x; ix <= max_x; ++ix) {
                const size_t row = static_cast<size_t>(ix) * static_cast<size_t>(ny);
                for (int iy = min_y; iy <= max_y; ++iy) {
                    const size_t lidx =
                        static_cast<size_t>(ix - bx0) * static_cast<size_t>(ly) + static_cast<size_t>(iy - by0);
                    if (in_plat[lidx] != 0) {
                        continue;
                    }
                    const size_t u = row + static_cast<size_t>(iy);
                    if (col_kind[u] == ColKind::HARD_OCC || col_kind[u] == ColKind::MAYBE_OVERHANG) {
                        continue;
                    }
                    bracket_vals.clear();
                    bool ok4 = true;
                    for (int d = 0; d < 4; ++d) {
                        bool found = false;
                        for (int step = 1; step <= 2; ++step) {
                            const int jx = ix + kDirs[d][0] * step;
                            const int jy = iy + kDirs[d][1] * step;
                            if (jx < bx0 || jx > bx1 || jy < by0 || jy > by1) {
                                break;
                            }
                            const size_t jlx = static_cast<size_t>(jx - bx0);
                            const size_t jly = static_cast<size_t>(jy - by0);
                            if (in_plat[jlx * static_cast<size_t>(ly) + jly] != 0) {
                                const size_t jidx =
                                    static_cast<size_t>(jx) * static_cast<size_t>(ny) + static_cast<size_t>(jy);
                                bracket_vals.push_back(raw_ground[jidx]);
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            ok4 = false;
                            break;
                        }
                    }
                    if (!ok4) {
                        continue;
                    }
                    const float gz = medianOf(bracket_vals);
                    if (col_kind[u] == ColKind::UNOBSERVED
                        || (col_kind[u] == ColKind::CANDIDATE_GROUND && std::isfinite(raw_ground[u])
                            && raw_ground[u] < gz - cfg_.gap_fill_distance))
                    {
                        hole_fills.emplace_back(u, gz);
                    }
                }
            }
            for (const auto& [u, gz]: hole_fills) {
                col_kind[u] = ColKind::CANDIDATE_GROUND;
                raw_ground[u] = gz;
                overhang_z[u] = kInf;
            }
        }
    }

    // 阶段 2：悬空列恢复（仅在有射线扫空证据或高空天花板夹于双侧地面之间时触发，不误放实心高台/矮墙）。
    if (cfg_.overhang_ground_radius > 0) {
        const std::vector<ColKind> kind_snap = col_kind;
        const std::vector<float> ground_snap = raw_ground;
        std::vector<float> ring_vals;
        ring_vals.reserve(static_cast<size_t>(8 * cfg_.overhang_ground_radius));
        std::vector<uint8_t> ray_recovered;
        std::vector<size_t> bfs_q;
        if (!grid.is_solid_mesh && has_cleared && any_cleared_below) {
            ray_recovered.assign(n_cells, 0);
        }

        for (int ix = 0; ix < nx; ++ix) {
            for (int iy = 0; iy < ny; ++iy) {
                const size_t idx = static_cast<size_t>(ix) * static_cast<size_t>(ny) + static_cast<size_t>(iy);
                const float z_lo = lowest_occ_z[idx];
                if (!std::isfinite(z_lo)) {
                    continue;
                }

                // 情况 A：射线直接扫空了最低占据下方（含阶段 1.5A 降级后的平底厚悬空结构）
                const bool ray_confirmed_overhang =
                    (!grid.is_solid_mesh && has_cleared_below[idx] != 0
                     && (kind_snap[idx] == ColKind::MAYBE_OVERHANG || kind_snap[idx] == ColKind::CANDIDATE_GROUND));
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
                            if (!ray_recovered.empty()) {
                                ray_recovered[idx] = 1;
                                bfs_q.push_back(idx);
                            }
                        }
                    } else if ((has_pos_x && has_neg_x) || (has_pos_y && has_neg_y)) {
                        col_kind[idx] = ColKind::CANDIDATE_GROUND;
                        raw_ground[idx] = ref_g;
                        overhang_z[idx] = z_lo;
                    }
                }
            }
        }

        // 宽无地面隧道 BFS 传播：仅沿已通过射线净空核验的悬空邻格传播参考地面高度（每格仍独立校验净空与真实扫空厚度），
        // 解除 overhang_ground_radius 对长/宽无地面隧道深处的限制，并在墙体/低杆/无扫空盲区处自然截断。
        if (!bfs_q.empty()) {
            size_t q_head = 0;
            while (q_head < bfs_q.size()) {
                const size_t u = bfs_q[q_head++];
                const int ux = static_cast<int>(u / static_cast<size_t>(ny));
                const int uy = static_cast<int>(u % static_cast<size_t>(ny));
                const float ref_g = raw_ground[u];
                const int jx_lo = std::max(0, ux - 1);
                const int jx_hi = std::min(nx - 1, ux + 1);
                const int jy_lo = std::max(0, uy - 1);
                const int jy_hi = std::min(ny - 1, uy + 1);
                for (int jx = jx_lo; jx <= jx_hi; ++jx) {
                    const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ny);
                    for (int jy = jy_lo; jy <= jy_hi; ++jy) {
                        if (jx == ux && jy == uy) {
                            continue;
                        }
                        const size_t v = jrow + static_cast<size_t>(jy);
                        if (ray_recovered[v] != 0 || has_cleared_below[v] == 0) {
                            continue;
                        }
                        if (kind_snap[v] != ColKind::MAYBE_OVERHANG && kind_snap[v] != ColKind::CANDIDATE_GROUND) {
                            continue;
                        }
                        const float z_lo = lowest_occ_z[v];
                        if (!std::isfinite(z_lo) || (z_lo - ref_g < cfg_.min_overhang_clearance)) {
                            continue;
                        }
                        merged_clr = grid.cleared_spans[v];
                        mergeSpans(merged_clr, 0.01f);
                        const float real_clr = clearedCoverage(merged_clr, ref_g + 0.03f, z_lo - 0.02f);
                        if (real_clr >= cfg_.min_cleared_overhang_span) {
                            col_kind[v] = ColKind::CANDIDATE_GROUND;
                            raw_ground[v] = ref_g;
                            overhang_z[v] = z_lo;
                            ray_recovered[v] = 1;
                            bfs_q.push_back(v);
                        }
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
                        plane_pts.push_back(SamplePt {static_cast<float>(dx) * res, static_cast<float>(dy) * res, gn});
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
