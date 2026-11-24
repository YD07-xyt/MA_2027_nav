#pragma once

#include "controller/traj_interface.hpp"
#include "map/core/grid_map.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>

namespace control {

struct FoldParams {
    double fold_time = 1.0; // 云台物理折叠耗时 (s)，>= 0
    double unfold_time = 1.0; // 云台物理展开耗时 (s)，>= 0
    double fold_margin = 0.5; // 提前折叠与合并相邻隧道的安全时间裕量 (s)，>= 0
    double gimbal_radius = 0.3; // 禁入区 T_in 外扩半径 = 云台包络半径 + 定位误差 (m)，>= 0
    double unfold_hysteresis = 0.2; // 出洞滞回外扩量 (m)，>= 0；T_out = gimbal_radius + unfold_hysteresis
    double unfold_confirm_time = 0.4; // 洞外且预瞄无隧道后触发抬升的去抖时间 (s)，>= 0
    double feedback_timeout = 0.3; // 下位机反馈超时阈值 (s)，> 0
    double fold_timeout_margin = 1.0; // 折叠超时告警裕量 (s)，>= 0
    double gate_brake_acc = 3.0; // 虚拟门禁制动速度包络减速度 (m/s²)，> 0
    double gate_stop_buffer = 0.15; // 门禁停靠时距禁入区边界的停车缓冲距离 (m)，>= 0
    double lookahead_dt = 0.05; // 沿当前轨迹前瞻采样步长 (s)，> 0
    double cmd_pub_rate = 20.0; // 折叠电平心跳发布频率 (Hz)，> 0
};

struct FoldDecision {
    bool fold_cmd = false; // 期望下发电平：true=折叠锁定，false=允许展开
    bool cmd_changed = false; // 相对上一周期 fold_cmd 是否翻转
    bool confirmed_folded = false; // 反馈新鲜且物理确认已折叠到位
    bool feedback_stale = true; // 反馈缺失或超过 feedback_timeout
    bool fold_timeout = false; // 持续请求折叠超过 fold_time + fold_timeout_margin 仍未到位
    bool gate_active = false; // 门禁锁定：未确认折叠到位且身处或逼近禁入区 T_in
    double max_speed = std::numeric_limits<double>::infinity(); // 门禁允许的最大平面速率 (m/s)
};

// 闭环云台折叠状态机与虚拟门禁（纯核心层，零 ROS 依赖）：
// - update() 仅由控制线程单一调用，on_feedback() 线程安全，可由订阅回调并发写入；
// - 基于 GridMap::semanticsData() 按语义内容惰性缓存精确 2D 隧道欧氏距离场，不修改地图模块；
// - 只要反馈超时或未确认折叠到位，洞口前施加运动学制动包络，最晚在禁入区外稳停等待。
class FoldGuard {
public:
    FoldGuard() = default;
    explicit FoldGuard(const FoldParams& params): params_(params) {}

    auto set_param(const FoldParams& params) -> void {
        params_ = params;
    }

    auto get_param() const -> const FoldParams& {
        return params_;
    }

    // 线程安全：写入最新下位机物理折叠反馈（true=已确认折叠到位 DOWN，false=未到位/运动中/故障）。
    // stamp_sec 须与 update() 的 now_sec 处于同一单调/仿真时钟域。
    auto on_feedback(bool is_folded, double stamp_sec) -> void {
        if (!std::isfinite(stamp_sec)) return;
        std::lock_guard<std::mutex> lock(fb_mtx_);
        fb_has_value_ = true;
        fb_is_folded_ = is_folded;
        fb_stamp_sec_ = stamp_sec;
    }

    auto reset() -> void {
        fold_cmd_ = false;
        unfold_clear_since_sec_.reset();
        fold_wait_since_sec_.reset();
    }

    // 控制周期入口：同步借用 map 与 traj 只读指针（均可为空），输出当前折叠电平与速度门禁。
    auto update(
        const grid_map::GridMap* map,
        const MaSplineTrajectoryInterface* traj,
        double t_track,
        const Eigen::Vector2d& pos,
        double now_sec
    ) -> FoldDecision {
        FoldDecision out {};
        if (!std::isfinite(now_sec)) {
            out.fold_cmd = fold_cmd_;
            return out;
        }

        // 1. 读取下位机反馈快照并检查新鲜度（兼容极小跨线程抖动，仿真时钟回拨按过期处理）
        bool has_fb = false;
        bool fb_folded = false;
        double fb_stamp = 0.0;
        {
            std::lock_guard<std::mutex> lock(fb_mtx_);
            has_fb = fb_has_value_;
            fb_folded = fb_is_folded_;
            fb_stamp = fb_stamp_sec_;
        }
        const double fb_age = (!has_fb || now_sec < fb_stamp - 0.05) ? std::numeric_limits<double>::infinity()
                                                                     : std::max(0.0, now_sec - fb_stamp);
        out.feedback_stale = !has_fb || (fb_age > params_.feedback_timeout);
        out.confirmed_folded = !out.feedback_stale && fb_folded;

        if (!map || !pos.allFinite()) {
            out.fold_cmd = fold_cmd_;
            return out;
        }

        // 2. 按语义层变化惰性更新 2D 隧道距离场
        sync_tunnel_cache(*map);

        const double r_in = std::max(0.0, params_.gimbal_radius);
        const double r_out = r_in + std::max(0.0, params_.unfold_hysteresis);
        const double d_now = dist_to_tunnel(*map, pos);
        const bool in_t_in = has_tunnel_ && (d_now <= r_in);
        const bool in_t_out = has_tunnel_ && (d_now <= r_out);

        // 3. 沿当前完整轨迹（刹车段 + 样条段）前瞻下一次进入 T_in 的剩余时间与弧长
        const double t_trigger = std::max(0.0, params_.fold_time) + std::max(0.0, params_.fold_margin);
        const double t_merge = std::max(0.0, params_.fold_time) + std::max(0.0, params_.unfold_time)
            + 2.0 * std::max(0.0, params_.fold_margin);
        const double a_brake = std::max(params_.gate_brake_acc, 1e-3);
        const double t_gate_horizon = 4.0 / std::max(a_brake, 0.5) + 0.5;
        const double t_horizon = std::max({t_trigger, t_merge, t_gate_horizon});

        double dt_enter = std::numeric_limits<double>::infinity();
        double s_rem = std::numeric_limits<double>::infinity();
        Eigen::Vector2d entry_pt = pos;

        if (has_tunnel_ && traj && traj->valid()) {
            const double dur = traj->duration();
            const double t0 = std::clamp(t_track, 0.0, dur);
            const double t_end = std::min(dur, t0 + t_horizon);
            const double step = std::max(params_.lookahead_dt, 0.01);

            TrajectoryTypes::ReferencePoint ref0;
            if (traj->sample(t0, ref0)) {
                Eigen::Vector2d prev_p(ref0.state(0), ref0.state(1));
                double prev_t = t0;
                double prev_d = dist_to_tunnel(*map, prev_p);
                if (prev_d <= r_in) {
                    dt_enter = 0.0;
                    s_rem = std::max(0.0, d_now - r_in);
                    entry_pt = prev_p;
                } else {
                    double arc = 0.0;
                    for (double t = t0 + step; t <= t_end + 1e-9; t += step) {
                        const double ts = std::min(t, t_end);
                        TrajectoryTypes::ReferencePoint ref;
                        if (!traj->sample(ts, ref)) break;

                        const Eigen::Vector2d cur_p(ref.state(0), ref.state(1));
                        const double seg_len = (cur_p - prev_p).norm();
                        const double cur_d = dist_to_tunnel(*map, cur_p);

                        if (cur_d <= r_in) {
                            const double denom = std::max(prev_d - cur_d, 1e-6);
                            const double alpha = std::clamp((prev_d - r_in) / denom, 0.0, 1.0);
                            dt_enter = (prev_t - t0) + alpha * (ts - prev_t);
                            s_rem = arc + alpha * seg_len;
                            entry_pt = prev_p + alpha * (cur_p - prev_p);
                            break;
                        }

                        arc += seg_len;
                        prev_p = cur_p;
                        prev_t = ts;
                        prev_d = cur_d;
                        if (ts >= t_end - 1e-9) break;
                    }
                }
            }
        }

        // 4. 幂等电平决策（进入触发 + 出洞滞回 + 重规划去抖）
        const bool prev_cmd = fold_cmd_;
        const bool need_fold = in_t_in || (dt_enter <= t_trigger);
        const bool clear_to_unfold = !in_t_out && (dt_enter > t_merge);

        if (need_fold) {
            fold_cmd_ = true;
            unfold_clear_since_sec_.reset();
        } else if (fold_cmd_) {
            if (clear_to_unfold) {
                if (!unfold_clear_since_sec_.has_value() || now_sec < unfold_clear_since_sec_.value()) {
                    unfold_clear_since_sec_ = now_sec;
                }
                if (now_sec - unfold_clear_since_sec_.value() >= params_.unfold_confirm_time) {
                    fold_cmd_ = false;
                    unfold_clear_since_sec_.reset();
                }
            } else {
                unfold_clear_since_sec_.reset();
            }
        } else {
            unfold_clear_since_sec_.reset();
        }

        out.fold_cmd = fold_cmd_;
        out.cmd_changed = (fold_cmd_ != prev_cmd);

        // 5. 折叠超时检测（持续要求折叠却迟迟未确认到位）
        if (fold_cmd_ && !out.confirmed_folded) {
            if (!fold_wait_since_sec_.has_value() || now_sec < fold_wait_since_sec_.value()) {
                fold_wait_since_sec_ = now_sec;
            }
            const double max_wait = std::max(0.0, params_.fold_time) + std::max(0.0, params_.fold_timeout_margin);
            out.fold_timeout = (now_sec - fold_wait_since_sec_.value()) > max_wait;
        } else {
            fold_wait_since_sec_.reset();
            out.fold_timeout = false;
        }

        // 6. 虚拟门禁安全盾：未确认折叠到位时，洞内即停、洞口前按运动学制动包络限速直至稳停
        if (!out.confirmed_folded) {
            if (in_t_in) {
                out.gate_active = true;
                out.max_speed = 0.0;
            } else if (std::isfinite(s_rem)) {
                const double s_to_gate = std::min(s_rem, (entry_pt - pos).norm());
                const double s_eff = std::max(0.0, s_to_gate - std::max(0.0, params_.gate_stop_buffer));
                out.gate_active = true;
                out.max_speed = std::sqrt(2.0 * a_brake * s_eff);
            }
        }

        return out;
    }

    // 将门禁制动速度包络作用于世界系速度指令；返回 true 表示指令被限幅/截停。
    static auto apply_velocity_gate(Eigen::Vector2d& vel_world, const FoldDecision& decision) -> bool {
        if (!decision.gate_active) {
            return false;
        }
        if (decision.max_speed <= 1e-6) {
            vel_world.setZero();
            return true;
        }
        const double speed = vel_world.norm();
        if (speed > decision.max_speed) {
            vel_world *= (decision.max_speed / speed);
            return true;
        }
        return false;
    }

private:
    static constexpr double kInfSq = 1e20;

    // 1D 精确平方欧氏距离变换（Felzenszwalb & Huttenlocher, O(n)）
    static auto
    edt_1d(const std::vector<double>& f, int n, std::vector<double>& d, std::vector<int>& v, std::vector<double>& z)
        -> void {
        if (n <= 0) return;
        int k = 0;
        v[0] = 0;
        z[0] = -std::numeric_limits<double>::infinity();
        z[1] = std::numeric_limits<double>::infinity();

        for (int q = 1; q < n; ++q) {
            if (f[q] >= kInfSq * 0.5) continue;
            while (true) {
                const int r = v[k];
                if (f[r] >= kInfSq * 0.5) {
                    v[0] = q;
                    z[0] = -std::numeric_limits<double>::infinity();
                    z[1] = std::numeric_limits<double>::infinity();
                    break;
                }
                const double s = ((f[q] + static_cast<double>(q) * q) - (f[r] + static_cast<double>(r) * r))
                    / (2.0 * static_cast<double>(q - r));
                if (s <= z[k]) {
                    --k;
                } else {
                    ++k;
                    v[k] = q;
                    z[k] = s;
                    z[k + 1] = std::numeric_limits<double>::infinity();
                    break;
                }
            }
        }

        if (f[v[0]] >= kInfSq * 0.5) {
            for (int q = 0; q < n; ++q) {
                d[q] = kInfSq;
            }
            return;
        }

        int idx = 0;
        for (int q = 0; q < n; ++q) {
            while (z[idx + 1] < static_cast<double>(q)) {
                ++idx;
            }
            const double dq = static_cast<double>(q - v[idx]);
            d[q] = dq * dq + f[v[idx]];
        }
    }

    auto sync_tunnel_cache(const grid_map::GridMap& map) -> void {
        const Eigen::Vector2i vnum = map.getVoxelNum();
        const int nx = vnum.x();
        const int ny = vnum.y();
        const double res = map.getResolution();
        const uint64_t rev = map.revision();

        if (nx <= 0 || ny <= 0 || res <= 0.0) {
            has_tunnel_ = false;
            return;
        }

        const auto sem = map.semanticsData();
        const size_t n = static_cast<size_t>(nx) * static_cast<size_t>(ny);
        if (sem.size() != n) {
            has_tunnel_ = false;
            return;
        }

        // 快速路径：版本号、尺寸与分辨率均未变化
        if (cache_valid_ && cached_revision_ == rev && cached_nx_ == nx && cached_ny_ == ny && cached_res_ == res) {
            return;
        }

        // 动态障碍更新会递增 revision()，但隧道语义未变时直接跳过 EDT 重算
        const bool same_geom = cache_valid_ && cached_nx_ == nx && cached_ny_ == ny && cached_res_ == res;
        if (same_geom && cached_semantics_.size() == n && std::equal(sem.begin(), sem.end(), cached_semantics_.begin()))
        {
            cached_revision_ = rev;
            return;
        }

        cached_nx_ = nx;
        cached_ny_ = ny;
        cached_res_ = res;
        cached_revision_ = rev;
        cache_valid_ = true;
        cached_semantics_.assign(sem.begin(), sem.end());

        const auto tunnel_val = static_cast<uint8_t>(grid_map::GridMap::Semantics::TUNNEL);
        bool any_tunnel = false;
        for (uint8_t s: cached_semantics_) {
            if (s == tunnel_val) {
                any_tunnel = true;
                break;
            }
        }
        has_tunnel_ = any_tunnel;
        if (!has_tunnel_) {
            tunnel_dist_.clear();
            return;
        }

        std::vector<double> tmp_sq(n, kInfSq);
        const int max_dim = std::max(nx, ny);
        std::vector<double> f(static_cast<size_t>(max_dim), kInfSq);
        std::vector<double> d(static_cast<size_t>(max_dim), kInfSq);
        std::vector<int> v(static_cast<size_t>(max_dim), 0);
        std::vector<double> z(static_cast<size_t>(max_dim + 1), 0.0);

        // Pass 1: 沿 y 方向做 1D EDT
        for (int x = 0; x < nx; ++x) {
            const size_t base = static_cast<size_t>(x) * static_cast<size_t>(ny);
            for (int y = 0; y < ny; ++y) {
                f[static_cast<size_t>(y)] =
                    (cached_semantics_[base + static_cast<size_t>(y)] == tunnel_val) ? 0.0 : kInfSq;
            }
            edt_1d(f, ny, d, v, z);
            for (int y = 0; y < ny; ++y) {
                tmp_sq[base + static_cast<size_t>(y)] = d[static_cast<size_t>(y)];
            }
        }

        // Pass 2: 沿 x 方向做 1D EDT 并换算到物理距离 (m)
        tunnel_dist_.resize(n);
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                f[static_cast<size_t>(x)] =
                    tmp_sq[static_cast<size_t>(x) * static_cast<size_t>(ny) + static_cast<size_t>(y)];
            }
            edt_1d(f, nx, d, v, z);
            for (int x = 0; x < nx; ++x) {
                const double sq = d[static_cast<size_t>(x)];
                tunnel_dist_[static_cast<size_t>(x) * static_cast<size_t>(ny) + static_cast<size_t>(y)] =
                    (sq < kInfSq * 0.5) ? (std::sqrt(sq) * res) : std::numeric_limits<double>::infinity();
            }
        }
    }

    auto dist_to_tunnel(const grid_map::GridMap& map, const Eigen::Vector2d& pos) const -> double {
        if (!has_tunnel_ || tunnel_dist_.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        Eigen::Vector2i idx;
        map.posToIndex(pos, idx);
        const Eigen::Vector2i clamped(std::clamp(idx.x(), 0, cached_nx_ - 1), std::clamp(idx.y(), 0, cached_ny_ - 1));
        const double cell_d = tunnel_dist_
            [static_cast<size_t>(clamped.x()) * static_cast<size_t>(cached_ny_) + static_cast<size_t>(clamped.y())];
        if (idx == clamped) {
            return cell_d;
        }
        Eigen::Vector2d clamped_pos;
        map.indexToPos(clamped, clamped_pos);
        return cell_d + (pos - clamped_pos).norm();
    }

    FoldParams params_;

    mutable std::mutex fb_mtx_;
    bool fb_has_value_ = false;
    bool fb_is_folded_ = false;
    double fb_stamp_sec_ = 0.0;

    bool fold_cmd_ = false;
    std::optional<double> unfold_clear_since_sec_;
    std::optional<double> fold_wait_since_sec_;

    bool cache_valid_ = false;
    bool has_tunnel_ = false;
    uint64_t cached_revision_ = 0;
    int cached_nx_ = 0;
    int cached_ny_ = 0;
    double cached_res_ = 0.0;
    std::vector<uint8_t> cached_semantics_;
    std::vector<double> tunnel_dist_;
};

} // namespace control
