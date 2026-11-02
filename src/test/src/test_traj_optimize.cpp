/*
 * 轨迹优化（MaSplineTrajectoryOptimizer，两阶段 L-BFGS）测试
 *
 *  1. 读取 images/ 下的黑白地图(黑色 = 障碍) -> GridMap (Cv2Map)。
 *  2. 参数取 config/planner.yaml：path_planning_params（搜索/后处理）与 ma_spline_opt_params（优化器），与线上一致。
 *  3. 每张地图用固定随机种子采样若干组 JPS 可达的起终点，按线上顺序 JPS -> 后处理 -> 优化（OMNI_XY）。
 *     起点边界条件两组：
 *       - static   : v0 = a0 = 0（冷启动）
 *       - init_vel : v0 = kInitSpeed 沿剪枝后路径首段方向，a0 = 0（模拟热启动），后处理时间分配同样用该初速度
 *     每组查询优化重复 kRepeat 次取中位数耗时。
 *  4. 每条优化轨迹按 kSampleDt 采样，统计 max|v|、max|a|、最小 ESDF 距离、ESDF < safe_distance 的时长占比。
 *     检查：优化成功、max|v| <= v_max*(1+kDynTol)、max|a| <= a_max*(1+kDynTol)、最小 ESDF >= 0（不穿墙）。
 *     ESDF < safe_distance 只统计不判失败（避障是软约束）。
 *  5. 结果写入 images/output/traj_optimize/：
 *       - <map>_<mode>_traj.png     最长 4 组查询：灰=JPS 原始路径，红点+蓝线=后处理参考点，优化轨迹按速度着色（蓝慢 -> 红快）
 *       - <map>_<mode>_profile.png  最长查询的 |v|、|a|、ESDF 距离随时间曲线（虚线为限幅/安全距离）
 *       - <map>_time.png            每组查询的优化耗时（static / init_vel 对比，对数坐标）
 *       - summary.png               各地图平均优化耗时
 *       - traj_optimize.csv         逐查询原始数据
 *  6. Google Benchmark：每张地图 × 每种起点注册一个用例，一次迭代对全部查询跑一次优化（不含搜索/后处理）。
 */

#include "cv2map.hpp"
#include "cv_show.hpp"
#include "path_planning/post_processing.h"
#include "path_planning/search/jps.h"
#include "traj_optimize/grid_map_esdf.hpp"
#include "traj_optimize/ma_spline_opt/traj_optimizer.h"
#include "utils/logger.hpp"

#include <benchmark/benchmark.h>
#include <spdlog/sinks/base_sink.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif

namespace {

namespace viz = test_utils::viz;
using viz::strf;
using PostProc = path_planning::PathPostProcessing;
using Optimizer = ma_spline_opt::MaSplineTrajectoryOptimizer<minco_opt::GridMapESDF>;

// ---------------- 测试参数 ----------------
constexpr double kResolution = 0.05;  // 560x300 像素 <-> 28m x 15m 场地
constexpr int kNumQueries = 20;       // 每张地图的查询数
constexpr double kMinQueryDist = 8.0; // 起终点最小直线距离(m)
constexpr int kRepeat = 5;            // 每次优化重复次数(取中位数)
constexpr int kTimeoutMs = 2000;
constexpr unsigned kSeed = 42;
constexpr double kDrawScale = 2.0;
constexpr double kSampleDt = 0.01;   // 轨迹检查采样间隔(s)
constexpr double kDynTol = 0.10;     // 速度/加速度超限容差（软约束）
constexpr double kInitSpeed = 1.5;   // init_vel 组的初速度(m/s)

const std::string kImageDir = std::string(ROOT_DIR) + "images/";
const std::string kOutputDir = kImageDir + "output/traj_optimize/";
const std::string kConfigPath = std::string(ROOT_DIR) + "config/planner.yaml";
const std::vector<std::string> kMapNames = {"rmuc2024", "rmuc2025"};

constexpr int kNumModes = 2;
const std::array<std::string, kNumModes> kModeNames = {"static", "init_vel"};
const std::array<cv::Scalar, kNumModes> kModeColors = {viz::kBlue, viz::kOrange};

const cv::Scalar kColorRefPoint = viz::kRed; // 后处理参考点
const cv::Scalar kColorRefLine = viz::kBlue; // 参考点连线
const cv::Scalar kColorRaw = viz::kGray;     // JPS 原始路径

// 统计 traj_optimize 日志中 "keep stage1"（stage2 失败回退）的次数
class CountSink : public spdlog::sinks::base_sink<std::mutex> {
public:
    std::atomic<int> stage2_fallback {0};

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        const std::string_view s(msg.payload.data(), msg.payload.size());
        if (s.find("keep stage1") != std::string_view::npos) ++stage2_fallback;
    }
    void flush_() override {}
};

struct Query {
    Eigen::Vector2d start;
    Eigen::Vector2d goal;
};

struct Metrics {
    bool success = false;
    bool stage2_fallback = false;
    double opt_ms = 0.0;
    double cost = 0.0;
    double duration = 0.0;     // 优化后总时长(s)
    double ref_duration = 0.0; // 后处理参考总时长(s)
    double length = 0.0;       // 优化轨迹长度(m)
    double max_v = 0.0;
    double max_a = 0.0;
    double min_esdf = 0.0;
    double unsafe_ratio = 0.0; // ESDF < safe_distance 的时长占比
    int segments = 0;
    bool v_ok = true, a_ok = true, esdf_ok = true;
    // 采样序列（画图用）
    std::vector<double> t, v, a, d;
    std::vector<Eigen::Vector2d> p;
};

struct Run {
    PostProc::Trajectory ref; // 后处理结果
    ma_spline_opt::MaSplineInput input;
    Metrics m;
};

struct Case {
    std::string name;
    std::unique_ptr<test_utils::Cv2Map> map;
    std::shared_ptr<const grid_map::GridMap> gm;
    std::unique_ptr<minco_opt::GridMapESDF> esdf;
    path_planning::JPS jps;
    std::vector<Query> queries;
    std::vector<std::vector<Eigen::Vector2d>> raw_paths;
    std::array<std::vector<Run>, kNumModes> runs;
};

// ---------------- 参数读取（键与 ros2/config.hpp 一致；测试不链接 ros2 库） ----------------

auto load_post_params(const YAML::Node& n) -> PostProc::PathPostProcessingParams {
    PostProc::PathPostProcessingParams p {};
    p.safe_threshold = 0.35;
    p.max_vel = 3.0;
    p.max_acc = 3.0;
    p.time_resolution = 0.4;
    p.min_traj_num = 5;
    p.traj_cut_length = 10.0;
    p.distance_weight = 1.0;
    p.yaw_weight = 0.5;
    if (!n) throw std::runtime_error("planner_config.path_planning_params not found");
    auto rd = [&](const char* k, double& v) {
        if (n[k]) v = n[k].as<double>();
    };
    auto ri = [&](const char* k, int& v) {
        if (n[k]) v = n[k].as<int>();
    };
    rd("safe_threshold", p.safe_threshold);
    rd("max_vel", p.max_vel);
    rd("max_acc", p.max_acc);
    rd("time_resolution", p.time_resolution);
    ri("min_traj_num", p.min_traj_num);
    ri("max_traj_num", p.max_traj_num);
    rd("traj_cut_length", p.traj_cut_length);
    rd("distance_weight", p.distance_weight);
    rd("yaw_weight", p.yaw_weight);
    rd("dense_sample_resolution", p.dense_sample_resolution);
    rd("rotation_penalty_weight", p.rotation_penalty_weight);
    rd("fold_time", p.fold_time);
    rd("unfold_time", p.unfold_time);
    rd("fold_margin", p.fold_margin);
    rd("fold_prep_speed", p.fold_prep_speed);
    rd("fold_prep_margin", p.fold_prep_margin);
    return p;
}

auto load_stage(const YAML::Node& n, ma_spline_opt::StageOptimizerConfig& s) -> void {
    if (!n) return;
    auto rd = [&](const char* k, double& v) {
        if (n[k]) v = n[k].as<double>();
    };
    auto ri = [&](const char* k, int& v) {
        if (n[k]) v = n[k].as<int>();
    };
    rd("rho_energy", s.rho_energy);
    rd("weight_time", s.weight_time);
    rd("weight_mean_time", s.weight_mean_time);
    rd("mean_lower", s.mean_lower);
    rd("mean_upper", s.mean_upper);
    rd("min_time", s.min_time);
    rd("weight_min_time", s.weight_min_time);
    rd("weight_obstacle", s.weight_obstacle);
    rd("weight_vel", s.weight_vel);
    rd("weight_acc", s.weight_acc);
    ri("max_iterations", s.max_iterations);
    rd("g_epsilon", s.g_epsilon);
    ri("lbfgs_mem_size", s.lbfgs_mem_size);
    rd("lbfgs_delta", s.lbfgs_delta);
}

auto load_opt_config(const YAML::Node& n) -> ma_spline_opt::MaSplineOptimizerConfig {
    ma_spline_opt::MaSplineOptimizerConfig c;
    if (!n) throw std::runtime_error("planner_config.ma_spline_opt_params not found");
    auto rd = [&](const char* k, double& v) {
        if (n[k]) v = n[k].as<double>();
    };
    rd("safe_distance", c.safe_distance);
    rd("v_max", c.v_max);
    rd("a_max", c.a_max);
    if (n["integral_num_steps"]) c.integral_num_steps = n["integral_num_steps"].as<int>();
    rd("weight_yaw_vel", c.weight_yaw_vel);
    rd("weight_yaw_acc", c.weight_yaw_acc);
    rd("yaw_rate_max", c.yaw_rate_max);
    rd("yaw_acc_max", c.yaw_acc_max);
    load_stage(n["stage1"], c.stage1);
    load_stage(n["stage2"], c.stage2);
    return c;
}

// ---------------- 工具 ----------------

auto median(std::vector<double> v) -> double {
    if (v.empty()) return NAN;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

auto ms_since(std::chrono::steady_clock::time_point t0) -> double {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// 采样起终点:自由栅格(ESDF 留出一格余量)、直线距离足够远、JPS 可达。
auto sample_queries(Case& c, double safe, std::mt19937& rng) -> void {
    const auto& m = *c.map;
    const auto& gm = m.grid_map();
    std::vector<Eigen::Vector2d> free_cells;
    for (int row = 0; row < m.height(); ++row) {
        for (int col = 0; col < m.width(); ++col) {
            const Eigen::Vector2d p = m.pixel_to_world(col, row);
            if (gm.getDistance(p) >= safe + kResolution) free_cells.push_back(p);
        }
    }
    if (free_cells.empty()) return;
    std::uniform_int_distribution<size_t> pick(0, free_cells.size() - 1);
    for (int attempt = 0; attempt < kNumQueries * 200 && static_cast<int>(c.queries.size()) < kNumQueries;
         ++attempt) {
        const Query q {free_cells[pick(rng)], free_cells[pick(rng)]};
        if ((q.goal - q.start).norm() < kMinQueryDist) continue;
        auto path = c.jps.jps_search(q.start, q.goal, kTimeoutMs);
        if (path.size() < 2) continue; // 不连通
        c.queries.push_back(q);
        c.raw_paths.push_back(std::move(path));
    }
}

// 后处理（与 path_planning.cpp 顺序一致），start_vel 为起点速度向量
auto post_process(const grid_map::GridMap& gm, const PostProc::PathPostProcessingParams& params, const Query& q,
                  const std::vector<Eigen::Vector2d>& raw, const Eigen::Vector2d& start_vel) -> PostProc::Trajectory {
    PostProc post;
    post.set_map(gm);
    post.set_params(params);
    post.set_start_vel(start_vel.norm());
    post.set_end_vel(0.0);
    post.set_start_velocity_vector(Eigen::Vector3d(start_vel.x(), start_vel.y(), 0.0));
    post.set_end_velocity_vector(Eigen::Vector3d::Zero());

    PostProc::Trajectory traj;
    traj.raw_path = raw;
    traj.optimized_path = post.optimize_path(traj.raw_path);
    traj.path_states = post.sample_path_states(traj.optimized_path);
    post.assign_trajectory_timing(traj);
    post.fill_additional_trajectory_info(traj, q.start, q.goal, 0.0, 0.0);
    return traj;
}

// 沿剪枝后路径首段方向的单位向量
auto first_dir(const std::vector<Eigen::Vector2d>& path) -> Eigen::Vector2d {
    for (size_t i = 1; i < path.size(); ++i) {
        const Eigen::Vector2d d = path[i] - path[0];
        if (d.norm() > 1e-6) return d.normalized();
    }
    return Eigen::Vector2d::Zero();
}

auto evaluate(const ma_spline_opt::MAsplineOutput& out, const minco_opt::GridMapESDF& esdf,
              const ma_spline_opt::MaSplineOptimizerConfig& cfg, Metrics& m) -> void {
    m.success = out.success && out.trajectory.isInitialized();
    if (!m.success) return;
    m.cost = out.cost;
    m.duration = ma_spline_opt::total_duration(out);
    m.segments = static_cast<int>(out.time_segments.size());
    m.min_esdf = std::numeric_limits<double>::max();
    int unsafe = 0, inside = 0;
    const int n = std::max(1, static_cast<int>(std::ceil(m.duration / kSampleDt)));
    Eigen::Vector2d last;
    for (int i = 0; i <= n; ++i) {
        const double t = std::min(m.duration, i * kSampleDt);
        Eigen::Vector2d p, v, a;
        ma_spline_opt::sample_xy(out, t, p, v, a);
        if (i > 0) m.length += (p - last).norm();
        last = p;
        double d = NAN;
        if (esdf.isInside(p.x(), p.y())) {
            d = esdf.getDistance(p.x(), p.y());
            m.min_esdf = std::min(m.min_esdf, d);
            ++inside;
            if (d < cfg.safe_distance) ++unsafe;
        }
        m.max_v = std::max(m.max_v, v.norm());
        m.max_a = std::max(m.max_a, a.norm());
        m.t.push_back(t);
        m.p.push_back(p);
        m.v.push_back(v.norm());
        m.a.push_back(a.norm());
        m.d.push_back(d);
    }
    m.unsafe_ratio = inside > 0 ? static_cast<double>(unsafe) / inside : 0.0;
    m.v_ok = m.max_v <= cfg.v_max * (1.0 + kDynTol);
    m.a_ok = m.max_a <= cfg.a_max * (1.0 + kDynTol);
    m.esdf_ok = m.min_esdf >= 0.0;
}

auto run_case(Case& c, const PostProc::PathPostProcessingParams& pp, const ma_spline_opt::MaSplineOptimizerConfig& oc,
              CountSink& sink) -> void {
    for (int mode = 0; mode < kNumModes; ++mode) {
        auto& runs = c.runs[mode];
        runs.clear();
        for (size_t qi = 0; qi < c.queries.size(); ++qi) {
            Run r;
            // init_vel：初速度方向取 JPS 路径剪枝后的首段方向（先用零速后处理一次求方向）
            Eigen::Vector2d v0 = Eigen::Vector2d::Zero();
            if (mode == 1) {
                const auto probe = post_process(*c.gm, pp, c.queries[qi], c.raw_paths[qi], Eigen::Vector2d::Zero());
                v0 = kInitSpeed * first_dir(probe.optimized_path);
            }
            r.ref = post_process(*c.gm, pp, c.queries[qi], c.raw_paths[qi], v0);
            r.m.ref_duration = r.ref.total_time;
            r.input = ma_spline_opt::from_path_planning_trajectory(r.ref);
            r.input.start_vel = v0;
            r.input.start_acc = Eigen::Vector2d::Zero();
            r.input.model = ma_spline_opt::OptModel::OMNI_XY;

            std::vector<double> times;
            ma_spline_opt::MAsplineOutput out;
            for (int k = 0; k < kRepeat; ++k) {
                Optimizer opt; // 每次新建，避免内部状态影响计时
                opt.set_config(oc);
                opt.set_esdf_interface(c.esdf.get());
                const int fb_before = sink.stage2_fallback.load();
                const auto t0 = std::chrono::steady_clock::now();
                auto o = opt.optimize(r.input);
                times.push_back(ms_since(t0));
                if (k == 0) {
                    r.m.stage2_fallback = sink.stage2_fallback.load() > fb_before;
                    out = std::move(o);
                }
            }
            r.m.opt_ms = median(times);
            evaluate(out, *c.esdf, oc, r.m);
            runs.push_back(std::move(r));
        }
    }
}

// ---------------- 绘图 ----------------

auto points_of(const PostProc::Trajectory& t) -> std::vector<Eigen::Vector2d> {
    std::vector<Eigen::Vector2d> pts;
    for (const auto& tp: t.timed_trajectory) pts.emplace_back(tp.state.x(), tp.state.y());
    return pts;
}

auto draw_query_panel(const Case& c, int mode, size_t qi, double vmax) -> cv::Mat {
    const auto& m = *c.map;
    const auto& r = c.runs[mode][qi];
    const auto& q = c.queries[qi];
    cv::Mat img = viz::base_image(m, kDrawScale);

    viz::draw_path(img, m, c.raw_paths[qi], kColorRaw, 1, false, kDrawScale);
    const auto ref = points_of(r.ref);
    viz::draw_path(img, m, ref, kColorRefLine, 1, false, kDrawScale);
    if (r.m.success) viz::draw_colored_path(img, m, r.m.p, r.m.v, 0.0, vmax, 3, kDrawScale);
    viz::draw_points(img, m, ref, kColorRefPoint, 4, kDrawScale);
    viz::draw_start_goal(img, m, q.start, q.goal, kDrawScale);
    viz::draw_colorbar(img, {img.cols - 80, 110}, 160, 0.0, vmax, "|v| m/s");

    viz::draw_label(img,
                    c.name + "  " + kModeNames[mode] + "  query #" + std::to_string(qi) +
                        "  (gray=JPS, red pts+blue line=reference, colored=optimized)",
                    {8, 20}, viz::kBlack);
    if (!r.m.success) {
        viz::draw_label(img, "OPT FAILED", {8, 42}, viz::kRed);
    } else {
        viz::draw_label(img,
                        "opt " + strf("%.2f ms", r.m.opt_ms) + "  T " + strf("%.2f", r.m.ref_duration) + " -> " +
                            strf("%.2f s", r.m.duration) + "  seg " + std::to_string(r.m.segments) +
                            (r.m.stage2_fallback ? "  stage2 fallback" : ""),
                        {8, 42}, viz::kBlack);
        viz::draw_label(img,
                        "max|v| " + strf("%.2f", r.m.max_v) + "  max|a| " + strf("%.2f", r.m.max_a) + "  min d " +
                            strf("%.3f m", r.m.min_esdf) + "  d<safe " + strf("%.1f%%", 100.0 * r.m.unsafe_ratio),
                        {8, 64}, (r.m.v_ok && r.m.a_ok && r.m.esdf_ok) ? viz::kBlack : viz::kRed);
    }
    viz::draw_frame(img);
    return img;
}

auto longest_queries(const Case& c, size_t n) -> std::vector<size_t> {
    std::vector<size_t> idx(c.queries.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](size_t l, size_t r) {
        return (c.queries[l].goal - c.queries[l].start).norm() > (c.queries[r].goal - c.queries[r].start).norm();
    });
    idx.resize(std::min(n, idx.size()));
    return idx;
}

auto draw_profile(const Case& c, int mode, size_t qi, const ma_spline_opt::MaSplineOptimizerConfig& oc) -> cv::Mat {
    const auto& m = c.runs[mode][qi].m;
    const std::string head = c.name + " " + kModeNames[mode] + " query #" + std::to_string(qi);
    const cv::Mat va = viz::draw_line_chart(head + "  speed / acceleration",
                                            {{"|v|", viz::kBlue, m.t, m.v}, {"|a|", viz::kRed, m.t, m.a}},
                                            {{"v_max", viz::kBlue, oc.v_max}, {"a_max", viz::kRed, oc.a_max}}, "t (s)",
                                            "m/s, m/s^2", 1100, 340);
    const cv::Mat d = viz::draw_line_chart(head + "  ESDF distance along trajectory", {{"d", viz::kGreen, m.t, m.d}},
                                           {{"safe_distance", viz::kOrange, oc.safe_distance}, {"0", viz::kRed, 0.0}},
                                           "t (s)", "m", 1100, 300);
    return viz::vstack({va, d});
}

// ---------------- 汇总 ----------------

auto mean_ms(const std::vector<Run>& rs) -> double {
    double s = 0.0;
    int n = 0;
    for (const auto& r: rs) {
        if (!r.m.success) continue;
        s += r.m.opt_ms;
        ++n;
    }
    return n > 0 ? s / n : NAN;
}

auto print_summary(const Case& c, int mode) -> int {
    const auto& rs = c.runs[mode];
    int ok = 0, fb = 0, vbad = 0, abad = 0, dbad = 0;
    double max_v = 0, max_a = 0, min_d = std::numeric_limits<double>::max(), unsafe = 0;
    std::vector<double> times;
    for (const auto& r: rs) {
        if (!r.m.success) continue;
        ++ok;
        fb += r.m.stage2_fallback;
        vbad += !r.m.v_ok;
        abad += !r.m.a_ok;
        dbad += !r.m.esdf_ok;
        max_v = std::max(max_v, r.m.max_v);
        max_a = std::max(max_a, r.m.max_a);
        min_d = std::min(min_d, r.m.min_esdf);
        unsafe += r.m.unsafe_ratio;
        times.push_back(r.m.opt_ms);
    }
    std::sort(times.begin(), times.end());
    const double p90 = times.empty() ? NAN : times[std::min(times.size() - 1, times.size() * 9 / 10)];
    std::printf("[%s/%s] queries %zu, success %d, stage2 fallback %d\n", c.name.c_str(), kModeNames[mode].c_str(),
                rs.size(), ok, fb);
    std::printf("  opt time  mean %.3f ms  p90 %.3f ms  max %.3f ms\n", mean_ms(rs), p90,
                times.empty() ? NAN : times.back());
    std::printf("  max|v| %.3f  max|a| %.3f  min ESDF %.3f m  mean d<safe %.1f%%\n", max_v, max_a, min_d,
                ok > 0 ? 100.0 * unsafe / ok : 0.0);
    std::printf("  checks: v over limit %d, a over limit %d, ESDF < 0 %d\n", vbad, abad, dbad);
    return (static_cast<int>(rs.size()) - ok) + vbad + abad + dbad;
}

auto write_csv(const std::vector<std::unique_ptr<Case>>& cases) -> void {
    const std::string path = kOutputDir + "traj_optimize.csv";
    std::ofstream f(path);
    f << "map,mode,query,start_x,start_y,goal_x,goal_y,success,stage2_fallback,opt_ms,cost,segments,"
         "ref_duration_s,duration_s,length_m,max_v,max_a,min_esdf_m,unsafe_ratio,v_ok,a_ok,esdf_ok\n";
    for (const auto& c: cases) {
        for (int mode = 0; mode < kNumModes; ++mode) {
            for (size_t i = 0; i < c->runs[mode].size(); ++i) {
                const auto& q = c->queries[i];
                const auto& m = c->runs[mode][i].m;
                f << c->name << ',' << kModeNames[mode] << ',' << i << ',' << q.start.x() << ',' << q.start.y()
                  << ',' << q.goal.x() << ',' << q.goal.y() << ',' << m.success << ',' << m.stage2_fallback << ','
                  << m.opt_ms << ',' << m.cost << ',' << m.segments << ',' << m.ref_duration << ',' << m.duration
                  << ',' << m.length << ',' << m.max_v << ',' << m.max_a << ',' << m.min_esdf << ','
                  << m.unsafe_ratio << ',' << m.v_ok << ',' << m.a_ok << ',' << m.esdf_ok << '\n';
            }
        }
    }
    std::printf("  saved %s\n", path.c_str());
}

} // namespace

int main(int argc, char** argv) {
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;

    std::filesystem::create_directories(kOutputDir);
    std::mt19937 rng(kSeed);

    PostProc::PathPostProcessingParams pp;
    ma_spline_opt::MaSplineOptimizerConfig oc;
    try {
        const YAML::Node root = YAML::LoadFile(kConfigPath);
        pp = load_post_params(root["planner_config"]["path_planning_params"]);
        oc = load_opt_config(root["planner_config"]["ma_spline_opt_params"]);
    } catch (const std::exception& e) {
        std::printf("failed to load %s: %s\n", kConfigPath.c_str(), e.what());
        return 1;
    }
    std::printf("opt params: safe %.2f m, v_max %.2f, a_max %.2f, K %d | stage1 w_obs %.0f iters %d | stage2 w_obs %.0f "
                "iters %d\n",
                oc.safe_distance, oc.v_max, oc.a_max, oc.integral_num_steps, oc.stage1.weight_obstacle,
                oc.stage1.max_iterations, oc.stage2.weight_obstacle, oc.stage2.max_iterations);

    // 统计 stage2 回退；同时关掉控制台的 warn 刷屏
    auto sink = std::make_shared<CountSink>();
    logger::traj_opt->sinks().clear();
    logger::traj_opt->sinks().push_back(sink);
    logger::traj_opt->set_level(spdlog::level::warn);

    std::vector<std::unique_ptr<Case>> cases;
    for (const auto& name: kMapNames) {
        auto c = std::make_unique<Case>();
        c->name = name;
        try {
            test_utils::Cv2Map::Options opt;
            opt.resolution = kResolution;
            c->map = std::make_unique<test_utils::Cv2Map>(kImageDir + name + ".png", opt);
        } catch (const std::exception& e) {
            std::printf("skip %s: %s\n", name.c_str(), e.what());
            continue;
        }
        c->gm = std::make_shared<const grid_map::GridMap>(c->map->grid_map());
        c->esdf = std::make_unique<minco_opt::GridMapESDF>(c->gm);
        c->jps.set_map(*c->gm);
        c->jps.set_safe_threshold(pp.safe_threshold);

        const auto level = logger::planning->level();
        logger::planning->set_level(spdlog::level::off); // 不连通查询会打 error 日志
        sample_queries(*c, pp.safe_threshold, rng);
        logger::planning->set_level(level);
        if (c->queries.empty()) {
            std::printf("skip %s: no valid queries\n", name.c_str());
            continue;
        }
        cases.push_back(std::move(c));
    }
    if (cases.empty()) {
        std::printf("no map loaded from %s\n", kImageDir.c_str());
        return 1;
    }

    // ---- 优化 + 检查 + 图片 ----
    std::printf("==== MaSplineTrajectoryOptimizer (median of %d runs) ====\n", kRepeat);
    int issues = 0;
    std::vector<std::string> map_labels;
    std::vector<viz::Series> summary;
    for (int mode = 0; mode < kNumModes; ++mode) summary.push_back({kModeNames[mode], kModeColors[mode], {}});
    for (auto& c: cases) {
        run_case(*c, pp, oc, *sink);
        const auto top = longest_queries(*c, 4);
        for (int mode = 0; mode < kNumModes; ++mode) {
            issues += print_summary(*c, mode);
            std::vector<cv::Mat> panels;
            for (size_t i: top) panels.push_back(draw_query_panel(*c, mode, i, oc.v_max));
            viz::save(kOutputDir, c->name + "_" + kModeNames[mode] + "_traj.png", viz::mosaic_2x2(std::move(panels)));
            if (!top.empty() && c->runs[mode][top[0]].m.success) {
                viz::save(kOutputDir, c->name + "_" + kModeNames[mode] + "_profile.png",
                          draw_profile(*c, mode, top[0], oc));
            }
            summary[mode].values.push_back(mean_ms(c->runs[mode]));
        }

        std::vector<std::string> qlabels;
        for (size_t i = 0; i < c->queries.size(); ++i) qlabels.push_back(std::to_string(i));
        std::vector<viz::Series> per_query;
        for (int mode = 0; mode < kNumModes; ++mode) {
            viz::Series s {kModeNames[mode], kModeColors[mode], {}};
            for (const auto& r: c->runs[mode]) s.values.push_back(r.m.success ? r.m.opt_ms : NAN);
            per_query.push_back(std::move(s));
        }
        viz::save(kOutputDir, c->name + "_time.png",
                  viz::draw_grouped_bars(c->name + " trajectory optimization time per query   mean static " +
                                             strf("%.2f", mean_ms(c->runs[0])) + " ms, init_vel " +
                                             strf("%.2f", mean_ms(c->runs[1])) + " ms",
                                         qlabels, per_query, true, "ms", 1400, 520, false));
        map_labels.push_back(c->name);
    }
    viz::save(kOutputDir, "summary.png",
              viz::draw_grouped_bars("Mean trajectory optimization time", map_labels, summary, false, "ms", 700, 450,
                                     true));
    write_csv(cases);
    std::printf("checks: %s (%d issue(s))\n", issues == 0 ? "PASS" : "FAIL", issues);

    // ---- Google Benchmark ----
    logger::traj_opt->set_level(spdlog::level::off);
    for (auto& c: cases) {
        for (int mode = 0; mode < kNumModes; ++mode) {
            Case* cp = c.get();
            const auto nq = static_cast<int64_t>(cp->queries.size());
            benchmark::RegisterBenchmark(("TrajOpt/" + cp->name + "/" + kModeNames[mode]).c_str(),
                                         [cp, mode, nq, oc](benchmark::State& st) {
                                             Optimizer opt;
                                             opt.set_config(oc);
                                             opt.set_esdf_interface(cp->esdf.get());
                                             for (auto _: st) {
                                                 for (const auto& r: cp->runs[mode]) {
                                                     auto o = opt.optimize(r.input);
                                                     benchmark::DoNotOptimize(o.cost);
                                                 }
                                             }
                                             st.SetItemsProcessed(st.iterations() * nq);
                                             st.counters["queries"] = static_cast<double>(nq);
                                         })
                ->Unit(benchmark::kMillisecond);
        }
    }
    std::printf("==== Google Benchmark (one iteration = all queries of a map) ====\n");
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return issues == 0 ? 0 : 2;
}
