/*
 * 路径后处理（PathPostProcessing）测试
 *
 *  1. 读取 images/ 下的黑白地图(黑色 = 障碍) -> GridMap (Cv2Map)。
 *  2. 后处理参数取 config/planner.yaml 的 planner_config.path_planning_params（与线上一致）。
 *  3. 每张地图用固定随机种子采样若干组 JPS 可达的起终点，JPS 搜索得到 raw_path，再按线上顺序跑后处理四个阶段：
 *       optimize_path(视线剪枝) -> sample_path_states -> assign_trajectory_timing -> fill_additional_trajectory_info
 *     每个阶段重复 kRepeat 次取中位数耗时。
 *  4. 结果写入 images/output/post_processing/：
 *       - <map>_post.png      最长 4 组查询：后处理输出的点（红）与连线（蓝），灰色细线为 JPS 原始路径
 *       - <map>_time.png      每组查询各阶段耗时柱状图（对数坐标）
 *       - summary.png         各地图各阶段平均耗时
 *       - post_processing.csv 逐查询原始数据
 *     并打印检查：剪枝后连线无碰撞、段数在 [min_traj_num, max_traj_num]、总时长 > 0 且时间递增。
 *  5. Google Benchmark：每张地图注册一个用例，一次迭代对全部查询跑完整后处理（不含搜索）。
 */

#include "cv2map.hpp"
#include "cv_show.hpp"
#include "path_planning/post_processing.h"
#include "path_planning/search/jps.h"
#include "utils/logger.hpp"

#include <benchmark/benchmark.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
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

// ---------------- 测试参数 ----------------
constexpr double kResolution = 0.05;  // 560x300 像素 <-> 28m x 15m 场地
constexpr int kNumQueries = 20;       // 每张地图的查询数
constexpr double kMinQueryDist = 8.0; // 起终点最小直线距离(m)
constexpr int kRepeat = 21;           // 每阶段重复次数(取中位数)
constexpr int kTimeoutMs = 2000;
constexpr unsigned kSeed = 42;
constexpr double kDrawScale = 2.0;

const std::string kImageDir = std::string(ROOT_DIR) + "images/";
const std::string kOutputDir = kImageDir + "output/post_processing/";
const std::string kConfigPath = std::string(ROOT_DIR) + "config/planner.yaml";
const std::vector<std::string> kMapNames = {"rmuc2024", "rmuc2025"};

const cv::Scalar kColorPoint = viz::kRed; // 后处理输出的点
const cv::Scalar kColorLine = viz::kBlue; // 后处理输出点之间的连线
const cv::Scalar kColorRaw = viz::kGray;  // JPS 原始路径

constexpr int kNumStages = 4;
const std::array<std::string, kNumStages> kStageNames = {"optimize_path", "sample_states", "timing", "fill_info"};
const std::array<cv::Scalar, kNumStages> kStageColors = {viz::kRed, viz::kBlue, viz::kGreen, viz::kOrange};

struct Query {
    Eigen::Vector2d start;
    Eigen::Vector2d goal;
};

struct Result {
    bool ok = false;
    std::array<double, kNumStages> stage_ms {};
    double total_ms = 0.0;
    PostProc::Trajectory traj;
    bool line_free = true; // 剪枝后相邻点连线无碰撞
    bool count_ok = true;  // timed_trajectory 段数在范围内
    bool time_ok = true;   // 总时长 > 0 且时间严格递增
};

struct Case {
    std::string name;
    std::unique_ptr<test_utils::Cv2Map> map;
    path_planning::JPS jps;
    PostProc post;
    std::vector<Query> queries;
    std::vector<std::vector<Eigen::Vector2d>> raw_paths;
    std::vector<Result> results;
};

// 读取与 ros2/config.hpp 的 load_path_post_processing_params 相同的键（测试不链接 ros2 库）
auto load_params(const std::string& path) -> PostProc::PathPostProcessingParams {
    PostProc::PathPostProcessingParams p {};
    p.safe_threshold = 0.35;
    p.max_vel = 3.0;
    p.max_acc = 3.0;
    p.time_resolution = 0.4;
    p.min_traj_num = 5;
    p.traj_cut_length = 10.0;
    p.distance_weight = 1.0;
    p.yaw_weight = 0.5;

    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node n = root["planner_config"]["path_planning_params"];
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
    p.start_vel = 0.0; // 静止起步、终点停车
    p.end_vel = 0.0;
    return p;
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
    c.queries.clear();
    c.raw_paths.clear();
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

auto median(std::vector<double> v) -> double {
    if (v.empty()) return NAN;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

auto ms_since(std::chrono::steady_clock::time_point t0) -> double {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// 跑一次完整后处理（与 path_planning.cpp 的调用顺序一致）
auto run_pipeline(PostProc& post, const Query& q, const std::vector<Eigen::Vector2d>& raw,
                  std::array<double, kNumStages>* stage_ms) -> PostProc::Trajectory {
    PostProc::Trajectory traj;
    traj.raw_path = raw;

    auto t0 = std::chrono::steady_clock::now();
    traj.optimized_path = post.optimize_path(traj.raw_path);
    if (stage_ms) (*stage_ms)[0] = ms_since(t0);

    t0 = std::chrono::steady_clock::now();
    traj.path_states = post.sample_path_states(traj.optimized_path);
    if (stage_ms) (*stage_ms)[1] = ms_since(t0);

    t0 = std::chrono::steady_clock::now();
    post.assign_trajectory_timing(traj);
    if (stage_ms) (*stage_ms)[2] = ms_since(t0);

    t0 = std::chrono::steady_clock::now();
    post.fill_additional_trajectory_info(traj, q.start, q.goal, 0.0, 0.0);
    if (stage_ms) (*stage_ms)[3] = ms_since(t0);
    return traj;
}

auto timed_points(const PostProc::Trajectory& traj) -> std::vector<Eigen::Vector2d> {
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(traj.timed_trajectory.size());
    for (const auto& tp: traj.timed_trajectory) pts.emplace_back(tp.state.x(), tp.state.y());
    return pts;
}

auto run_case(Case& c, const PostProc::PathPostProcessingParams& p) -> void {
    c.results.clear();
    for (size_t qi = 0; qi < c.queries.size(); ++qi) {
        Result r;
        std::array<std::vector<double>, kNumStages> samples;
        std::vector<double> totals;
        for (int k = 0; k < kRepeat; ++k) {
            std::array<double, kNumStages> st {};
            auto traj = run_pipeline(c.post, c.queries[qi], c.raw_paths[qi], &st);
            double sum = 0.0;
            for (int s = 0; s < kNumStages; ++s) {
                samples[s].push_back(st[s]);
                sum += st[s];
            }
            totals.push_back(sum);
            if (k == 0) r.traj = std::move(traj);
        }
        for (int s = 0; s < kNumStages; ++s) r.stage_ms[s] = median(samples[s]);
        r.total_ms = median(totals);

        const auto& t = r.traj;
        r.ok = t.timed_trajectory.size() >= 2 && t.total_time > 1e-6;
        for (size_t i = 1; i < t.optimized_path.size(); ++i) {
            if (c.post.check_line_collision(t.optimized_path[i - 1], t.optimized_path[i])) r.line_free = false;
        }
        const int n_seg = static_cast<int>(t.timed_trajectory.size()) - 1;
        r.count_ok = n_seg >= p.min_traj_num && n_seg <= p.max_traj_num;
        r.time_ok = t.total_time > 0.0;
        for (size_t i = 1; i < t.timed_trajectory.size(); ++i) {
            if (!(t.timed_trajectory[i].time > t.timed_trajectory[i - 1].time)) r.time_ok = false;
        }
        c.results.push_back(std::move(r));
    }
}

// ---------------- 绘图 ----------------

auto draw_query_panel(const Case& c, size_t qi) -> cv::Mat {
    const auto& m = *c.map;
    cv::Mat img = viz::base_image(m, kDrawScale);
    const auto& r = c.results[qi];
    const auto& q = c.queries[qi];
    const auto pts = timed_points(r.traj);

    viz::draw_path(img, m, c.raw_paths[qi], kColorRaw, 1, false, kDrawScale);
    viz::draw_path(img, m, pts, kColorLine, 2, false, kDrawScale);
    viz::draw_points(img, m, pts, kColorPoint, 4, kDrawScale);
    viz::draw_start_goal(img, m, q.start, q.goal, kDrawScale);

    viz::draw_label(img, c.name + "  query #" + std::to_string(qi) + "  (green=start, magenta=goal, gray=JPS raw)",
                    {8, 20}, viz::kBlack);
    viz::draw_label(img,
                    "points(red) " + std::to_string(pts.size()) + "  lines(blue)   raw " +
                        std::to_string(c.raw_paths[qi].size()) + " -> pruned " +
                        std::to_string(r.traj.optimized_path.size()),
                    {8, 42}, kColorLine);
    viz::draw_label(img,
                    "len " + strf("%.2f m", r.traj.total_length) + "  T " + strf("%.2f s", r.traj.total_time) +
                        "  post " + strf("%.3f ms", r.total_ms),
                    {8, 64}, kColorPoint);
    viz::draw_frame(img);
    return img;
}

// 选直线距离最长的 4 组查询拼成 2x2
auto draw_mosaic(const Case& c) -> cv::Mat {
    std::vector<size_t> idx(c.queries.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](size_t l, size_t r) {
        return (c.queries[l].goal - c.queries[l].start).norm() > (c.queries[r].goal - c.queries[r].start).norm();
    });
    idx.resize(std::min<size_t>(4, idx.size()));
    std::vector<cv::Mat> panels;
    for (size_t i: idx) panels.push_back(draw_query_panel(c, i));
    return viz::mosaic_2x2(std::move(panels));
}

auto stage_series(const std::vector<Result>& rs) -> std::vector<viz::Series> {
    std::vector<viz::Series> out;
    for (int s = 0; s < kNumStages; ++s) {
        viz::Series ser {kStageNames[s], kStageColors[s], {}};
        for (const auto& r: rs) ser.values.push_back(r.ok ? r.stage_ms[s] : NAN);
        out.push_back(std::move(ser));
    }
    return out;
}

// s < 0 表示总耗时
auto mean_stage(const std::vector<Result>& rs, int s) -> double {
    double sum = 0.0;
    int n = 0;
    for (const auto& r: rs) {
        if (!r.ok) continue;
        sum += s < 0 ? r.total_ms : r.stage_ms[s];
        ++n;
    }
    return n > 0 ? sum / n : NAN;
}

auto write_csv(const std::vector<std::unique_ptr<Case>>& cases) -> void {
    const std::string path = kOutputDir + "post_processing.csv";
    std::ofstream f(path);
    f << "map,query,start_x,start_y,goal_x,goal_y,ok,raw_pts,pruned_pts,timed_pts,total_length_m,"
         "weighted_length_m,total_time_s,line_free,count_ok,time_ok,"
         "optimize_path_ms,sample_states_ms,timing_ms,fill_info_ms,total_ms\n";
    for (const auto& c: cases) {
        for (size_t i = 0; i < c->queries.size(); ++i) {
            const auto& q = c->queries[i];
            const auto& r = c->results[i];
            const auto& t = r.traj;
            f << c->name << ',' << i << ',' << q.start.x() << ',' << q.start.y() << ',' << q.goal.x() << ','
              << q.goal.y() << ',' << r.ok << ',' << c->raw_paths[i].size() << ',' << t.optimized_path.size() << ','
              << t.timed_trajectory.size() << ',' << t.total_length << ',' << t.weighted_length << ','
              << t.total_time << ',' << r.line_free << ',' << r.count_ok << ',' << r.time_ok;
            for (int s = 0; s < kNumStages; ++s) f << ',' << r.stage_ms[s];
            f << ',' << r.total_ms << '\n';
        }
    }
    std::printf("  saved %s\n", path.c_str());
}

// 返回问题数
auto print_summary(const Case& c) -> int {
    int ok = 0, line_bad = 0, count_bad = 0, time_bad = 0;
    for (const auto& r: c.results) {
        ok += r.ok;
        line_bad += !r.line_free;
        count_bad += !r.count_ok;
        time_bad += !r.time_ok;
    }
    std::printf("[%s] queries %zu, ok %d\n", c.name.c_str(), c.queries.size(), ok);
    for (int s = 0; s < kNumStages; ++s) {
        std::printf("  %-14s mean %.4f ms\n", kStageNames[s].c_str(), mean_stage(c.results, s));
    }
    std::printf("  %-14s mean %.4f ms\n", "total", mean_stage(c.results, -1));
    std::printf("  checks: line collision %d, segment count out of range %d, bad timing %d\n", line_bad, count_bad,
                time_bad);
    return (static_cast<int>(c.queries.size()) - ok) + line_bad + count_bad + time_bad;
}

} // namespace

int main(int argc, char** argv) {
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;

    std::filesystem::create_directories(kOutputDir);
    std::mt19937 rng(kSeed);

    PostProc::PathPostProcessingParams params;
    try {
        params = load_params(kConfigPath);
    } catch (const std::exception& e) {
        std::printf("failed to load %s: %s\n", kConfigPath.c_str(), e.what());
        return 1;
    }
    std::printf("params (%s): safe %.2f m, vmax %.2f, amax %.2f, time_res %.2f s, traj_num [%d,%d], dense %.3f m, "
                "rot_w %.2f\n",
                kConfigPath.c_str(), params.safe_threshold, params.max_vel, params.max_acc, params.time_resolution,
                params.min_traj_num, params.max_traj_num, params.dense_sample_resolution,
                params.rotation_penalty_weight);

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
        c->jps.set_map(c->map->grid_map());
        c->jps.set_safe_threshold(params.safe_threshold);
        c->post.set_map(c->map->grid_map());
        c->post.set_params(params);

        const auto level = logger::planning->level();
        logger::planning->set_level(spdlog::level::off); // 不连通查询会打 error 日志
        sample_queries(*c, params.safe_threshold, rng);
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

    // ---- 分阶段计时 + 输出图片 ----
    std::printf("==== PathPostProcessing (median of %d runs per stage) ====\n", kRepeat);
    int issues = 0;
    std::vector<std::string> map_labels;
    std::vector<viz::Series> summary_series;
    for (int s = 0; s < kNumStages; ++s) summary_series.push_back({kStageNames[s], kStageColors[s], {}});
    for (auto& c: cases) {
        run_case(*c, params);
        issues += print_summary(*c);

        viz::save(kOutputDir, c->name + "_post.png", draw_mosaic(*c));

        std::vector<std::string> qlabels;
        for (size_t i = 0; i < c->queries.size(); ++i) qlabels.push_back(std::to_string(i));
        viz::save(kOutputDir, c->name + "_time.png",
                  viz::draw_grouped_bars(c->name + " post-processing time per query   mean total " +
                                             strf("%.3f", mean_stage(c->results, -1)) + " ms",
                                         qlabels, stage_series(c->results), true, "ms", 1400, 520, false));
        map_labels.push_back(c->name);
        for (int s = 0; s < kNumStages; ++s) summary_series[s].values.push_back(mean_stage(c->results, s));
    }
    viz::save(kOutputDir, "summary.png",
              viz::draw_grouped_bars("Mean post-processing time per stage", map_labels, summary_series, true, "ms",
                                     900, 480, true));
    write_csv(cases);
    std::printf("checks: %s (%d issue(s))\n", issues == 0 ? "PASS" : "FAIL", issues);

    // ---- Google Benchmark ----
    logger::planning->set_level(spdlog::level::off); // 计时期间不打日志
    for (auto& c: cases) {
        Case* cp = c.get();
        const auto nq = static_cast<int64_t>(cp->queries.size());
        benchmark::RegisterBenchmark(("PostProcessing/" + cp->name).c_str(), [cp, nq](benchmark::State& st) {
            for (auto _: st) {
                for (size_t i = 0; i < cp->queries.size(); ++i) {
                    auto t = run_pipeline(cp->post, cp->queries[i], cp->raw_paths[i], nullptr);
                    benchmark::DoNotOptimize(t.total_time);
                }
            }
            st.SetItemsProcessed(st.iterations() * nq);
            st.counters["queries"] = static_cast<double>(nq);
        })->Unit(benchmark::kMicrosecond);
    }
    std::printf("==== Google Benchmark (one iteration = all queries of a map) ====\n");
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return issues == 0 ? 0 : 2;
}
