/*
 * A* vs JPS 对比测试
 *
 *  1. 读取 images/ 下的黑白地图(黑色 = 障碍) -> GridMap (Cv2Map)。
 *  2. 每张地图用固定随机种子采样若干组可达的起终点。
 *  3. 对比运行:每组起终点各跑 kRepeat 次取中位数耗时,统计路径长度/航点数,
 *     结果写入 images/output/:
 *       - <map>_paths.png      最长 4 组查询的 A*(红) / JPS(蓝) 轨迹叠加原图
 *       - <map>_time.png       每组查询的耗时柱状图(对数坐标)
 *       - summary.png          各地图平均耗时 / 平均路径长度对比
 *       - benchmark.csv        逐查询原始数据
 *  4. Google Benchmark:每张地图 × 每个算法注册一个用例,一次迭代跑完全部查询。
 *     可用 --benchmark_out=images/output/benchmark.json --benchmark_out_format=json 导出。
 */

#include "cv2map.hpp"
#include "cv_show.hpp"
#include "path_planning/search/astar.h"
#include "path_planning/search/jps.h"
#include "utils/logger.hpp"

#include <benchmark/benchmark.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif

namespace {

// ---------------- 测试参数 ----------------
constexpr double kResolution = 0.05;    // 560x300 像素 <-> 28m x 15m 场地
constexpr double kSafeThreshold = 0.05; // 规划安全距离(m)
constexpr int kNumQueries = 20;         // 每张地图的查询数
constexpr double kMinQueryDist = 8.0;   // 起终点最小直线距离(m)
constexpr int kRepeat = 5;              // 对比运行时每组查询重复次数(取中位数)
constexpr int kTimeoutMs = 2000;
constexpr unsigned kSeed = 42;
constexpr double kDrawScale = 2.0; // 轨迹图放大倍数

const std::string kImageDir = std::string(ROOT_DIR) + "images/";
const std::string kOutputDir = kImageDir + "output/";
const std::vector<std::string> kMapNames = {"rmuc2024", "rmuc2025"};

namespace viz = test_utils::viz;
using viz::strf;
const cv::Scalar kColorAStar = viz::kRed;
const cv::Scalar kColorJps = viz::kBlue;

struct Query {
    Eigen::Vector2d start;
    Eigen::Vector2d goal;
};

struct RunResult {
    bool ok = false;
    double time_ms = 0.0; // 中位数耗时
    double length = 0.0;  // 路径长度(m)
    size_t waypoints = 0;
    std::vector<Eigen::Vector2d> path;
};

struct Case {
    std::string name;
    std::unique_ptr<test_utils::Cv2Map> map;
    path_planning::AStar astar;
    path_planning::JPS jps;
    std::vector<Query> queries;
    std::vector<RunResult> astar_res;
    std::vector<RunResult> jps_res;
};

auto path_length(const std::vector<Eigen::Vector2d>& p) -> double {
    double len = 0.0;
    for (size_t i = 1; i < p.size(); ++i) len += (p[i] - p[i - 1]).norm();
    return len;
}

template<typename F>
auto run_timed(F&& plan) -> RunResult {
    RunResult r;
    std::vector<double> times;
    for (int i = 0; i < kRepeat; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        auto path = plan();
        const auto t1 = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        if (i == 0) r.path = std::move(path);
    }
    std::nth_element(times.begin(), times.begin() + kRepeat / 2, times.end());
    r.time_ms = times[kRepeat / 2];
    r.ok = !r.path.empty();
    r.length = path_length(r.path);
    r.waypoints = r.path.size();
    return r;
}

auto mean_of(const std::vector<RunResult>& rs, double RunResult::*field) -> double {
    double sum = 0.0;
    int n = 0;
    for (const auto& r: rs) {
        if (!r.ok) continue;
        sum += r.*field;
        ++n;
    }
    return n > 0 ? sum / n : std::nan("");
}

// 采样起终点:自由栅格(ESDF 留出一格余量)、直线距离足够远、JPS 可达。
auto sample_queries(Case& c, std::mt19937& rng) -> std::vector<Query> {
    const auto& m = *c.map;
    const auto& gm = m.grid_map();

    std::vector<Eigen::Vector2d> free_cells;
    for (int row = 0; row < m.height(); ++row) {
        for (int col = 0; col < m.width(); ++col) {
            const Eigen::Vector2d p = m.pixel_to_world(col, row);
            if (gm.getDistance(p) >= kSafeThreshold + kResolution) free_cells.push_back(p);
        }
    }
    std::vector<Query> qs;
    if (free_cells.empty()) return qs;

    std::uniform_int_distribution<size_t> pick(0, free_cells.size() - 1);
    for (int attempt = 0; attempt < kNumQueries * 200 && static_cast<int>(qs.size()) < kNumQueries; ++attempt) {
        const Query q {free_cells[pick(rng)], free_cells[pick(rng)]};
        if ((q.goal - q.start).norm() < kMinQueryDist) continue;
        if (c.jps.jps_search(q.start, q.goal, kTimeoutMs).empty()) continue; // 不连通
        qs.push_back(q);
    }
    return qs;
}

// ---------------- 绘图 ----------------

auto draw_query_panel(const Case& c, size_t qi) -> cv::Mat {
    const auto& m = *c.map;
    cv::Mat img = viz::base_image(m, kDrawScale);

    const auto& a = c.astar_res[qi];
    const auto& j = c.jps_res[qi];
    viz::draw_path(img, m, a.path, kColorAStar, 4, false, kDrawScale);
    viz::draw_path(img, m, j.path, kColorJps, 2, true, kDrawScale);

    const auto& q = c.queries[qi];
    viz::draw_start_goal(img, m, q.start, q.goal, kDrawScale);

    viz::draw_label(img, c.name + "  query #" + std::to_string(qi) + "  (green=start, magenta=goal)", {8, 20},
                    viz::kBlack);
    viz::draw_label(img,
                    "A*  : " + (a.ok ? strf("%.3f ms", a.time_ms) + "  len " + strf("%.2f m", a.length)
                                     : std::string("FAIL")),
                    {8, 42}, kColorAStar);
    viz::draw_label(img,
                    "JPS : " + (j.ok ? strf("%.3f ms", j.time_ms) + "  len " + strf("%.2f m", j.length)
                                     : std::string("FAIL")),
                    {8, 64}, kColorJps);
    viz::draw_frame(img);
    return img;
}

// 选直线距离最长的 4 组查询拼成 2x2
auto draw_paths_mosaic(const Case& c) -> cv::Mat {
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

auto to_series(const std::vector<RunResult>& rs, double RunResult::*field) -> std::vector<double> {
    std::vector<double> v;
    for (const auto& r: rs) v.push_back(r.ok ? r.*field : NAN);
    return v;
}

// ---------------- 对比运行 ----------------

auto run_case(Case& c) -> void {
    c.astar_res.clear();
    c.jps_res.clear();
    for (const auto& q: c.queries) {
        c.astar_res.push_back(run_timed([&] { return c.astar.original_astar_search(q.start, q.goal, kTimeoutMs); }));
        c.jps_res.push_back(run_timed([&] { return c.jps.jps_search(q.start, q.goal, kTimeoutMs); }));
    }
}

auto write_csv(const std::vector<std::unique_ptr<Case>>& cases) -> void {
    const std::string path = kOutputDir + "benchmark.csv";
    std::ofstream f(path);
    f << "map,query,start_x,start_y,goal_x,goal_y,"
         "astar_ok,astar_ms,astar_len_m,astar_waypoints,jps_ok,jps_ms,jps_len_m,jps_waypoints\n";
    for (const auto& c: cases) {
        for (size_t i = 0; i < c->queries.size(); ++i) {
            const auto& q = c->queries[i];
            const auto& a = c->astar_res[i];
            const auto& j = c->jps_res[i];
            f << c->name << ',' << i << ',' << q.start.x() << ',' << q.start.y() << ',' << q.goal.x() << ','
              << q.goal.y() << ',' << a.ok << ',' << a.time_ms << ',' << a.length << ',' << a.waypoints << ','
              << j.ok << ',' << j.time_ms << ',' << j.length << ',' << j.waypoints << '\n';
        }
    }
    std::printf("  saved %s\n", path.c_str());
}

auto print_summary(const Case& c) -> void {
    int a_ok = 0, j_ok = 0;
    for (const auto& r: c.astar_res) a_ok += r.ok;
    for (const auto& r: c.jps_res) j_ok += r.ok;
    const double at = mean_of(c.astar_res, &RunResult::time_ms);
    const double jt = mean_of(c.jps_res, &RunResult::time_ms);
    std::printf(
        "[%s] %dx%d px, occupied %.1f%%, queries %zu\n"
        "  A* : success %d, mean %.3f ms, mean len %.2f m\n"
        "  JPS: success %d, mean %.3f ms, mean len %.2f m\n"
        "  speedup (A*/JPS): %.1fx\n",
        c.name.c_str(), c.map->width(), c.map->height(), c.map->occupied_ratio() * 100.0, c.queries.size(), a_ok, at,
        mean_of(c.astar_res, &RunResult::length), j_ok, jt, mean_of(c.jps_res, &RunResult::length), at / jt);
}

} // namespace

int main(int argc, char** argv) {
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;

    std::filesystem::create_directories(kOutputDir);
    std::mt19937 rng(kSeed);

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
        c->astar.set_map(c->map->grid_map());
        c->astar.set_safe_threshold(kSafeThreshold);
        c->jps.set_map(c->map->grid_map());
        c->jps.set_safe_threshold(kSafeThreshold);

        // 采样时不连通的查询会让 JPS 打 error 日志,先静音
        const auto level = logger::planning->level();
        logger::planning->set_level(spdlog::level::off);
        c->queries = sample_queries(*c, rng);
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

    // ---- 对比运行 + 输出图片 ----
    std::printf("==== A* vs JPS comparison (median of %d runs, safe_threshold %.2f m) ====\n", kRepeat, kSafeThreshold);
    std::vector<std::string> map_labels;
    std::vector<double> sum_at, sum_jt, sum_al, sum_jl;
    for (auto& c: cases) {
        run_case(*c);
        print_summary(*c);

        viz::save(kOutputDir, c->name + "_paths.png", draw_paths_mosaic(*c));

        std::vector<std::string> qlabels;
        for (size_t i = 0; i < c->queries.size(); ++i) qlabels.push_back(std::to_string(i));
        const double at = mean_of(c->astar_res, &RunResult::time_ms);
        const double jt = mean_of(c->jps_res, &RunResult::time_ms);
        viz::save(kOutputDir, c->name + "_time.png",
             viz::draw_grouped_bars(c->name + " planning time per query   mean A* " + strf("%.3f", at) + " ms, JPS "
                                   + strf("%.3f", jt) + " ms, speedup " + strf("%.1fx", at / jt),
                               qlabels,
                               {{"A*", kColorAStar, to_series(c->astar_res, &RunResult::time_ms)},
                                {"JPS", kColorJps, to_series(c->jps_res, &RunResult::time_ms)}},
                               true, "ms", 1200, 520, false));

        map_labels.push_back(c->name);
        sum_at.push_back(at);
        sum_jt.push_back(jt);
        sum_al.push_back(mean_of(c->astar_res, &RunResult::length));
        sum_jl.push_back(mean_of(c->jps_res, &RunResult::length));
    }

    const cv::Mat time_chart =
        viz::draw_grouped_bars("Mean planning time", map_labels,
                               {{"A*", kColorAStar, sum_at}, {"JPS", kColorJps, sum_jt}}, true, "ms", 600, 450, true);
    const cv::Mat len_chart =
        viz::draw_grouped_bars("Mean path length", map_labels,
                               {{"A*", kColorAStar, sum_al}, {"JPS", kColorJps, sum_jl}}, false, "m", 600, 450, true);
    cv::Mat summary;
    cv::hconcat(time_chart, len_chart, summary);
    viz::save(kOutputDir, "summary.png", summary);
    write_csv(cases);

    // ---- Google Benchmark ----
    logger::planning->set_level(spdlog::level::off); // 计时期间不打日志
    for (auto& c: cases) {
        Case* cp = c.get();
        const auto nq = static_cast<int64_t>(cp->queries.size());
        benchmark::RegisterBenchmark(("AStar/" + cp->name).c_str(), [cp, nq](benchmark::State& st) {
            for (auto _: st) {
                for (const auto& q: cp->queries) {
                    auto p = cp->astar.original_astar_search(q.start, q.goal, kTimeoutMs);
                    benchmark::DoNotOptimize(p.data());
                }
            }
            st.SetItemsProcessed(st.iterations() * nq);
            st.counters["queries"] = static_cast<double>(nq);
        })->Unit(benchmark::kMillisecond);
        benchmark::RegisterBenchmark(("JPS/" + cp->name).c_str(), [cp, nq](benchmark::State& st) {
            for (auto _: st) {
                for (const auto& q: cp->queries) {
                    auto p = cp->jps.jps_search(q.start, q.goal, kTimeoutMs);
                    benchmark::DoNotOptimize(p.data());
                }
            }
            st.SetItemsProcessed(st.iterations() * nq);
            st.counters["queries"] = static_cast<double>(nq);
        })->Unit(benchmark::kMillisecond);
    }
    std::printf("==== Google Benchmark (one iteration = all queries of a map) ====\n");
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
