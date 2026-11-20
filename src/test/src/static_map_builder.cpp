// 离线静态地图与 2.5D 地形先验构建工具（无 ROS 依赖）。
// 支持三种输入：
//   1. .stl        : 3D 场地三角网格（实体垂向射线精确求交 + 三角面采样）
//   2. .pcd        : 静态 3D 点云地图（ASCII / binary / binary_compressed）
//   3. frames.bin  : terrain_replay_extract.py 导出的带位姿世界系点云帧序列（复用 OnlineTerrainMapper 3D 射线扫空）
// 输出：
//   <output>.pgm + <output>.yaml + <output>.terrain.bin（严格对齐输入世界坐标系）
//   可选 --viz <prefix> 输出诊断 PNG（占据图 / 地面高程图 / 悬空净空叠加图）

#include "map/3d_occ_map/config.hpp"
#include "map/core/map_io.hpp"
#include "map/perception/online_terrain_mapper.hpp"
#include "map/perception/sensor_model.hpp"
#include "map/perception/static_terrain_builder.hpp"
#include "stl_loader.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

auto msSince(Clock::time_point t0) -> double {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

struct CliOptions {
    std::string input_path;
    std::string output_prefix;
    std::string mode = "auto"; // auto | stl | pcd | frames
    std::string config_path;
    std::string viz_prefix;
    int viz_scale = 2;

    double resolution = 0.05;
    double sample_step = -1.0; // <= 0 时默认 resolution / 4
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    bool ground_to_zero = false;

    ma_map::StaticTerrainBuilder::Config terrain_cfg {};
    int pcd_min_pts = 1;
    double z_filter_min = -2.0;
    double z_filter_max = 4.0;

    bool has_t_bl = false;
    Eigen::Vector3d t_base_lidar {0.0, 0.0, 0.194};
    bool has_rpy = false;
    Eigen::Vector3d rpy_deg {0.0, -20.0, 0.0};

    int margin_cells = 2;
    bool crop_to_explored = true;
    bool keep_largest_cc = false;
};

auto printUsage(const char* prog) -> void {
    std::cerr
        << "Usage: " << prog << " --input <file.stl|file.pcd|frames.bin> --output <prefix> [options]\n"
        << "Options:\n"
        << "  --mode <auto|stl|pcd|frames>     Input format (default: auto)\n"
        << "  --resolution <m>                 Grid resolution (default: 0.05)\n"
        << "  --offset <x> <y> <z>             World translation applied to input (default: 0 0 0)\n"
        << "  --ground-to-zero                 Shift input Z so min_z becomes 0 (default: off)\n"
        << "  --robot-height <m>               Robot height clearance (default: 0.24)\n"
        << "  --min-overhang-clearance <m>     Minimum overhang clearance (default: 0.225)\n"
        << "  --max-ground-z <m>               Max passable ground Z (default: 1.5)\n"
        << "  --min-ground-z <m>               Min passable ground Z (default: -1.0)\n"
        << "  --max-slope-deg <deg>            Max passable slope in degrees (default: 25.0)\n"
        << "  --step-tol <m>                   Adjacent step tolerance (default: 0.03)\n"
        << "  --max-surface-thickness <m>      Point-cloud vertical wall span threshold (default: 0.11)\n"
        << "  --pcd-min-pts <n>                Min points per 3D voxel in PCD mode (default: 1)\n"
        << "  --sample-step <m>                STL surface sampling step (default: resolution / 4)\n"
        << "  --config <map.yaml>              Optional map.yaml for frames.bin mode\n"
        << "  --t-base-lidar <x> <y> <z>       LiDAR translation in base_odom frame (m) for frames.bin\n"
        << "  --rpy-deg <r> <p> <y>            LiDAR RPY (deg, REP-103) for frames.bin\n"
        << "  --margin-cells <n>               Observed-region crop margin (default: 2)\n"
        << "  --no-crop                        Disable cropping output grid to observed bounds\n"
        << "  --keep-largest-cc                Keep only the largest connected FREE component\n"
        << "  --viz <prefix>                   Write diagnostic PNGs (<prefix>_{occ,ground,overhang}.png)\n"
        << "  --viz-scale <n>                  Diagnostic PNG integer scale factor (default: 2)\n";
}

auto parseArgs(int argc, char** argv, CliOptions& opt) -> bool {
    auto needVal = [&](int& i, int count, const char* flag) -> bool {
        if (i + count >= argc) {
            std::cerr << "Missing value(s) for " << flag << "\n";
            return false;
        }
        return true;
    };

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--input" || arg == "-i") {
            if (!needVal(i, 1, "--input")) return false;
            opt.input_path = argv[++i];
        } else if (arg == "--output" || arg == "-o") {
            if (!needVal(i, 1, "--output")) return false;
            opt.output_prefix = argv[++i];
        } else if (arg == "--mode") {
            if (!needVal(i, 1, "--mode")) return false;
            opt.mode = argv[++i];
        } else if (arg == "--resolution") {
            if (!needVal(i, 1, "--resolution")) return false;
            opt.resolution = std::stod(argv[++i]);
        } else if (arg == "--sample-step") {
            if (!needVal(i, 1, "--sample-step")) return false;
            opt.sample_step = std::stod(argv[++i]);
        } else if (arg == "--offset") {
            if (!needVal(i, 3, "--offset")) return false;
            opt.offset.x() = std::stod(argv[++i]);
            opt.offset.y() = std::stod(argv[++i]);
            opt.offset.z() = std::stod(argv[++i]);
        } else if (arg == "--ground-to-zero") {
            opt.ground_to_zero = true;
        } else if (arg == "--robot-height") {
            if (!needVal(i, 1, "--robot-height")) return false;
            opt.terrain_cfg.robot_height = std::stof(argv[++i]);
        } else if (arg == "--min-overhang-clearance") {
            if (!needVal(i, 1, "--min-overhang-clearance")) return false;
            opt.terrain_cfg.min_overhang_clearance = std::stof(argv[++i]);
        } else if (arg == "--max-ground-z") {
            if (!needVal(i, 1, "--max-ground-z")) return false;
            opt.terrain_cfg.max_ground_height = std::stof(argv[++i]);
        } else if (arg == "--min-ground-z") {
            if (!needVal(i, 1, "--min-ground-z")) return false;
            opt.terrain_cfg.min_ground_height = std::stof(argv[++i]);
        } else if (arg == "--max-slope-deg") {
            if (!needVal(i, 1, "--max-slope-deg")) return false;
            opt.terrain_cfg.max_slope_deg = std::stof(argv[++i]);
        } else if (arg == "--step-tol") {
            if (!needVal(i, 1, "--step-tol")) return false;
            opt.terrain_cfg.step_tol = std::stof(argv[++i]);
        } else if (arg == "--max-surface-thickness") {
            if (!needVal(i, 1, "--max-surface-thickness")) return false;
            opt.terrain_cfg.max_surface_thickness = std::stof(argv[++i]);
        } else if (arg == "--pcd-min-pts") {
            if (!needVal(i, 1, "--pcd-min-pts")) return false;
            opt.pcd_min_pts = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--config") {
            if (!needVal(i, 1, "--config")) return false;
            opt.config_path = argv[++i];
        } else if (arg == "--t-base-lidar") {
            if (!needVal(i, 3, "--t-base-lidar")) return false;
            opt.t_base_lidar.x() = std::stod(argv[++i]);
            opt.t_base_lidar.y() = std::stod(argv[++i]);
            opt.t_base_lidar.z() = std::stod(argv[++i]);
            opt.has_t_bl = true;
        } else if (arg == "--rpy-deg") {
            if (!needVal(i, 3, "--rpy-deg")) return false;
            opt.rpy_deg.x() = std::stod(argv[++i]);
            opt.rpy_deg.y() = std::stod(argv[++i]);
            opt.rpy_deg.z() = std::stod(argv[++i]);
            opt.has_rpy = true;
        } else if (arg == "--margin-cells") {
            if (!needVal(i, 1, "--margin-cells")) return false;
            opt.margin_cells = std::max(0, std::stoi(argv[++i]));
        } else if (arg == "--no-crop") {
            opt.crop_to_explored = false;
        } else if (arg == "--keep-largest-cc") {
            opt.keep_largest_cc = true;
        } else if (arg == "--viz") {
            if (!needVal(i, 1, "--viz")) return false;
            opt.viz_prefix = argv[++i];
        } else if (arg == "--viz-scale") {
            if (!needVal(i, 1, "--viz-scale")) return false;
            opt.viz_scale = std::max(1, std::stoi(argv[++i]));
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }

    if (opt.input_path.empty() || opt.output_prefix.empty()) {
        std::cerr << "Both --input and --output are required.\n";
        return false;
    }
    if (!std::isfinite(opt.resolution) || opt.resolution <= 0.0) {
        std::cerr << "Invalid --resolution\n";
        return false;
    }
    return true;
}

auto detectMode(const CliOptions& opt) -> std::string {
    if (opt.mode != "auto") {
        return opt.mode;
    }
    std::string ext = std::filesystem::path(opt.input_path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".stl") return "stl";
    if (ext == ".pcd") return "pcd";
    if (ext == ".bin") return "frames";
    return "";
}

auto compactColumnSpans(std::vector<ma_map::StaticTerrainBuilder::ZSpan>& spans, float merge_gap) -> void {
    if (spans.size() <= 1) {
        return;
    }
    std::sort(
        spans.begin(),
        spans.end(),
        [](const auto& a, const auto& b) {
            return (a.z_lo < b.z_lo) || (a.z_lo == b.z_lo && a.z_hi < b.z_hi);
        }
    );
    size_t write = 0;
    for (size_t i = 1; i < spans.size(); ++i) {
        if (spans[i].z_lo <= spans[write].z_hi + merge_gap) {
            spans[write].z_hi = std::max(spans[write].z_hi, spans[i].z_hi);
        } else {
            spans[++write] = spans[i];
        }
    }
    spans.resize(write + 1);
}

auto buildFromStl(const CliOptions& opt) -> ma_map::StaticTerrainBuilder::BuildResult {
    const auto t0 = Clock::now();
    test_utils::StlMesh mesh = test_utils::load_stl(opt.input_path);
    if (mesh.triangle_num() == 0) {
        throw std::runtime_error("STL mesh contains 0 triangles: " + opt.input_path);
    }

    Eigen::Vector3d shift = opt.offset;
    if (opt.ground_to_zero) {
        shift.z() -= (mesh.min.z() + opt.offset.z());
    }
    if (shift.squaredNorm() > 0.0) {
        mesh.translate(shift);
    }

    const double res = opt.resolution;
    const double inv_res = 1.0 / res;
    const double sample_step = (opt.sample_step > 0.0) ? opt.sample_step : (0.25 * res);

    const int32_t min_vx = static_cast<int32_t>(std::floor(mesh.min.x() * inv_res));
    const int32_t max_vx = static_cast<int32_t>(std::floor(mesh.max.x() * inv_res));
    const int32_t min_vy = static_cast<int32_t>(std::floor(mesh.min.y() * inv_res));
    const int32_t max_vy = static_cast<int32_t>(std::floor(mesh.max.y() * inv_res));
    const int nx = static_cast<int>(max_vx - min_vx + 1);
    const int ny = static_cast<int>(max_vy - min_vy + 1);

    const Eigen::Vector2d origin(static_cast<double>(min_vx) * res, static_cast<double>(min_vy) * res);
    const grid_map::GridGeometry geom =
        grid_map::GridGeometry::fromOriginAndSize(origin, Eigen::Vector2i(nx, ny), res);
    auto col_grid = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(geom, /*solid_mesh=*/true, /*ray_clearing=*/false);

    std::printf(
        "[stl] %s: %zu triangles, bbox min [%.3f %.3f %.3f] max [%.3f %.3f %.3f], grid %dx%d @ %.3fm\n",
        opt.input_path.c_str(),
        mesh.triangle_num(),
        mesh.min.x(),
        mesh.min.y(),
        mesh.min.z(),
        mesh.max.x(),
        mesh.max.y(),
        mesh.max.z(),
        nx,
        ny,
        res
    );

    // 1. 中心竖直射线的精确闭环绕数求交（提取连续实体区间 [z_enter, z_exit]）
    struct Crossing {
        double z;
        int delta;
    };
    std::vector<std::vector<Crossing>> crossings(static_cast<size_t>(nx) * static_cast<size_t>(ny));
    for (size_t t = 0; t < mesh.triangle_num(); ++t) {
        const Eigen::Vector3d& a = mesh.v[3 * t];
        Eigen::Vector3d b = mesh.v[3 * t + 1];
        Eigen::Vector3d c = mesh.v[3 * t + 2];
        const Eigen::Vector3d normal = (b - a).cross(c - a);
        const double n_norm = normal.norm();
        if (std::abs(normal.z()) <= 1e-12 * n_norm) {
            continue;
        }
        const int delta = (normal.z() < 0.0) ? 1 : -1;
        if (normal.z() < 0.0) {
            std::swap(b, c);
        }
        const double x_lo_w = std::min({a.x(), b.x(), c.x()});
        const double x_hi_w = std::max({a.x(), b.x(), c.x()});
        const double y_lo_w = std::min({a.y(), b.y(), c.y()});
        const double y_hi_w = std::max({a.y(), b.y(), c.y()});
        const int ix_lo = std::max(0, static_cast<int>(std::floor(x_lo_w * inv_res)) - min_vx);
        const int ix_hi = std::min(nx - 1, static_cast<int>(std::floor(x_hi_w * inv_res)) - min_vx);
        const int iy_lo = std::max(0, static_cast<int>(std::floor(y_lo_w * inv_res)) - min_vy);
        const int iy_hi = std::min(ny - 1, static_cast<int>(std::floor(y_hi_w * inv_res)) - min_vy);

        for (int ix = ix_lo; ix <= ix_hi; ++ix) {
            const double px = (static_cast<double>(min_vx + ix) + 0.5) * res;
            const size_t row_off = static_cast<size_t>(ix) * static_cast<size_t>(ny);
            for (int iy = iy_lo; iy <= iy_hi; ++iy) {
                const double py = (static_cast<double>(min_vy + iy) + 0.5) * res;
                auto ownsEdge = [px, py](const Eigen::Vector3d& u, const Eigen::Vector3d& v) {
                    const double dx = v.x() - u.x();
                    const double dy = v.y() - u.y();
                    const double edge = dx * (py - u.y()) - dy * (px - u.x());
                    return edge > 0.0 || (edge == 0.0 && (dy > 0.0 || (dy == 0.0 && dx < 0.0)));
                };
                if (!ownsEdge(a, b) || !ownsEdge(b, c) || !ownsEdge(c, a)) {
                    continue;
                }
                const double z = a.z() - (normal.x() * (px - a.x()) + normal.y() * (py - a.y())) / normal.z();
                crossings[row_off + static_cast<size_t>(iy)].push_back(Crossing {z, delta});
            }
        }
    }

    size_t closed_cols = 0;
    size_t open_cols = 0;
    for (size_t idx = 0; idx < crossings.size(); ++idx) {
        auto& hits = crossings[idx];
        if (hits.empty()) {
            continue;
        }
        std::sort(hits.begin(), hits.end(), [](const Crossing& x, const Crossing& y) { return x.z < y.z; });
        int winding = 0;
        bool invalid = false;
        std::vector<ma_map::StaticTerrainBuilder::ZSpan> solid_spans;
        double enter_z = 0.0;
        for (size_t i = 0; i < hits.size();) {
            size_t j = i + 1;
            int d = hits[i].delta;
            while (j < hits.size() && std::abs(hits[j].z - hits[i].z) <= 1e-9) {
                d += hits[j++].delta;
            }
            const int prev_w = winding;
            winding += d;
            if (winding < 0) {
                invalid = true;
                break;
            }
            if (prev_w == 0 && winding > 0) {
                enter_z = hits[i].z;
            } else if (prev_w > 0 && winding == 0) {
                solid_spans.push_back(
                    ma_map::StaticTerrainBuilder::ZSpan {static_cast<float>(enter_z), static_cast<float>(hits[i].z)}
                );
            }
            i = j;
        }
        if (!invalid && winding == 0 && !solid_spans.empty()) {
            col_grid.occ_spans[idx] = std::move(solid_spans);
            ++closed_cols;
        } else {
            ++open_cols;
            for (const auto& h: hits) {
                const float zf = static_cast<float>(h.z);
                col_grid.occ_spans[idx].push_back(ma_map::StaticTerrainBuilder::ZSpan {zf - 0.003f, zf + 0.003f});
            }
        }
    }
    crossings.clear();
    crossings.shrink_to_fit();

    // 2. 三角面重心网格采样，确保薄竖直墙、窄栏杆和未过格心的几何不漏检
    size_t surface_samples = 0;
    for (size_t t = 0; t < mesh.triangle_num(); ++t) {
        const Eigen::Vector3d& a = mesh.v[3 * t];
        const Eigen::Vector3d& b = mesh.v[3 * t + 1];
        const Eigen::Vector3d& c = mesh.v[3 * t + 2];
        const double max_edge = std::max({(b - a).norm(), (c - a).norm(), (c - b).norm()});
        const int n_steps = std::max(1, static_cast<int>(std::ceil(max_edge / sample_step)));
        const double inv_n = 1.0 / static_cast<double>(n_steps);
        const Eigen::Vector3d e1 = b - a;
        const Eigen::Vector3d e2 = c - a;

        for (int i = 0; i <= n_steps; ++i) {
            const double u = static_cast<double>(i) * inv_n;
            for (int j = 0; j <= n_steps - i; ++j) {
                const double v = static_cast<double>(j) * inv_n;
                const Eigen::Vector3d p = a + u * e1 + v * e2;
                const int ix = static_cast<int>(std::floor(p.x() * inv_res)) - min_vx;
                const int iy = static_cast<int>(std::floor(p.y() * inv_res)) - min_vy;
                if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) {
                    continue;
                }
                const size_t idx = static_cast<size_t>(ix) * static_cast<size_t>(ny) + static_cast<size_t>(iy);
                const float zf = static_cast<float>(p.z());
                auto& col = col_grid.occ_spans[idx];
                if (!col.empty() && zf >= col.back().z_lo - 0.005f && zf <= col.back().z_hi + 0.005f) {
                    col.back().z_lo = std::min(col.back().z_lo, zf);
                    col.back().z_hi = std::max(col.back().z_hi, zf);
                } else {
                    col.push_back(ma_map::StaticTerrainBuilder::ZSpan {zf, zf});
                    if (col.size() > 64) {
                        compactColumnSpans(col, 0.005f);
                    }
                }
                ++surface_samples;
            }
        }
    }

    ma_map::StaticTerrainBuilder builder(opt.terrain_cfg);
    auto result = builder.build(col_grid);
    std::printf(
        "[stl] closed_cols=%zu open_cols=%zu surface_samples=%zu elapsed=%.1f ms\n",
        closed_cols,
        open_cols,
        surface_samples,
        msSince(t0)
    );
    return result;
}

auto buildFromPcd(const CliOptions& opt) -> ma_map::StaticTerrainBuilder::BuildResult {
    const auto t0 = Clock::now();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(opt.input_path, cloud) != 0 || cloud.empty()) {
        throw std::runtime_error("Failed to load non-empty PCD file: " + opt.input_path);
    }

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    std::vector<Eigen::Vector3d> pts;
    pts.reserve(cloud.size());
    for (const auto& pt: cloud.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        Eigen::Vector3d p(
            static_cast<double>(pt.x) + opt.offset.x(),
            static_cast<double>(pt.y) + opt.offset.y(),
            static_cast<double>(pt.z) + opt.offset.z()
        );
        pts.push_back(p);
        min_x = std::min(min_x, p.x());
        max_x = std::max(max_x, p.x());
        min_y = std::min(min_y, p.y());
        max_y = std::max(max_y, p.y());
        min_z = std::min(min_z, p.z());
        max_z = std::max(max_z, p.z());
    }
    if (pts.empty()) {
        throw std::runtime_error("PCD contains no finite points: " + opt.input_path);
    }

    if (opt.ground_to_zero) {
        const double dz = -min_z;
        for (auto& p: pts) {
            p.z() += dz;
        }
        max_z += dz;
        min_z = 0.0;
    }

    const double res = opt.resolution;
    const double inv_res = 1.0 / res;
    const int32_t min_vx = static_cast<int32_t>(std::floor(min_x * inv_res));
    const int32_t max_vx = static_cast<int32_t>(std::floor(max_x * inv_res));
    const int32_t min_vy = static_cast<int32_t>(std::floor(min_y * inv_res));
    const int32_t max_vy = static_cast<int32_t>(std::floor(max_y * inv_res));
    const int nx = static_cast<int>(max_vx - min_vx + 1);
    const int ny = static_cast<int>(max_vy - min_vy + 1);

    std::printf(
        "[pcd] %s: %zu pts, bbox min [%.3f %.3f %.3f] max [%.3f %.3f %.3f], grid %dx%d @ %.3fm\n",
        opt.input_path.c_str(),
        pts.size(),
        min_x,
        min_y,
        min_z,
        max_x,
        max_y,
        max_z,
        nx,
        ny,
        res
    );

    // 按 (ix, iy, iz) 聚合每体素精确 [z_min, z_max] 与点数，支持 pcd_min_pts 过滤孤立飞点
    struct VoxelKey {
        int32_t ix;
        int32_t iy;
        int32_t iz;
        auto operator==(const VoxelKey& o) const -> bool = default;
    };
    struct VoxelKeyHash {
        auto operator()(const VoxelKey& k) const noexcept -> size_t {
            uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(k.ix)) * 73856093ULL;
            h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.iy)) * 19349663ULL;
            h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.iz)) * 83492791ULL;
            h ^= (h >> 33U);
            h *= 0xff51afd7ed558ccdULL;
            h ^= (h >> 33U);
            return static_cast<size_t>(h);
        }
    };
    struct VoxelStat {
        float z_lo = std::numeric_limits<float>::infinity();
        float z_hi = -std::numeric_limits<float>::infinity();
        uint32_t count = 0;
    };

    std::unordered_map<VoxelKey, VoxelStat, VoxelKeyHash> voxels;
    voxels.reserve(pts.size() / 4 + 1024);

    for (const auto& p: pts) {
        if (p.z() < opt.z_filter_min || p.z() > opt.z_filter_max) {
            continue;
        }
        const int32_t ix = static_cast<int32_t>(std::floor(p.x() * inv_res)) - min_vx;
        const int32_t iy = static_cast<int32_t>(std::floor(p.y() * inv_res)) - min_vy;
        const int32_t iz = static_cast<int32_t>(std::floor(p.z() * inv_res));
        if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) {
            continue;
        }
        auto& st = voxels[VoxelKey {ix, iy, iz}];
        const float zf = static_cast<float>(p.z());
        st.z_lo = std::min(st.z_lo, zf);
        st.z_hi = std::max(st.z_hi, zf);
        ++st.count;
    }

    const Eigen::Vector2d origin(static_cast<double>(min_vx) * res, static_cast<double>(min_vy) * res);
    const grid_map::GridGeometry geom =
        grid_map::GridGeometry::fromOriginAndSize(origin, Eigen::Vector2i(nx, ny), res);
    auto col_grid =
        ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(geom, /*solid_mesh=*/false, /*ray_clearing=*/false);

    for (const auto& kv: voxels) {
        if (static_cast<int>(kv.second.count) < opt.pcd_min_pts) {
            continue;
        }
        const size_t col_idx =
            static_cast<size_t>(kv.first.ix) * static_cast<size_t>(ny) + static_cast<size_t>(kv.first.iy);
        col_grid.occ_spans[col_idx].push_back(ma_map::StaticTerrainBuilder::ZSpan {kv.second.z_lo, kv.second.z_hi});
    }

    ma_map::StaticTerrainBuilder builder(opt.terrain_cfg);
    auto result = builder.build(col_grid);
    std::printf("[pcd] occupied_3d_voxels=%zu elapsed=%.1f ms\n", voxels.size(), msSince(t0));
    return result;
}

auto buildFromFrames(const CliOptions& opt) -> ma_map::StaticTerrainBuilder::BuildResult {
    const auto t0 = Clock::now();
    std::ifstream f(opt.input_path, std::ios::binary);
    if (!f) {
        throw std::runtime_error("Cannot open frames file: " + opt.input_path);
    }

    int32_t n_frames = 0;
    f.read(reinterpret_cast<char*>(&n_frames), 4);
    if (!f || n_frames <= 0) {
        throw std::runtime_error("Invalid or empty frames file: " + opt.input_path);
    }

    ma_map::SensorModel::Config scfg;
    ma_map::OnlineTerrainMapper::Config mcfg;
    mcfg.resolution = opt.resolution;
    mcfg.terrain = opt.terrain_cfg;

    if (!opt.config_path.empty()) {
        rog_map::Config rcfg(opt.config_path);
        scfg.t_base_lidar = rcfg.perception_t_base_lidar;
        scfg.rpy_deg = rcfg.perception_rpy_deg;
        scfg.fov_min_elev_deg = rcfg.perception_fov_min_elev_deg;
        scfg.fov_max_elev_deg = rcfg.perception_fov_max_elev_deg;
        scfg.fov_margin_deg = rcfg.perception_fov_margin_deg;
        scfg.min_range = rcfg.perception_sensor_min_range;
        scfg.max_range = rcfg.perception_sensor_max_range;
        scfg.self_filter_min = rcfg.perception_self_filter_min;
        scfg.self_filter_max = rcfg.perception_self_filter_max;

        mcfg.trust_range = rcfg.perception_map_trust_range;
        mcfg.clear_range = rcfg.perception_map_clear_range;
        mcfg.clear_stop_margin = rcfg.perception_map_clear_stop_margin;
        mcfg.min_z = rcfg.perception_map_min_z;
        mcfg.max_z = rcfg.perception_map_max_z;
        mcfg.static_min_hits = rcfg.perception_map_static_min_hits;
        mcfg.static_clear_ratio = rcfg.perception_map_static_clear_ratio;
        mcfg.tilt_jump_deg = rcfg.perception_map_tilt_jump_deg;
        mcfg.tilt_hold_frames = rcfg.perception_map_tilt_hold_frames;
    }
    if (opt.has_t_bl) {
        scfg.t_base_lidar = opt.t_base_lidar;
    }
    if (opt.has_rpy) {
        scfg.rpy_deg = opt.rpy_deg;
    }

    ma_map::SensorModel sensor(scfg);
    ma_map::OnlineTerrainMapper mapper(mcfg);

    std::vector<float> xyz_buf;
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(20000);

    for (int32_t fi = 0; fi < n_frames; ++fi) {
        double buf[8] {};
        f.read(reinterpret_cast<char*>(buf), sizeof(buf));
        int32_t np = 0;
        f.read(reinterpret_cast<char*>(&np), 4);
        if (!f || np < 0) {
            break;
        }
        xyz_buf.resize(static_cast<size_t>(np) * 3U);
        if (np > 0) {
            f.read(reinterpret_cast<char*>(xyz_buf.data()), static_cast<std::streamsize>(np) * 12);
        }
        if (!f) {
            break;
        }

        const Eigen::Vector3d p_base(buf[1] + opt.offset.x(), buf[2] + opt.offset.y(), buf[3] + opt.offset.z());
        const Eigen::Quaterniond q_base(buf[4], buf[5], buf[6], buf[7]);
        const ma_map::SensorPose sp = sensor.computePose(p_base, q_base);

        pts.clear();
        pts.reserve(static_cast<size_t>(np));
        for (int32_t k = 0; k < np; ++k) {
            pts.emplace_back(
                static_cast<double>(xyz_buf[3U * static_cast<size_t>(k) + 0U]) + opt.offset.x(),
                static_cast<double>(xyz_buf[3U * static_cast<size_t>(k) + 1U]) + opt.offset.y(),
                static_cast<double>(xyz_buf[3U * static_cast<size_t>(k) + 2U]) + opt.offset.z()
            );
        }

        (void)mapper.integrateFrame(pts, sp, q_base, sensor, /*build_local_preview=*/false);
    }

    auto built = mapper.buildGlobalStaticMap();
    if (!built.has_value()) {
        throw std::runtime_error("OnlineTerrainMapper produced empty static map from frames: " + opt.input_path);
    }
    const auto st = mapper.stats();
    std::printf(
        "[frames] %s: %u frames, chunks=%zu footprint_cells=%zu elapsed=%.1f ms\n",
        opt.input_path.c_str(),
        st.frame_count,
        st.chunk_count,
        st.footprint_cells,
        msSince(t0)
    );
    return *built;
}

auto filterLargestFreeComponent(ma_map::StaticTerrainBuilder::BuildResult& res) -> int {
    const int nx = res.geometry.voxel_num.x();
    const int ny = res.geometry.voxel_num.y();
    const size_t n = res.cells.size();
    std::vector<int> comp_id(n, -1);
    int best_comp = -1;
    int best_size = 0;
    int cur_comp = 0;

    std::queue<int> q;
    constexpr int kDx[4] = {1, -1, 0, 0};
    constexpr int kDy[4] = {0, 0, 1, -1};

    for (size_t idx = 0; idx < n; ++idx) {
        if (res.cells[idx] != ma_map::CellState::FREE || comp_id[idx] != -1) {
            continue;
        }
        int sz = 0;
        comp_id[idx] = cur_comp;
        q.push(static_cast<int>(idx));
        while (!q.empty()) {
            const int cur = q.front();
            q.pop();
            ++sz;
            const int ix = cur / ny;
            const int iy = cur % ny;
            for (int d = 0; d < 4; ++d) {
                const int jx = ix + kDx[d];
                const int jy = iy + kDy[d];
                if (jx < 0 || jx >= nx || jy < 0 || jy >= ny) {
                    continue;
                }
                const int nidx = jx * ny + jy;
                if (res.cells[static_cast<size_t>(nidx)] == ma_map::CellState::FREE
                    && comp_id[static_cast<size_t>(nidx)] == -1)
                {
                    comp_id[static_cast<size_t>(nidx)] = cur_comp;
                    q.push(nidx);
                }
            }
        }
        if (sz > best_size) {
            best_size = sz;
            best_comp = cur_comp;
        }
        ++cur_comp;
    }

    int removed = 0;
    for (size_t idx = 0; idx < n; ++idx) {
        if (res.cells[idx] == ma_map::CellState::FREE && comp_id[idx] != best_comp) {
            res.cells[idx] = ma_map::CellState::OCCUPIED;
            ++removed;
        }
    }
    res.free_count -= removed;
    res.occupied_count += removed;
    return removed;
}

auto writeVisualizations(
    const ma_map::StaticTerrainBuilder::BuildResult& res,
    const std::string& viz_prefix,
    int scale
) -> void {
    const std::filesystem::path p(viz_prefix);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }

    const int nx = res.geometry.voxel_num.x();
    const int ny = res.geometry.voxel_num.y();

    // 标准地图图像朝向：col = ix，row = ny - 1 - iy（与 .pgm / RViz 俯视方向一致）
    cv::Mat occ_img(ny, nx, CV_8UC1, cv::Scalar(205));
    cv::Mat ground_raw(ny, nx, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    cv::Mat overhang_img(ny, nx, CV_8UC3, cv::Scalar(205, 205, 205));

    float g_min = std::numeric_limits<float>::infinity();
    float g_max = -std::numeric_limits<float>::infinity();

    for (int ix = 0; ix < nx; ++ix) {
        const size_t row_off = static_cast<size_t>(ix) * static_cast<size_t>(ny);
        for (int iy = 0; iy < ny; ++iy) {
            const size_t idx = row_off + static_cast<size_t>(iy);
            const int img_r = ny - 1 - iy;
            const int img_c = ix;

            const auto st = res.cells[idx];
            if (st == ma_map::CellState::FREE) {
                occ_img.at<uint8_t>(img_r, img_c) = 254;
                overhang_img.at<cv::Vec3b>(img_r, img_c) = cv::Vec3b(254, 254, 254);
            } else if (st == ma_map::CellState::OCCUPIED) {
                occ_img.at<uint8_t>(img_r, img_c) = 0;
                overhang_img.at<cv::Vec3b>(img_r, img_c) = cv::Vec3b(0, 0, 0);
            }

            const float gz = res.terrain_prior.ground_z[idx];
            if (std::isfinite(gz)) {
                ground_raw.at<float>(img_r, img_c) = gz;
                g_min = std::min(g_min, gz);
                g_max = std::max(g_max, gz);
            }

            const float oh = res.terrain_prior.overhang_min_z[idx];
            if (std::isfinite(oh)) {
                // 青色标记已知可通行悬空（限高杆/洞顶）
                overhang_img.at<cv::Vec3b>(img_r, img_c) = cv::Vec3b(255, 210, 0);
            }
        }
    }

    cv::Mat ground_u8(ny, nx, CV_8UC1, cv::Scalar(0));
    const float g_span = (std::isfinite(g_min) && g_max > g_min + 1e-4f) ? (g_max - g_min) : 1.0f;
    for (int r = 0; r < ny; ++r) {
        for (int c = 0; c < nx; ++c) {
            const float gz = ground_raw.at<float>(r, c);
            if (std::isfinite(gz)) {
                const float t = std::clamp((gz - g_min) / g_span, 0.0f, 1.0f);
                ground_u8.at<uint8_t>(r, c) = static_cast<uint8_t>(std::lround(t * 255.0f));
            }
        }
    }
    cv::Mat ground_bgr;
    cv::applyColorMap(ground_u8, ground_bgr, cv::COLORMAP_TURBO);
    for (int r = 0; r < ny; ++r) {
        for (int c = 0; c < nx; ++c) {
            if (!std::isfinite(ground_raw.at<float>(r, c))) {
                ground_bgr.at<cv::Vec3b>(r, c) = cv::Vec3b(60, 60, 60);
            }
        }
    }

    auto maybeScale = [scale](const cv::Mat& src) -> cv::Mat {
        if (scale <= 1) {
            return src;
        }
        cv::Mat dst;
        cv::resize(src, dst, cv::Size(), scale, scale, cv::INTER_NEAREST);
        return dst;
    };

    cv::imwrite(viz_prefix + "_occ.png", maybeScale(occ_img));
    cv::imwrite(viz_prefix + "_ground.png", maybeScale(ground_bgr));
    cv::imwrite(viz_prefix + "_overhang.png", maybeScale(overhang_img));
}

} // namespace

int main(int argc, char** argv) {
    CliOptions opt;
    if (!parseArgs(argc, argv, opt)) {
        printUsage(argv[0]);
        return 2;
    }

    const std::string mode = detectMode(opt);
    if (mode != "stl" && mode != "pcd" && mode != "frames") {
        std::cerr << "Cannot infer input mode from extension (use --mode stl|pcd|frames): " << opt.input_path << "\n";
        return 2;
    }

    try {
        ma_map::StaticTerrainBuilder::BuildResult result;
        if (mode == "stl") {
            result = buildFromStl(opt);
        } else if (mode == "pcd") {
            result = buildFromPcd(opt);
        } else {
            result = buildFromFrames(opt);
        }

        if (opt.keep_largest_cc) {
            const int removed = filterLargestFreeComponent(result);
            std::printf("[filter] keep-largest-cc: converted %d disconnected FREE cells to OCCUPIED\n", removed);
        }

        ma_map::SaveOptions save_opts;
        save_opts.crop_to_explored = opt.crop_to_explored;
        save_opts.margin_cells = opt.margin_cells;
        save_opts.save_terrain_prior = true;

        const auto saved = ma_map::MapIO::save(
            opt.output_prefix,
            result.geometry,
            result.cells,
            save_opts,
            &result.terrain_prior
        );
        if (!saved.has_value()) {
            std::cerr << "[ERROR] MapIO::save failed: " << saved.error() << "\n";
            return 1;
        }

        if (!opt.viz_prefix.empty()) {
            writeVisualizations(result, opt.viz_prefix, opt.viz_scale);
        }

        std::printf(
            "[done] saved %s + %s%s | grid=%dx%d origin=[%.3f, %.3f] res=%.3f | free=%d occ=%d unk=%d overhang=%d\n",
            saved->pgm_path.c_str(),
            saved->yaml_path.c_str(),
            saved->saved_terrain_prior ? (" + " + saved->terrain_prior_path).c_str() : "",
            saved->saved_geometry.voxel_num.x(),
            saved->saved_geometry.voxel_num.y(),
            saved->saved_geometry.origin.x(),
            saved->saved_geometry.origin.y(),
            saved->saved_geometry.resolution,
            saved->free_count,
            saved->occupied_count,
            saved->unknown_count,
            result.overhang_count
        );
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
}
