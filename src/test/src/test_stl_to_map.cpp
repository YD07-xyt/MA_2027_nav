// STL 场地 -> map 模块：完整 3D 占据地图（ROG-Map） -> 2D 占据（按列判断） -> 2D ESDF（GridMap），并输出可视化。
// 不依赖 ROS。用法：
//   test_stl_to_map [stl] [yaml] [out_dir]
//   默认 images/rmuc_2027.stl  src/test/config/stl_map.yaml  images/output/stl_map
// 输出（out_dir 下）：
//   occ_3d_voxels.ply   ROG 占据体素画成实心立方体（只保留外露面，按高度着色），open3d / pcl_viewer 查看
//   occ_3d_height.png   3D 占据俯视：每列最高占据体素高度
//   occ_2d.png          2D 占据（白=可通行，黑=障碍）
//   esdf_2d.png         2D ESDF 热力图（障碍内为负，白色等距线每 0.5 m）
//   summary.png         上面三张拼图
// 3D -> 2D 规则（每个 (x, y) 列）：
//   1) 地面 = 最低的占据体素 z，满足其上方 robot_height 内全部空闲，且高度 <= max_ground_height；
//   2) 无地面 -> 障碍；
//   3) 与 8 邻格地面高差 > max_step_height -> 障碍（台阶、墙脚两侧都标）。
#include "map/3d_occ_map/rog_map.h"
#include "map/grid_map.hpp"
#include "stl_loader.hpp"
#include "utils/yaml_loader.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

auto ms_since(Clock::time_point t0) -> double {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// ROG 占据查询结果的稠密拷贝，范围同 STL 体素网格（索引 + min_id = 全局体素索引）
struct Occ3d {
    Eigen::Vector3i min_id, dim;
    double res = 0.05;
    std::vector<uint8_t> occ;
    auto addr(int x, int y, int z) const -> size_t {
        return (static_cast<size_t>(x) * dim.y() + y) * dim.z() + z;
    }
    auto at(int x, int y, int z) const -> bool {
        if (x < 0 || y < 0 || z < 0 || x >= dim.x() || y >= dim.y() || z >= dim.z()) return false;
        return occ[addr(x, y, z)] != 0;
    }
};

// 高度 -> BGR（turbo）
auto height_color(double t) -> cv::Vec3b {
    static cv::Mat lut = [] {
        cv::Mat ramp(1, 256, CV_8UC1);
        for (int i = 0; i < 256; ++i) ramp.at<uint8_t>(0, i) = static_cast<uint8_t>(i);
        cv::Mat out;
        cv::applyColorMap(ramp, out, cv::COLORMAP_TURBO);
        return out;
    }();
    const int i = std::clamp(static_cast<int>(std::lround(t * 255.0)), 0, 255);
    return lut.at<cv::Vec3b>(0, i);
}

// 占据体素 -> 外露面立方体网格（binary PLY，顶点带法向和颜色，每个面 4 个独立顶点，保证着色是平面块）
auto write_voxel_ply(const Occ3d& m, const std::string& path, double z_color_max) -> size_t {
    struct Vtx {
        float x, y, z, nx, ny, nz;
        uint8_t r, g, b;
    } __attribute__((packed));
    std::vector<Vtx> vtx;
    std::vector<int> quads;
    // 6 个方向：法向 + 该面 4 个角（单位立方体内坐标，逆时针朝外）
    static const int kDir[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    static const int kCorner[6][4][3] = {
        {{1, 0, 0}, {1, 1, 0}, {1, 1, 1}, {1, 0, 1}},
        {{0, 0, 0}, {0, 0, 1}, {0, 1, 1}, {0, 1, 0}},
        {{0, 1, 0}, {0, 1, 1}, {1, 1, 1}, {1, 1, 0}},
        {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}},
        {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}},
        {{0, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 0, 0}},
    };
    for (int x = 0; x < m.dim.x(); ++x) {
        for (int y = 0; y < m.dim.y(); ++y) {
            for (int z = 0; z < m.dim.z(); ++z) {
                if (!m.at(x, y, z)) continue;
                const Eigen::Vector3i g = Eigen::Vector3i(x, y, z) + m.min_id;
                const double zc = (g.z() + 0.5) * m.res;
                const cv::Vec3b c = height_color(zc / z_color_max);
                for (int d = 0; d < 6; ++d) {
                    if (m.at(x + kDir[d][0], y + kDir[d][1], z + kDir[d][2])) continue; // 与相邻占据体素共享的面不画
                    const int base = static_cast<int>(vtx.size());
                    for (const auto& k: kCorner[d]) {
                        vtx.push_back(Vtx {
                            static_cast<float>((g.x() + k[0]) * m.res),
                            static_cast<float>((g.y() + k[1]) * m.res),
                            static_cast<float>((g.z() + k[2]) * m.res),
                            static_cast<float>(kDir[d][0]),
                            static_cast<float>(kDir[d][1]),
                            static_cast<float>(kDir[d][2]),
                            c[2],
                            c[1],
                            c[0],
                        });
                    }
                    quads.push_back(base);
                }
            }
        }
    }
    std::ofstream f(path, std::ios::binary);
    f << "ply\nformat binary_little_endian 1.0\n"
      << "element vertex " << vtx.size() << "\n"
      << "property float x\nproperty float y\nproperty float z\n"
      << "property float nx\nproperty float ny\nproperty float nz\n"
      << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
      << "element face " << quads.size() * 2 << "\n"
      << "property list uchar int vertex_indices\nend_header\n";
    f.write(reinterpret_cast<const char*>(vtx.data()), static_cast<std::streamsize>(vtx.size() * sizeof(Vtx)));
    for (const int b: quads) {
        // 一个正方形拆两个三角形
        const int tri[2][3] = {{b, b + 1, b + 2}, {b, b + 2, b + 3}};
        for (const auto& t: tri) {
            const uint8_t n = 3;
            f.write(reinterpret_cast<const char*>(&n), 1);
            f.write(reinterpret_cast<const char*>(t), 12);
        }
    }
    return quads.size();
}

// 2D 栅格 (x, y) -> 图片：行 = H-1-y，列 = x，放大 scale 倍
auto to_image(const cv::Mat& xy, int scale) -> cv::Mat {
    cv::Mat img;
    cv::flip(xy.t(), img, 0);
    if (scale > 1) cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_NEAREST);
    return img;
}

auto put_title(cv::Mat& img, const std::string& text) -> void {
    cv::rectangle(img, cv::Point(0, 0), cv::Point(img.cols, 26), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(img, text, cv::Point(6, 19), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
}

} // namespace

int main(int argc, char** argv) {
    const std::string root = ROOT_DIR;
    const std::string stl_path = argc > 1 ? argv[1] : root + "images/rmuc_2027.stl";
    const std::string yaml_path = argc > 2 ? argv[2] : root + "src/test/config/stl_map.yaml";
    const std::string out_dir = argc > 3 ? argv[3] : root + "images/output/stl_map";
    std::filesystem::create_directories(out_dir);

    yaml_loader::YamlLoader loader(yaml_path);
    bool ground_to_zero = true, do_fill = true;
    double sample_step = 0.0125, max_ground_height = 1.0;
    loader.LoadParam("stl_test/ground_to_zero", ground_to_zero, true);
    loader.LoadParam("stl_test/sample_step", sample_step, 0.0125);
    loader.LoadParam("stl_test/fill_solid", do_fill, true);
    loader.LoadParam("stl_test/max_ground_height", max_ground_height, 1.0);

    // ---------- 1. 读 STL + 体素化 ----------
    auto t0 = Clock::now();
    auto mesh = test_utils::load_stl(stl_path);
    if (ground_to_zero) mesh.translate(Eigen::Vector3d(0, 0, -mesh.min.z()));
    const double t_load = ms_since(t0);
    std::printf("[stl] %s: %zu triangles, bbox min [%.3f %.3f %.3f] max [%.3f %.3f %.3f]\n", stl_path.c_str(),
                mesh.triangle_num(), mesh.min.x(), mesh.min.y(), mesh.min.z(), mesh.max.x(), mesh.max.y(),
                mesh.max.z());

    rog_map::Config cfg(yaml_path);
    const double res = cfg.resolution;
    t0 = Clock::now();
    test_utils::VoxelizeStat vstat;
    auto vg = test_utils::voxelize(mesh, res, sample_step, vstat);
    if (do_fill) test_utils::fill_solid(vg, vstat);
    const double t_vox = ms_since(t0);
    std::printf("[stl] samples %zu, surface voxels %zu, inside voxels %zu, open columns (fallback surface) %zu\n",
                vstat.sample_num, vstat.surface_num, vstat.inside_num, vstat.open_column_num);

    // ---------- 2. 体素中心点云 -> ROG-Map 3D 占据 ----------
    rog_map::PointCloud cloud;
    cloud.reserve(vstat.surface_num + vstat.inside_num);
    for (int x = 0; x < vg.dim.x(); ++x)
        for (int y = 0; y < vg.dim.y(); ++y)
            for (int z = 0; z < vg.dim.z(); ++z) {
                if (!vg.occupied(x, y, z)) continue;
                const Eigen::Vector3d c = ((Eigen::Vector3i(x, y, z) + vg.min_id).cast<double>().array() + 0.5) * res;
                rog_map::PclPoint p;
                p.x = static_cast<float>(c.x());
                p.y = static_cast<float>(c.y());
                p.z = static_cast<float>(c.z());
                p.intensity = 0.f;
                cloud.push_back(p);
            }
    const size_t expected_occ = cloud.size();
    vg.flags.clear();
    vg.flags.shrink_to_fit();

    t0 = Clock::now();
    rog_map::ROGMap rog;
    rog.setConfig(cfg);
    rog.init();
    const double t_init = ms_since(t0);
    t0 = Clock::now();
    rog.updateOccPointCloud(cloud);
    const double t_insert = ms_since(t0);

    // 回读 ROG 占据（全部经 map 模块查询），后续 2D 与可视化都只用这份结果
    Occ3d m;
    m.res = res;
    m.min_id = vg.min_id;
    m.dim = vg.dim;
    m.occ.assign(static_cast<size_t>(m.dim.prod()), 0);
    t0 = Clock::now();
    size_t rog_occ = 0;
    for (int x = 0; x < m.dim.x(); ++x)
        for (int y = 0; y < m.dim.y(); ++y)
            for (int z = 0; z < m.dim.z(); ++z) {
                const Eigen::Vector3d c = ((Eigen::Vector3i(x, y, z) + m.min_id).cast<double>().array() + 0.5) * res;
                if (rog.isOccupied(c)) {
                    m.occ[m.addr(x, y, z)] = 1;
                    ++rog_occ;
                }
            }
    const double t_query = ms_since(t0);
    std::printf("[3d] inserted %zu voxels, ROG occupied %zu (%s)\n", expected_occ, rog_occ,
                rog_occ == expected_occ ? "match" : "MISMATCH");

    // ---------- 3. 3D -> 2D 占据（按列） ----------
    const double gx_l = cfg.grid_map_x_left, gx_r = cfg.grid_map_x_right;
    const double gy_l = cfg.grid_map_y_left, gy_r = cfg.grid_map_y_right;
    grid_map::GridMap gm;
    gm.init(gx_l, gx_r, gy_l, gy_r, cfg.grid_map_resolution);
    const Eigen::Vector2i n2 = gm.getVoxelNum();
    const int clear_i = static_cast<int>(std::ceil(cfg.terrain_robot_height / res - 1e-6));
    const int max_ground_i = static_cast<int>(std::floor(max_ground_height / res));
    const double step_tol = cfg.terrain_max_step_height + 1e-6;

    t0 = Clock::now();
    // ground(x, y)：地面顶面高度(m)，NaN 表示无地面
    grid_map::RowMatrixXd ground = grid_map::RowMatrixXd::Constant(n2.x(), n2.y(), std::nan(""));
    for (int ix = 0; ix < n2.x(); ++ix) {
        for (int iy = 0; iy < n2.y(); ++iy) {
            Eigen::Vector2d p;
            gm.indexToPos(Eigen::Vector2i(ix, iy), p);
            const Eigen::Vector3i g = test_utils::voxel_index(Eigen::Vector3d(p.x(), p.y(), 0.0), res) - m.min_id;
            if (g.x() < 0 || g.y() < 0 || g.x() >= m.dim.x() || g.y() >= m.dim.y()) continue;
            for (int z = 0; z < m.dim.z(); ++z) {
                if (z + m.min_id.z() > max_ground_i) break;
                if (!m.at(g.x(), g.y(), z)) continue;
                bool clear = true;
                for (int k = 1; k <= clear_i && clear; ++k) clear = !m.at(g.x(), g.y(), z + k);
                if (clear) {
                    ground(ix, iy) = (z + m.min_id.z() + 1) * res;
                    break;
                }
            }
        }
    }
    grid_map::RowMatrixXi occ2d = grid_map::RowMatrixXi::Zero(n2.x(), n2.y());
    int no_ground = 0, step_obs = 0;
    for (int ix = 0; ix < n2.x(); ++ix) {
        for (int iy = 0; iy < n2.y(); ++iy) {
            const double h = ground(ix, iy);
            if (std::isnan(h)) {
                occ2d(ix, iy) = 1;
                ++no_ground;
                continue;
            }
            for (int dx = -1; dx <= 1 && !occ2d(ix, iy); ++dx)
                for (int dy = -1; dy <= 1; ++dy) {
                    const int jx = ix + dx, jy = iy + dy;
                    if (jx < 0 || jy < 0 || jx >= n2.x() || jy >= n2.y()) continue;
                    const double hn = ground(jx, jy);
                    if (!std::isnan(hn) && std::abs(hn - h) > step_tol) {
                        occ2d(ix, iy) = 1;
                        ++step_obs;
                        break;
                    }
                }
        }
    }
    const double t_2d = ms_since(t0);
    const int obs_total = occ2d.sum();
    std::printf("[2d] grid %d x %d @ %.3f m: obstacles %d (no ground %d, step %d), free %d\n", n2.x(), n2.y(),
                gm.getResolution(), obs_total, no_ground, step_obs, n2.prod() - obs_total);

    // ---------- 4. 2D ESDF ----------
    t0 = Clock::now();
    gm.setMap(occ2d);
    const double t_esdf = ms_since(t0);
    const grid_map::RowMatrixXd esdf = gm.getMap();
    std::printf("[esdf] min %.3f m, max %.3f m\n", esdf.minCoeff(), esdf.maxCoeff());

    // ---------- 5. 可视化 ----------
    t0 = Clock::now();
    const double z_color_max = std::max(mesh.max.z(), res);
    const size_t quads = write_voxel_ply(m, out_dir + "/occ_3d_voxels.ply", z_color_max);
    std::printf("[viz] occ_3d_voxels.ply: %zu exposed faces\n", quads);

    // 3D 俯视：每列最高占据体素高度，与 2D 栅格同范围同分辨率
    cv::Mat height_xy(n2.x(), n2.y(), CV_8UC3, cv::Scalar(40, 40, 40));
    for (int ix = 0; ix < n2.x(); ++ix)
        for (int iy = 0; iy < n2.y(); ++iy) {
            Eigen::Vector2d p;
            gm.indexToPos(Eigen::Vector2i(ix, iy), p);
            const Eigen::Vector3i g = test_utils::voxel_index(Eigen::Vector3d(p.x(), p.y(), 0.0), res) - m.min_id;
            if (g.x() < 0 || g.y() < 0 || g.x() >= m.dim.x() || g.y() >= m.dim.y()) continue;
            for (int z = m.dim.z() - 1; z >= 0; --z) {
                if (!m.at(g.x(), g.y(), z)) continue;
                height_xy.at<cv::Vec3b>(ix, iy) = height_color((z + m.min_id.z() + 0.5) * res / z_color_max);
                break;
            }
        }

    cv::Mat occ_xy(n2.x(), n2.y(), CV_8UC3);
    for (int ix = 0; ix < n2.x(); ++ix)
        for (int iy = 0; iy < n2.y(); ++iy)
            occ_xy.at<cv::Vec3b>(ix, iy) = occ2d(ix, iy) ? cv::Vec3b(0, 0, 0) : cv::Vec3b(255, 255, 255);

    // ESDF：可通行区 0~2 m 用 turbo，障碍内按深度画灰度，每 0.5 m 一条白色等距线
    constexpr double kPosMax = 2.0, kNegMax = 0.5, kIso = 0.5;
    cv::Mat esdf_xy(n2.x(), n2.y(), CV_8UC3);
    for (int ix = 0; ix < n2.x(); ++ix)
        for (int iy = 0; iy < n2.y(); ++iy) {
            const double d = esdf(ix, iy);
            cv::Vec3b c;
            if (occ2d(ix, iy)) {
                const auto v = static_cast<uint8_t>(90 - 70 * std::clamp(-d / kNegMax, 0.0, 1.0));
                c = cv::Vec3b(v, v, v);
            } else {
                c = height_color(std::clamp(d / kPosMax, 0.0, 1.0));
                const double r = std::fmod(d, kIso);
                if (d > 0.1 && (r < 0.5 * gm.getResolution() || r > kIso - 0.5 * gm.getResolution()))
                    c = cv::Vec3b(255, 255, 255);
            }
            esdf_xy.at<cv::Vec3b>(ix, iy) = c;
        }

    cv::Mat img_h = to_image(height_xy, 2), img_o = to_image(occ_xy, 2), img_e = to_image(esdf_xy, 2);
    cv::imwrite(out_dir + "/occ_3d_height.png", img_h);
    cv::imwrite(out_dir + "/occ_2d.png", img_o);
    cv::imwrite(out_dir + "/esdf_2d.png", img_e);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "3D occ top height (0 - %.2f m)", z_color_max);
    put_title(img_h, buf);
    std::snprintf(buf, sizeof(buf), "2D occ: clearance %.2f m, step %.2f m", cfg.terrain_robot_height,
                  cfg.terrain_max_step_height);
    put_title(img_o, buf);
    put_title(img_e, "2D ESDF (0 - 2 m, iso 0.5 m)");
    cv::Mat summary;
    cv::vconcat(std::vector<cv::Mat> {img_h, img_o, img_e}, summary);
    cv::imwrite(out_dir + "/summary.png", summary);
    const double t_viz = ms_since(t0);

    std::printf("[time ms] load %.1f | voxelize %.1f | rog init %.1f | rog insert %.1f | rog query %.1f | "
                "2d %.1f | esdf %.1f | viz %.1f\n",
                t_load, t_vox, t_init, t_insert, t_query, t_2d, t_esdf, t_viz);
    std::printf("[out] %s\n", out_dir.c_str());
    return rog_occ == expected_occ ? 0 : 1;
}
