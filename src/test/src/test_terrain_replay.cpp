// 地形分析离线回放：读 terrain_replay_extract.py 导出的帧文件，逐帧喂 MaMap，输出可比较的统计。
// 不依赖 ROS。用法：
//   test_terrain_replay <frames.bin> <map.yaml> <out_prefix> [nav] [realtime] [static=<static.yaml>] [zshift=<m>]
//     nav      : 导航模式（set_mapping_model(false)），默认建图模式
//     realtime : 按 bag 时间戳节拍 sleep（ROG 遗忘用墙钟时间，测遗忘时需要）
//     static=  : 导航模式先 load_static_map(<static.yaml>)（建图模式忽略）
//     zshift=  : 每帧输入前从点云 z 与位姿 z 减去该值（m），用于抵消 LIO 世界系相对先验图的竖直偏移
// 输出：
//   <out_prefix>_final.pgm   最终 2D 占据（白=空闲，黑=障碍）
//   <out_prefix>_frames.csv  每帧：t, x, y, z, 障碍格数, 车周 3 m 内障碍格数, 足迹内障碍格数, update_map 耗时 ms,
//                            车周 5 m 内相对静态先验新增的障碍格数, 三段限高杆区域障碍格数及其新增数
//   <out_prefix>_extra.pgm   导航模式：整段运行相对静态先验新增障碍格的并集（0=新增，255=其它）
//   <out_prefix>_saved.pgm / .yaml / .terrain.bin  建图模式 save_explored_map 结果
//   stdout                   汇总：轨迹扫过的格中最终被判障碍的数量（车走过的地方不应是障碍）
#include "map/ma_map.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

struct Frame {
    double t;
    double p[3];
    double q[4];
    std::vector<float> xyz;
};

static auto read_frames(const std::string& path) -> std::vector<Frame> {
    std::ifstream f(path, std::ios::binary);
    int32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), 4);
    std::vector<Frame> frames(n);
    for (auto& fr: frames) {
        double buf[8];
        f.read(reinterpret_cast<char*>(buf), sizeof(buf));
        fr.t = buf[0];
        for (int i = 0; i < 3; ++i)
            fr.p[i] = buf[1 + i];
        for (int i = 0; i < 4; ++i)
            fr.q[i] = buf[4 + i];
        int32_t np = 0;
        f.read(reinterpret_cast<char*>(&np), 4);
        fr.xyz.resize(3 * np);
        f.read(reinterpret_cast<char*>(fr.xyz.data()), 12 * static_cast<std::streamsize>(np));
    }
    return frames;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: test_terrain_replay <frames.bin> <map.yaml> <out_prefix> [nav] [realtime]"
                     " [static=<static.yaml>] [zshift=<m>]\n";
        return 2;
    }
    bool nav = false, realtime = false;
    std::string static_path;
    double zshift = 0.0;
    for (int i = 4; i < argc; ++i) {
        const std::string arg = argv[i];
        nav |= arg == "nav";
        realtime |= arg == "realtime";
        if (arg.rfind("static=", 0) == 0) static_path = arg.substr(7);
        if (arg.rfind("zshift=", 0) == 0) zshift = std::stod(arg.substr(7));
    }
    const auto frames = read_frames(argv[1]);
    const std::string prefix = argv[3];
    ma_map::MaMap map(argv[2]);
    map.set_mapping_model(!nav);
    grid_map::RowMatrixXi static_occ;
    if (nav && !static_path.empty()) {
        const auto loaded = map.load_static_map(static_path);
        if (!loaded) {
            std::cerr << "load_static_map failed: " << loaded.error() << '\n';
            return 3;
        }
        static_occ = map.get_global_occ();
        std::fprintf(
            stderr,
            "static map: occ=%d free=%d unknown=%d terrain_prior=%d\n",
            loaded->occupied_count,
            loaded->free_count,
            loaded->unknown_count,
            static_cast<int>(loaded->has_terrain_prior)
        );
    }
    // 导航专项区域（LIO 世界系）：两段限高杆与右侧悬空结构。
    struct Region {
        double x0, x1, y0, y1;
    };
    const Region regions[3] = {{7.0, 8.0, -4.78, -4.64}, {13.2, 14.2, -4.90, -4.76}, {10.0, 11.6, -9.0, -7.9}};

    const auto gm0 = map.get_grid_map();
    const double res = gm0->getResolution();
    // 记录每帧机器人世界坐标；建图模式允许动态扩图，结束时再映射到最终网格尺寸，避免索引越界。
    std::vector<Eigen::Vector2d> robot_positions;
    robot_positions.reserve(frames.size());
    const double foot_r = 0.15;
    const int foot_c = static_cast<int>(std::ceil(foot_r / res));

    std::ofstream csv(prefix + "_frames.csv");
    csv << "t,x,y,z,obs,obs_near3m,obs_footprint,ms,live_extra_occ,bar1_occ,bar2_occ,bar3_occ,bar1_extra,bar2_extra,"
           "bar3_extra\n";
    // 导航模式：累计整段运行中相对先验新增的障碍格（用于离线按 STL 分类）。
    std::vector<uint8_t> extra_union;
    if (static_occ.rows() > 0) extra_union.assign(static_cast<size_t>(static_occ.rows()) * static_occ.cols(), 0);
    const auto wall0 = std::chrono::steady_clock::now();
    const double t0 = frames.empty() ? 0.0 : frames.front().t;
    std::vector<double> ms_all;
    for (const auto& fr: frames) {
        if (realtime) {
            std::this_thread::sleep_until(wall0 + std::chrono::duration<double>(fr.t - t0));
        }
        rog_map::Pose pose;
        pose.first = utils::Vec3f(fr.p[0], fr.p[1], fr.p[2] - zshift);
        pose.second = utils::Quatf(fr.q[0], fr.q[1], fr.q[2], fr.q[3]);
        map.update_odom(pose);
        rog_map::PointCloud pc;
        pc.resize(fr.xyz.size() / 3);
        for (size_t i = 0; i < pc.size(); ++i) {
            pc[i].x = fr.xyz[3 * i];
            pc[i].y = fr.xyz[3 * i + 1];
            pc[i].z = fr.xyz[3 * i + 2] - static_cast<float>(zshift);
        }
        map.update_cloud(pc);
        const auto c0 = std::chrono::steady_clock::now();
        map.update_map();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
        ms_all.push_back(ms);

        const auto gm = map.get_grid_map();
        const auto occ = map.get_occupancy_2d();
        Eigen::Vector2i rid;
        gm->posToIndex(Eigen::Vector2d(fr.p[0], fr.p[1]), rid);
        robot_positions.emplace_back(fr.p[0], fr.p[1]);
        const bool check_extra = static_occ.rows() == occ.rows() && static_occ.cols() == occ.cols();
        int obs = 0, near = 0, foot = 0, extra = 0;
        const int near_c = static_cast<int>(3.0 / res);
        const int extra_c = static_cast<int>(5.0 / res);
        for (int x = 0; x < occ.rows(); ++x) {
            for (int y = 0; y < occ.cols(); ++y) {
                if (occ(x, y) != 1) continue;
                ++obs;
                const int dx = x - rid.x(), dy = y - rid.y();
                if (dx * dx + dy * dy <= near_c * near_c) ++near;
                if (dx * dx + dy * dy <= foot_c * foot_c) ++foot;
                if (check_extra && dx * dx + dy * dy <= extra_c * extra_c && static_occ(x, y) == 0) {
                    ++extra;
                    extra_union[static_cast<size_t>(x) * static_cast<size_t>(occ.cols()) + static_cast<size_t>(y)] = 1;
                }
            }
        }
        // 限高杆区域：统计每帧区域内的障碍格数（导航模式才有意义）；extra 为相对静态先验新增。
        int bar[3] = {0, 0, 0}, bar_extra[3] = {0, 0, 0};
        for (int r = 0; r < 3 && nav; ++r) {
            Eigen::Vector2i lo, hi;
            gm->posToIndex(Eigen::Vector2d(regions[r].x0, regions[r].y0), lo);
            gm->posToIndex(Eigen::Vector2d(regions[r].x1, regions[r].y1), hi);
            const int x0 = std::clamp(std::min(lo.x(), hi.x()), 0, static_cast<int>(occ.rows()) - 1);
            const int x1 = std::clamp(std::max(lo.x(), hi.x()), 0, static_cast<int>(occ.rows()) - 1);
            const int y0 = std::clamp(std::min(lo.y(), hi.y()), 0, static_cast<int>(occ.cols()) - 1);
            const int y1 = std::clamp(std::max(lo.y(), hi.y()), 0, static_cast<int>(occ.cols()) - 1);
            for (int x = x0; x <= x1; ++x)
                for (int y = y0; y <= y1; ++y) {
                    if (occ(x, y) != 1) continue;
                    ++bar[r];
                    if (check_extra && static_occ(x, y) == 0) ++bar_extra[r];
                }
        }
        csv << fr.t - t0 << ',' << fr.p[0] << ',' << fr.p[1] << ',' << fr.p[2] << ',' << obs << ',' << near << ','
            << foot << ',' << ms << ',' << extra << ',' << bar[0] << ',' << bar[1] << ',' << bar[2] << ','
            << bar_extra[0] << ',' << bar_extra[1] << ',' << bar_extra[2] << '\n';
    }

    const auto gm_final = map.get_grid_map();
    const Eigen::Vector2i nvox = gm_final->getVoxelNum();
    const auto occ = map.get_occupancy_2d();
    std::vector<char> traversed(static_cast<size_t>(nvox.x()) * static_cast<size_t>(nvox.y()), 0);
    for (const auto& rp: robot_positions) {
        Eigen::Vector2i rid;
        gm_final->posToIndex(rp, rid);
        for (int dx = -foot_c; dx <= foot_c; ++dx)
            for (int dy = -foot_c; dy <= foot_c; ++dy) {
                const int x = rid.x() + dx, y = rid.y() + dy;
                if (dx * dx + dy * dy > foot_c * foot_c) continue;
                if (x < 0 || y < 0 || x >= nvox.x() || y >= nvox.y()) continue;
                traversed[static_cast<size_t>(x) * static_cast<size_t>(nvox.y()) + static_cast<size_t>(y)] = 1;
            }
    }
    int trav = 0, trav_obs = 0, obs_total = 0;
    for (int x = 0; x < occ.rows(); ++x)
        for (int y = 0; y < occ.cols(); ++y) {
            obs_total += occ(x, y) == 1;
            if (!traversed[static_cast<size_t>(x) * static_cast<size_t>(nvox.y()) + static_cast<size_t>(y)]) continue;
            ++trav;
            trav_obs += occ(x, y) == 1;
        }
    // PGM：行 = y（从上到下 y 递减），列 = x
    std::ofstream pgm(prefix + "_final.pgm", std::ios::binary);
    pgm << "P5\n" << occ.rows() << ' ' << occ.cols() << "\n255\n";
    for (int y = occ.cols() - 1; y >= 0; --y)
        for (int x = 0; x < occ.rows(); ++x) {
            const unsigned char v = occ(x, y) == 1
                ? 0
                : (traversed[static_cast<size_t>(x) * static_cast<size_t>(nvox.y()) + static_cast<size_t>(y)] ? 160
                                                                                                              : 255);
            pgm.put(static_cast<char>(v));
        }
    // 持久地面层：每个 2D 格一行 x, y, h, flags（只写有地面的格）
    {
        const auto& gl = map.get_ground_layer();
        std::ofstream g(prefix + "_ground.csv");
        g << "x,y,h,flags,ub\n";
        const double gres = gl.config().resolution;
        for (int x = 0; x < nvox.x(); ++x)
            for (int y = 0; y < nvox.y(); ++y) {
                Eigen::Vector2d p;
                gm_final->indexToPos(Eigen::Vector2i(x, y), p);
                const auto* c =
                    gl.cell(static_cast<int>(std::floor(p.x() / gres)), static_cast<int>(std::floor(p.y() / gres)));
                if (c == nullptr || c->flags == 0) continue;
                g << p.x() << ',' << p.y() << ',' << c->h << ',' << int(c->flags) << ',' << c->ub << '\n';
            }
    }
    // 导航新增障碍格并集：行 = 世界 X（顶行为最大 X），列 = 世界 Y（与 MapIO 保存约定一致）。
    if (!extra_union.empty()) {
        std::ofstream xg(prefix + "_extra.pgm", std::ios::binary);
        xg << "P5\n" << nvox.y() << ' ' << nvox.x() << "\n255\n";
        for (int x = nvox.x() - 1; x >= 0; --x)
            for (int y = 0; y < nvox.y(); ++y)
                xg.put(static_cast<char>(
                    extra_union[static_cast<size_t>(x) * static_cast<size_t>(nvox.y()) + static_cast<size_t>(y)] != 0
                        ? 0
                        : 255
                ));
    }
    // 建图模式：保存三态图与伴随 .terrain.bin（裁剪到已探索区域）。
    if (!nav) {
        const auto saved = map.save_explored_map(prefix + "_saved");
        if (!saved) {
            std::cerr << "save_explored_map failed: " << saved.error() << '\n';
            return 4;
        }
        std::printf(
            "saved=%s occ=%d free=%d unknown=%d terrain_prior=%d\n",
            saved->pgm_path.c_str(),
            saved->occupied_count,
            saved->free_count,
            saved->unknown_count,
            static_cast<int>(saved->saved_terrain_prior)
        );
    }
    std::sort(ms_all.begin(), ms_all.end());
    const double med = ms_all.empty() ? 0 : ms_all[ms_all.size() / 2];
    const double p95 = ms_all.empty() ? 0 : ms_all[ms_all.size() * 95 / 100];
    std::printf(
        "frames=%zu obs_final=%d traversed_cells=%d traversed_obs=%d update_ms_med=%.1f p95=%.1f\n",
        frames.size(),
        obs_total,
        trav,
        trav_obs,
        med,
        p95
    );
    return 0;
}
