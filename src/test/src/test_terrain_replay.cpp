// 地形分析离线回放：读 terrain_replay_extract.py 导出的帧文件，逐帧喂 MaMap，输出可比较的统计。
// 不依赖 ROS。用法：
//   test_terrain_replay <frames.bin> <map.yaml> <out_prefix> [nav] [realtime]
//     nav      : 导航模式（set_mapping_model(false)），默认建图模式
//     realtime : 按 bag 时间戳节拍 sleep（ROG 遗忘用墙钟时间，测遗忘时需要）
// 输出：
//   <out_prefix>_final.pgm   最终 2D 占据（白=空闲，黑=障碍）
//   <out_prefix>_frames.csv  每帧：t, x, y, z, 障碍格数, 车周 3 m 内障碍格数, 足迹内障碍格数, update_map 耗时 ms
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
        for (int i = 0; i < 3; ++i) fr.p[i] = buf[1 + i];
        for (int i = 0; i < 4; ++i) fr.q[i] = buf[4 + i];
        int32_t np = 0;
        f.read(reinterpret_cast<char*>(&np), 4);
        fr.xyz.resize(3 * np);
        f.read(reinterpret_cast<char*>(fr.xyz.data()), 12 * static_cast<std::streamsize>(np));
    }
    return frames;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: test_terrain_replay <frames.bin> <map.yaml> <out_prefix> [nav] [realtime]\n";
        return 2;
    }
    bool nav = false, realtime = false;
    for (int i = 4; i < argc; ++i) {
        nav |= std::string(argv[i]) == "nav";
        realtime |= std::string(argv[i]) == "realtime";
    }
    const auto frames = read_frames(argv[1]);
    const std::string prefix = argv[3];
    ma_map::MaMap map(argv[2]);
    map.set_mapping_model(!nav);

    const auto gm0 = map.get_grid_map();
    const double res = gm0->getResolution();
    const Eigen::Vector2i nvox = gm0->getVoxelNum();
    // 车走过的格（足迹半径 0.15 m）：最终不应是障碍
    std::vector<char> traversed(nvox.x() * nvox.y(), 0);
    const double foot_r = 0.15;
    const int foot_c = static_cast<int>(std::ceil(foot_r / res));

    std::ofstream csv(prefix + "_frames.csv");
    csv << "t,x,y,z,obs,obs_near3m,obs_footprint,ms\n";
    const auto wall0 = std::chrono::steady_clock::now();
    const double t0 = frames.empty() ? 0.0 : frames.front().t;
    std::vector<double> ms_all;
    for (const auto& fr: frames) {
        if (realtime) {
            std::this_thread::sleep_until(wall0 + std::chrono::duration<double>(fr.t - t0));
        }
        rog_map::Pose pose;
        pose.first = utils::Vec3f(fr.p[0], fr.p[1], fr.p[2]);
        pose.second = utils::Quatf(fr.q[0], fr.q[1], fr.q[2], fr.q[3]);
        map.update_odom(pose);
        rog_map::PointCloud pc;
        pc.resize(fr.xyz.size() / 3);
        for (size_t i = 0; i < pc.size(); ++i) {
            pc[i].x = fr.xyz[3 * i];
            pc[i].y = fr.xyz[3 * i + 1];
            pc[i].z = fr.xyz[3 * i + 2];
        }
        map.update_cloud(pc);
        const auto c0 = std::chrono::steady_clock::now();
        map.update_map();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
        ms_all.push_back(ms);

        const auto occ = map.get_occupancy_2d();
        Eigen::Vector2i rid;
        gm0->posToIndex(Eigen::Vector2d(fr.p[0], fr.p[1]), rid);
        int obs = 0, near = 0, foot = 0;
        const int near_c = static_cast<int>(3.0 / res);
        for (int x = 0; x < occ.rows(); ++x) {
            for (int y = 0; y < occ.cols(); ++y) {
                if (occ(x, y) != 1) continue;
                ++obs;
                const int dx = x - rid.x(), dy = y - rid.y();
                if (dx * dx + dy * dy <= near_c * near_c) ++near;
                if (dx * dx + dy * dy <= foot_c * foot_c) ++foot;
            }
        }
        for (int dx = -foot_c; dx <= foot_c; ++dx)
            for (int dy = -foot_c; dy <= foot_c; ++dy) {
                const int x = rid.x() + dx, y = rid.y() + dy;
                if (dx * dx + dy * dy > foot_c * foot_c) continue;
                if (x < 0 || y < 0 || x >= nvox.x() || y >= nvox.y()) continue;
                traversed[x * nvox.y() + y] = 1;
            }
        csv << fr.t - t0 << ',' << fr.p[0] << ',' << fr.p[1] << ',' << fr.p[2] << ',' << obs << ',' << near << ','
            << foot << ',' << ms << '\n';
    }

    const auto occ = map.get_occupancy_2d();
    int trav = 0, trav_obs = 0, obs_total = 0;
    for (int x = 0; x < occ.rows(); ++x)
        for (int y = 0; y < occ.cols(); ++y) {
            obs_total += occ(x, y) == 1;
            if (!traversed[x * nvox.y() + y]) continue;
            ++trav;
            trav_obs += occ(x, y) == 1;
        }
    // PGM：行 = y（从上到下 y 递减），列 = x
    std::ofstream pgm(prefix + "_final.pgm", std::ios::binary);
    pgm << "P5\n" << occ.rows() << ' ' << occ.cols() << "\n255\n";
    for (int y = occ.cols() - 1; y >= 0; --y)
        for (int x = 0; x < occ.rows(); ++x) {
            const unsigned char v = occ(x, y) == 1 ? 0 : (traversed[x * nvox.y() + y] ? 160 : 255);
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
                gm0->indexToPos(Eigen::Vector2i(x, y), p);
                const auto* c = gl.cell(static_cast<int>(std::floor(p.x() / gres)), static_cast<int>(std::floor(p.y() / gres)));
                if (c == nullptr || c->flags == 0) continue;
                g << p.x() << ',' << p.y() << ',' << c->h << ',' << int(c->flags) << ',' << c->ub << '\n';
            }
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
