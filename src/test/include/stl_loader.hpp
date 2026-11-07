// STL 读取与体素化（测试程序用，不依赖 ROS/PCL）。
//  - load_stl：支持二进制和 ASCII STL（文件长度等于 84 + 50 * N 时按二进制读）。
//  - voxelize：在每个三角面上按重心坐标网格均匀采点（相邻采样间距 <= step），
//    落入的体素记为表面体素，并按三角面法向记录朝上/朝下标志。
//  - fill_solid：STL 是空壳，按列做奇偶填充把实体内部填满：
//    自下而上扫描，遇到只含朝下面（实体底面）的体素进入实体，遇到只含朝上面（实体顶面）的体素离开。
//    扫到列顶仍在实体内说明该列网格不闭合，整列退回只保留表面体素。
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace test_utils {

struct StlMesh {
    std::vector<Eigen::Vector3d> v; // 每 3 个顶点一个三角面，顶点顺序决定外法向（右手）
    Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

    auto triangle_num() const -> size_t {
        return v.size() / 3;
    }
    auto translate(const Eigen::Vector3d& t) -> void {
        for (auto& p: v) p += t;
        min += t;
        max += t;
    }
};

inline auto load_stl(const std::string& path) -> StlMesh {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("load_stl: cannot open " + path);
    const std::string data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    StlMesh mesh;
    auto push = [&mesh](const Eigen::Vector3d& p) {
        mesh.v.push_back(p);
        mesh.min = mesh.min.cwiseMin(p);
        mesh.max = mesh.max.cwiseMax(p);
    };

    uint32_t n = 0;
    bool binary = false;
    if (data.size() >= 84) {
        std::memcpy(&n, data.data() + 80, 4);
        binary = data.size() == 84 + 50ull * n;
    }
    if (binary) {
        mesh.v.reserve(3ull * n);
        for (uint32_t i = 0; i < n; ++i) {
            const char* rec = data.data() + 84 + 50ull * i + 12; // 跳过文件内法向量，法向由顶点顺序重新计算
            for (int k = 0; k < 3; ++k) {
                float xyz[3];
                std::memcpy(xyz, rec + 12 * k, 12);
                push(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
            }
        }
    } else {
        std::istringstream ss(data);
        std::string tok;
        while (ss >> tok) {
            if (tok == "vertex") {
                double x = 0, y = 0, z = 0;
                ss >> x >> y >> z;
                push(Eigen::Vector3d(x, y, z));
            }
        }
        if (mesh.v.size() % 3 != 0) throw std::runtime_error("load_stl: bad ASCII STL " + path);
    }
    if (mesh.v.empty()) throw std::runtime_error("load_stl: empty STL " + path);
    return mesh;
}

// 体素索引 floor(p / res)，与 ROG-Map ORIGIN_AT_CORNER 的 posToGlobalIndex 一致
inline auto voxel_index(const Eigen::Vector3d& p, double res) -> Eigen::Vector3i {
    return (p.array() / res).floor().cast<int>();
}

// 包围盒内的稠密体素网格。flags：bit0 表面，bit1 含朝上面，bit2 含朝下面，bit3 实体内部（填充得到）
struct VoxelGrid {
    static constexpr uint8_t kSurface = 1, kUp = 2, kDown = 4, kInside = 8;

    double res = 0.05;
    Eigen::Vector3i min_id = Eigen::Vector3i::Zero(); // 网格 (0,0,0) 对应的全局体素索引
    Eigen::Vector3i dim = Eigen::Vector3i::Zero();
    std::vector<uint8_t> flags;

    auto addr(int x, int y, int z) const -> size_t {
        return (static_cast<size_t>(x) * dim.y() + y) * dim.z() + z;
    }
    auto occupied(int x, int y, int z) const -> bool {
        return flags[addr(x, y, z)] & (kSurface | kInside);
    }
    auto count(uint8_t mask) const -> size_t {
        return std::count_if(flags.begin(), flags.end(), [mask](uint8_t f) { return (f & mask) != 0; });
    }
};

struct VoxelizeStat {
    size_t sample_num = 0;
    size_t surface_num = 0;
    size_t inside_num = 0;
    size_t open_column_num = 0; // 奇偶填充失败（网格不闭合）退回表面的列数
};

inline auto voxelize(const StlMesh& mesh, double res, double step, VoxelizeStat& stat) -> VoxelGrid {
    VoxelGrid g;
    g.res = res;
    g.min_id = voxel_index(mesh.min, res);
    g.dim = voxel_index(mesh.max, res) - g.min_id + Eigen::Vector3i::Ones();
    g.flags.assign(static_cast<size_t>(g.dim.prod()), 0);

    for (size_t t = 0; t < mesh.triangle_num(); ++t) {
        const Eigen::Vector3d& a = mesh.v[3 * t];
        const Eigen::Vector3d e1 = mesh.v[3 * t + 1] - a;
        const Eigen::Vector3d e2 = mesh.v[3 * t + 2] - a;
        const Eigen::Vector3d nrm = e1.cross(e2);
        const double len = nrm.norm();
        // |n_z| > 0.3：坡面也算朝上/朝下；竖直墙面只标表面，不参与奇偶切换
        uint8_t dir = 0;
        if (len > 0.0) {
            const double nz = nrm.z() / len;
            dir = nz > 0.3 ? VoxelGrid::kUp : (nz < -0.3 ? VoxelGrid::kDown : 0);
        }
        // u = i/n, w = j/n, i + j <= n：三个边方向的相邻采样间距都 <= max_edge / n <= step
        const double max_edge = std::max({e1.norm(), e2.norm(), (e2 - e1).norm()});
        const int n = std::max(1, static_cast<int>(std::ceil(max_edge / step)));
        for (int i = 0; i <= n; ++i) {
            for (int j = 0; i + j <= n; ++j) {
                const Eigen::Vector3d p = a + (static_cast<double>(i) / n) * e1 + (static_cast<double>(j) / n) * e2;
                const Eigen::Vector3i id = (voxel_index(p, res) - g.min_id).cwiseMax(0).cwiseMin(g.dim - Eigen::Vector3i::Ones());
                g.flags[g.addr(id.x(), id.y(), id.z())] |= VoxelGrid::kSurface | dir;
                ++stat.sample_num;
            }
        }
    }
    stat.surface_num = g.count(VoxelGrid::kSurface);
    return g;
}

inline auto fill_solid(VoxelGrid& g, VoxelizeStat& stat) -> void {
    for (int x = 0; x < g.dim.x(); ++x) {
        for (int y = 0; y < g.dim.y(); ++y) {
            bool inside = false;
            for (int z = 0; z < g.dim.z(); ++z) {
                uint8_t& f = g.flags[g.addr(x, y, z)];
                if (inside && !(f & VoxelGrid::kSurface)) f |= VoxelGrid::kInside;
                const bool up = f & VoxelGrid::kUp, down = f & VoxelGrid::kDown;
                if (down && !up) inside = true;
                if (up && !down) inside = false;
            }
            if (inside) {
                ++stat.open_column_num;
                for (int z = 0; z < g.dim.z(); ++z) g.flags[g.addr(x, y, z)] &= ~VoxelGrid::kInside;
            }
        }
    }
    stat.inside_num = g.count(VoxelGrid::kInside);
}

} // namespace test_utils
