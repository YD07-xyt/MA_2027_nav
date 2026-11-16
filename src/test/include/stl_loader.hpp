// STL 读取与体素化（测试程序用，不依赖 ROS/PCL）。
//  - load_stl：支持二进制和 ASCII STL（文件长度等于 84 + 50 * N 时按二进制读）。
//  - voxelize：在每个三角面上按重心坐标网格均匀采点（相邻采样间距 <= step），
//    落入的体素记为表面体素，并按三角面法向记录朝上/朝下标志。
//  - fill_solid：在原始三角网格上求每列中心竖直射线的交点，按有向绕数填充实体内部。
//    先处理几何交点，再量化为体素，避免薄面的上下表面合并后丢失出入顺序。
//    交点不闭合或方向异常的列退回只保留原采样表面，不推测内部。
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

inline auto fill_solid(const StlMesh& mesh, VoxelGrid& g, VoxelizeStat& stat) -> void {
    struct Crossing {
        double z;
        int delta; // 朝下进入 +1，朝上离开 -1；重叠实体以绕数并集处理
    };
    std::vector<std::vector<Crossing>> columns(static_cast<size_t>(g.dim.x()) * g.dim.y());
    for (size_t t = 0; t < mesh.triangle_num(); ++t) {
        const Eigen::Vector3d& a = mesh.v[3 * t];
        Eigen::Vector3d b = mesh.v[3 * t + 1], c = mesh.v[3 * t + 2];
        const Eigen::Vector3d normal = (b - a).cross(c - a);
        if (std::abs(normal.z()) <= 1e-12 * normal.norm()) continue; // 竖直面不穿越竖直射线
        const int delta = normal.z() < 0.0 ? 1 : -1;
        if (normal.z() < 0.0) std::swap(b, c); // XY 投影统一为逆时针，原法向仍由 delta 保留
        const Eigen::Vector3i lo = (voxel_index(a.cwiseMin(b).cwiseMin(c), g.res) - g.min_id).cwiseMax(0);
        const Eigen::Vector3i hi = (voxel_index(a.cwiseMax(b).cwiseMax(c), g.res) - g.min_id).cwiseMin(g.dim - Eigen::Vector3i::Ones());
        for (int x = lo.x(); x <= hi.x(); ++x) {
            for (int y = lo.y(); y <= hi.y(); ++y) {
                const double px = (x + g.min_id.x() + 0.5) * g.res;
                const double py = (y + g.min_id.y() + 0.5) * g.res;
                // 半开边规则：共享三角边只归一侧，避免对角线交点被计数两次。
                auto owns_edge = [px, py](const Eigen::Vector3d& u, const Eigen::Vector3d& v) {
                    const double dx = v.x() - u.x(), dy = v.y() - u.y();
                    const double edge = dx * (py - u.y()) - dy * (px - u.x());
                    return edge > 0.0 || (edge == 0.0 && (dy > 0.0 || (dy == 0.0 && dx < 0.0)));
                };
                if (!owns_edge(a, b) || !owns_edge(b, c) || !owns_edge(c, a)) continue;
                const double z = a.z() - (normal.x() * (px - a.x()) + normal.y() * (py - a.y())) / normal.z();
                columns[static_cast<size_t>(x) * g.dim.y() + y].push_back({z, delta});
            }
        }
    }
    stat.open_column_num = 0;
    for (int x = 0; x < g.dim.x(); ++x) {
        for (int y = 0; y < g.dim.y(); ++y) {
            auto& hits = columns[static_cast<size_t>(x) * g.dim.y() + y];
            std::sort(hits.begin(), hits.end(), [](const Crossing& a, const Crossing& b) { return a.z < b.z; });
            int winding = 0;
            bool invalid = false;
            // 同高交点合并后再更新绕数，避免相接实体的处理依赖排序顺序。
            for (size_t i = 0; i < hits.size();) {
                size_t j = i + 1;
                int delta = hits[i].delta;
                while (j < hits.size() && std::abs(hits[j].z - hits[i].z) <= 1e-9) delta += hits[j++].delta;
                winding += delta;
                invalid = invalid || winding < 0;
                i = j;
            }
            invalid = invalid || winding != 0;
            if (invalid) ++stat.open_column_num;
            winding = 0;
            size_t i = 0;
            for (int z = 0; z < g.dim.z(); ++z) {
                uint8_t& flag = g.flags[g.addr(x, y, z)];
                flag &= ~VoxelGrid::kInside; // 支持重复调用，始终保留原采样表面
                if (invalid) continue;
                const double center_z = (z + g.min_id.z() + 0.5) * g.res;
                while (i < hits.size() && hits[i].z <= center_z) winding += hits[i++].delta;
                if (winding > 0 && !(flag & VoxelGrid::kSurface)) flag |= VoxelGrid::kInside;
            }
        }
    }
    stat.inside_num = g.count(VoxelGrid::kInside);
}

} // namespace test_utils
