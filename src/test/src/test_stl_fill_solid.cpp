// 独立回归：只覆盖 STL 测试输入构造，不链接/调用 map 算法。
// 本文件不自动加入比赛构建；获得构建许可后可单独编译运行。
#include "stl_loader.hpp"

#include <iostream>

namespace {
using test_utils::StlMesh;

auto require(bool value, const char* message) -> void {
    if (!value) throw std::runtime_error(message);
}

auto box(StlMesh& mesh, double bottom, double top) -> void {
    const Eigen::Vector3d v[] = {{0, 0, bottom}, {0.2, 0, bottom}, {0.2, 0.2, bottom}, {0, 0.2, bottom},
                                 {0, 0, top}, {0.2, 0, top}, {0.2, 0.2, top}, {0, 0.2, top}};
    const int faces[][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                            {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
                            {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    for (const auto& face: faces)
        for (int k: face) {
            mesh.v.push_back(v[k]);
            mesh.min = mesh.min.cwiseMin(v[k]);
            mesh.max = mesh.max.cwiseMax(v[k]);
        }
}

auto run(const StlMesh& mesh) -> test_utils::VoxelGrid {
    test_utils::VoxelizeStat stat;
    auto g = test_utils::voxelize(mesh, 0.05, 0.0125, stat);
    const auto surface = g.flags;
    test_utils::fill_solid(mesh, g, stat);
    require(stat.open_column_num == 0, "closed boxes unexpectedly rejected");
    for (size_t i = 0; i < surface.size(); ++i)
        require((surface[i] & 7) == (g.flags[i] & 7), "surface sampling changed");
    const auto once = g.flags;
    test_utils::fill_solid(mesh, g, stat);
    require(once == g.flags, "fill is not idempotent");
    return g;
}

auto occupied(const test_utils::VoxelGrid& g, int z) -> bool {
    // 中心 (0.075, 0.075) 恰好在水平面的三角剖分对角线上。
    return g.occupied(1 - g.min_id.x(), 1 - g.min_id.y(), z - g.min_id.z());
}
} // namespace

int main(int argc, char** argv) {
    StlMesh mesh;
    box(mesh, 0.0, 0.12);
    box(mesh, 0.141, 0.142); // 薄层与下方实体顶面量化到同一体素
    box(mesh, 0.395, 0.565);
    auto g = run(mesh);
    require(occupied(g, 1), "floor interior missing");
    for (int z = 3; z <= 6; ++z) require(!occupied(g, z), "overhead gap incorrectly filled");
    require(occupied(g, 9), "beam interior missing");

    StlMesh overlap;
    box(overlap, 0.0, 0.22);
    box(overlap, 0.12, 0.42);
    g = run(overlap);
    require(occupied(g, 6), "overlap union incorrectly treated as free");

    StlMesh touching;
    box(touching, 0.0, 0.20);
    box(touching, 0.20, 0.42);
    g = run(touching);
    require(occupied(g, 6), "touching solids incorrectly treated as free");

    test_utils::VoxelizeStat stat;
    auto open = mesh;
    open.v.erase(open.v.begin() + 6, open.v.begin() + 12); // 去掉最下方箱体的顶面
    g = test_utils::voxelize(open, 0.05, 0.0125, stat);
    test_utils::fill_solid(open, g, stat);
    require(stat.open_column_num > 0, "open columns not detected");
    require(!occupied(g, 3), "open column must retain surface only");
    const auto surface = g.flags;
    for (auto& flag: g.flags) flag |= test_utils::VoxelGrid::kInside;
    test_utils::fill_solid(open, g, stat);
    require(g.flags == surface, "invalid columns retain stale inside voxels");

    // 可选真实模型回归：只构造四个已定位柱，原表面标志来自原采样结果。
    if (argc > 1) {
        auto actual = test_utils::load_stl(argv[1]);
        actual.translate(Eigen::Vector3d(0, 0, -actual.min.z()));
        const double xy[][2] = {{3.2, 10.2}, {3.2, 10.25}, {5.05, -1.7}, {5.05, -1.65}};
        for (const auto& p: xy) {
            test_utils::VoxelGrid col;
            col.res = 0.05;
            col.min_id = test_utils::voxel_index(Eigen::Vector3d(p[0], p[1], 0), col.res);
            col.dim = Eigen::Vector3i(1, 1, 76);
            col.flags.assign(76, 0);
            col.flags[0] = 5; col.flags[2] = 7; col.flags[7] = 5; col.flags[11] = 3;
            test_utils::fill_solid(actual, col, stat);
            for (int z = 3; z <= 6; ++z) require(!col.occupied(0, 0, z), "actual beam gap filled");
            require(col.occupied(0, 0, 2) && col.occupied(0, 0, 7), "actual surfaces lost");
        }
    }
    std::cout << "PASS: thin surfaces, overhead gap, shared edges, overlap, touching, open columns, idempotence" << std::endl;
}
