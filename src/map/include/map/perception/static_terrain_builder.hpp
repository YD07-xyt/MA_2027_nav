#pragma once

#include "map/core/grid_geometry.hpp"
#include "map/core/map_io.hpp"
#include "map/core/static_terrain_prior.hpp"

#include <limits>
#include <vector>

namespace ma_map {

// 3D 静态占据体素列 -> 2D 三态可通行地图 + 2.5D 静态地形先验（ground_z + overhang_min_z）统一构建器。
// 纯 C++/Eigen（无 PCL / ROS 依赖），由在线建图（OnlineTerrainMapper）与离线工具（static_map_builder）共同复用。
class StaticTerrainBuilder {
public:
    struct ZSpan {
        float z_lo = 0.0f;
        float z_hi = 0.0f;
    };

    struct Config {
        float robot_height = 0.24f; // 机器人高度 (m)
        float min_overhang_clearance = 0.225f; // 悬空结构下方最小可通行净空 (m)
        float max_ground_height = 1.5f; // 可通行地面世界系 Z 上限 (m)
        float min_ground_height = -1.0f; // 可通行地面世界系 Z 下限 (m)
        float max_slope_deg = 25.0f; // 最大可通行坡度 (deg)
        float step_tol = 0.03f; // 邻格步高容差 (m)：最大允许单步高差 = d_xy * tan(max_slope_deg) + step_tol
        float plane_rms_max = 0.035f; // 5x5 局部平面拟合最大 RMS 残差 (m)
        float max_surface_thickness = 0.11f; // 点云模式单地面段最大竖向跨度 (m)，超过视为竖直墙体/立面
        float gap_fill_distance = 0.06f; // 同柱相邻占据段合并间隙上限 (m)
        float same_surface_dz = 0.08f; // 同面保边 3x3 中值滤波最大邻格高差 (m)
        int overhang_ground_radius = 6; // 悬空列寻找周围参考地面的切比雪夫半径 (格)
        float min_cleared_overhang_span = 0.12f; // 射线扫空确认悬空的最小垂直净空厚度 (m)
        bool despeckle_isolated_occ = true; // 点云模式清理 8 邻域全 FREE 的孤立单格非硬墙障碍噪点
    };

    struct VoxelColumnGrid {
        grid_map::GridGeometry geometry;
        std::vector<std::vector<ZSpan>> occ_spans; // 行主序 ix * ny + iy
        std::vector<std::vector<ZSpan>> cleared_spans; // 可选：射线扫空 Z 区间（行主序，可为空 vector）
        std::vector<float> footprint_ground_z; // 可选：车底脚印已知地面 Z（NaN 表示无，可为空 vector）
        bool is_solid_mesh = false; // true: STL 实体网格；false: 点云表面观测
        bool has_ray_clearing = false; // true: cleared_spans 含有效 3D 射线扫空证据

        static auto create(const grid_map::GridGeometry& geom, bool solid_mesh, bool ray_clearing) -> VoxelColumnGrid {
            VoxelColumnGrid g;
            g.geometry = geom;
            g.is_solid_mesh = solid_mesh;
            g.has_ray_clearing = ray_clearing;
            if (geom.isValid()) {
                const size_t n = geom.cellCount();
                g.occ_spans.resize(n);
                if (ray_clearing) {
                    g.cleared_spans.resize(n);
                }
                g.footprint_ground_z.assign(n, std::numeric_limits<float>::quiet_NaN());
            }
            return g;
        }
    };

    struct BuildResult {
        grid_map::GridGeometry geometry;
        std::vector<CellState> cells; // 行主序 ix * ny + iy
        StaticTerrainPrior terrain_prior; // 与 geometry 逐格对齐的 2.5D 静态先验
        int free_count = 0;
        int occupied_count = 0;
        int unknown_count = 0;
        int overhang_count = 0;
    };

    StaticTerrainBuilder() = default;
    explicit StaticTerrainBuilder(const Config& cfg): cfg_(cfg) {}

    auto setConfig(const Config& cfg) -> void {
        cfg_ = cfg;
    }

    auto config() const -> const Config& {
        return cfg_;
    }

    auto build(const VoxelColumnGrid& grid) const -> BuildResult;

private:
    Config cfg_;
};

} // namespace ma_map
