#pragma once

#include "3d_occ_map/rog_map.h"
#include "map/ground_layer.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace Terrain {

using Map = utils::vec_E<utils::Vec3f>;

class TerrainAnalyzer {
public:
    struct Config {
        int   kernel_size = 5;         // 中值滤波核大小
        float resolution = 0.1f;       // 网格分辨率 (米)
        float map_size_x = 5.0f;       // 分析窗口 x 方向大小 (米)
        float map_size_y = 5.0f;       // 分析窗口 y 方向大小 (米)
        float max_step_height = 0.15f; // 台阶/墙体最小高度差（用于触发间隙分析）
        float robot_height = 0.3f;     // 机器人安全通行所需的最小垂直间隙
        float steep_threshold = 0.17f;  // 建议设为与 max_step_height 相近或稍大，如0.2m
        // ---- 悬空证据（需要 voxel_query）----
        // 间隙 >= robot_height 直接放行；[min_overhang_clearance, robot_height) 之间只有
        // 下沿正下方的净空被射线证实（连续已知空闲一直通到地面）才放行。
        float min_overhang_clearance = 0.225f;
        int   overhang_free_voxels = 3;      // 下沿正下方至少连续多少个已知空闲体素
        int   overhang_unknown_skip = 1;     // 下沿正下方允许跳过的未知体素数（杆底混合体素）
        int   overhang_ground_radius = 6;    // 悬空列找邻域地面的半径（格）
        float obstacle_band = 0.08f;         // 高出参考地面多少以内算"贴地"
        float ground_jump_thresh = 0.10f;    // 本格地面比邻域最低地面高出多少才认为被抬高
        int   ground_min_radius = 2;         // 邻域最低地面的搜索半径（格）
        // ---- 可达生长（阶段 2，需要持久地面层）----
        bool  reach_en = false;
        float max_slope_deg = 25.0f;         // 可通行最大坡度
        float reach_step_tol = 0.03f;        // 相邻格高差在坡度允许值之外的容差 (m)
        float plane_rms_max = 0.02f;         // 局部平面拟合残差小于此值才用平面坡度判陡坡 (m)
        bool  drop_edge_en = false;          // 射线上界证明外侧更低且未观测 → 判下沿
        float drop_thresh = 0.10f;           // 下沿判定：外侧上界低于本格地面多少 (m)
    };

    // 体素状态查询：pos 为体素中心（世界系）。杆下方被射线扫空 → KNOWN_FREE；
    // 台面/台阶顶面下方是实心，射线进不去 → UNKNOWN。
    using VoxelQuery = std::function<utils::GridType(const utils::Vec3f&)>;

    TerrainAnalyzer() = default;
    explicit TerrainAnalyzer(const Config& cfg) : config_(cfg) {}

    void setConfig(const Config& cfg) { config_ = cfg; }
    // 持久地面层（可为空）：非空时列内最低体素缺失或高于持久地面的格，以持久地面为准
    void setGroundLayer(const GroundLayer* ground_layer) { ground_layer_ = ground_layer; }

    // 输入 occ_map：世界坐标系下的 3D 占据点云
    // robot_pose：机器人当前位姿（用于确定窗口中心与地面参考高度）
    // 输出：每个网格中心的点云，z=0 表示可通行，z=1 表示不可通行
    auto analyze(const utils::RobotState& robot_pose,
                 const Map& occ_map) -> Map;

    // 同上，额外输出空闲证据 free_out（网格中心，z=0）：
    // 本帧该格自身有占据点（地面可见）、未判为障碍，且判定明确
    // （平坦，或间隙分析为可通行）。仅因绝对高度阈值未判障碍的格不算空闲，
    // 这类格保持原值，避免把低矮障碍回写成空闲。
    auto analyze(const utils::RobotState& robot_pose,
                 const Map& occ_map, Map& free_out) -> Map;

    // 带 3D 空闲/未知证据的版本；voxel_query 为空时与上面完全一致。
    // 额外的空闲证据：本格没有占据体素、8 邻域本帧无障碍格，且邻域地面（中位数）往上
    // 到 robot_height 的体素已知空闲（最多跳过 overhang_unknown_skip 个未知）。
    // 用于清掉"3D 层已被射线擦空、2D 层还残留"的旧障碍。
    auto analyze(const utils::RobotState& robot_pose,
                 const Map& occ_map, Map& free_out,
                 const VoxelQuery& voxel_query) -> Map;

private:
    Config config_;
    const GroundLayer* ground_layer_ = nullptr;

    // 中值滤波（仅考虑非 NaN 邻居）
    auto medianFilter(const Eigen::ArrayXXf& input, int kernel_size) -> Eigen::ArrayXXf;

    // 网格化工具（以窗口 origin 为基准）
    struct GridInfo {
        int width, height;
        utils::Vec3f origin;   // 窗口左下角世界坐标
    };
    auto setupGrid(const utils::Vec3f& robot_pos) const -> GridInfo;
    auto worldToGrid(const GridInfo& grid, const utils::Vec3f& world) const
        -> std::optional<std::pair<int, int>>;
    auto gridToWorld(const GridInfo& grid, int gx, int gy) const -> utils::Vec3f;
};

} // namespace Terrain