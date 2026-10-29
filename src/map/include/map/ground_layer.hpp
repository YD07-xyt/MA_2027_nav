#pragma once

#include "map/3d_occ_map/config.hpp"
#include "utils/eigen_alias.hpp"
#include "utils/type_utils.hpp"
#include <climits>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace Terrain {

// 持久 2.5D 地面层：每个 2D 格记一个地面高度估计，跨帧融合，不随 ROG 射线擦除而消失。
// - 输入是原始世界系点云（不是 ROG 体素），每帧每格取最低点作量测；
// - 方差加权融合；比当前估计低 → 连续 drop_votes 帧一致才下降（易降），
//   高 → 连续 rise_votes 帧一致且不超过 rise_max 才上升（难升），高于 rise_max 的当悬空物忽略；
// - 最低点正下方已被射线证实为空闲（杆下沿）→ 不当地面；
// - 射线上界 ub：下行射线经过的格，地面不可能高于射线；ub 明显低于已记地面且该格久未命中 → 撤销该格地面；
// - 足迹格每帧标记"走过"，无观测时用机体位姿给低置信种子；首帧给车周 seed_radius 的盲区种子。
// 存储为以全局格坐标取模的环形缓冲（size×size 米），格内存全局坐标，坐标不符即视为空。
class GroundLayer {
public:
    struct Config {
        float resolution = 0.05f;
        float size = 20.0f;              // 环形缓冲边长 (m)
        float min_range = 0.5f;          // 距雷达水平距离小于此值的点丢弃（车体自身点），同 ray_range[0]
        float max_range = 7.5f;          // 只用距雷达水平距离以内的点 (m)
        float meas_sigma = 0.01f;        // 量测噪声 σ = meas_sigma + meas_sigma_per_m * 距离
        float meas_sigma_per_m = 0.003f;
        float gate_min = 0.03f;          // 融合门限下限 (m)
        int   drop_votes = 2;            // 下降需连续几帧一致
        int   rise_votes = 3;            // 上升需连续几帧一致
        float rise_max = 0.24f;          // 高于当前地面超过此值的量测当悬空物，不参与上升
        float footprint_radius = 0.20f;  // 足迹半径 (m)
        float footprint_ground_offset = 0.0f;  // 足迹种子高度 = base 平面 + 此偏移
        float seed_radius = 1.5f;        // 首帧盲区种子半径 (m)
        float seed_sigma = 0.05f;        // 种子 σ（低置信，任何真实量测直接覆盖）
        int   float_free_voxels = 3;     // 最低点正下方连续多少个已知空闲体素即判悬空
        float ub_end_margin = 0.3f;      // 射线末端多少米内不算上界（掠射噪声）
        float ub_margin = 0.05f;         // ub 低于地面多少才算矛盾
        int   ub_stale_frames = 5;       // 地面最近这么多帧内被命中过则不因 ub 撤销
    };

    enum Flag : uint8_t {
        OBS = 1,   // 有真实量测的地面
        SEED = 2,  // 种子地面（足迹/盲区），无真实量测
        TRAV = 4,  // 车走过
    };

    struct Cell {
        int kx = INT_MIN, ky = INT_MIN;  // 全局格坐标
        float h = 0.0f;                  // 地面高度
        float var = 0.0f;                // 方差
        float ub = std::numeric_limits<float>::infinity();  // 射线上界
        float vote_h = 0.0f;             // 待确认的升/降高度
        uint8_t flags = 0;
        int8_t down = 0, up = 0;         // 升/降投票数
        int last_hit = -1000;            // 最近一次量测的帧号
    };

    using VoxelQuery = std::function<utils::GridType(const utils::Vec3f&)>;

    GroundLayer() = default;
    void setConfig(const Config& cfg);

    // cloud：世界系点云；base_pose：机体位姿；sensor：射线起点（雷达位置，世界系）
    void update(
        const rog_map::PointCloud& cloud,
        const utils::Pose& base_pose,
        const utils::Vec3f& sensor,
        const VoxelQuery& voxel_query
    );

    // 全局格坐标 (floor(x/res), floor(y/res)) 处的格；未存储返回 nullptr
    auto cell(int kx, int ky) const -> const Cell*;

    auto config() const -> const Config& {
        return cfg_;
    }

private:
    auto ringIndex(int kx, int ky) const -> int;
    auto slot(int kx, int ky) -> Cell&;  // 坐标不符时重置为空格
    void seedDisk(const utils::Pose& base_pose, float radius, bool traversed);

    struct Scratch {
        int stamp = -1;
        int kx = 0, ky = 0;
        float m1 = 0.0f, m2 = 0.0f;  // 本帧最低、次低点
        int cnt = 0;
        float range = 0.0f;
    };

    Config cfg_;
    int n_ = 0;  // 环形缓冲每边格数
    int frame_ = 0;
    bool seeded_ = false;
    std::vector<Cell> cells_;
    std::vector<Scratch> scratch_;
    std::vector<int> touched_;
};

}  // namespace Terrain
