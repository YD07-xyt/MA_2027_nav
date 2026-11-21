#pragma once

#include "map/layered/layered_map.hpp"
#include "map/perception/local_terrain_frame.hpp"
#include "map/perception/sensor_model.hpp"
#include "utils/clock.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

namespace ma_map {

// 导航模式专用：静态先验背景差分 + 极坐标深度图射线清除 + STVL 时空体素衰减动态障碍层。
// 核心职责：
// - 利用纯静态 ESDF（StaticNavContext::static_esdf）在墙体/高台边沿 wall_suppress_dist 范围内抑制静物残影；
// - 利用 2.5D 静态先验（.terrain.bin）自动跟踪 LIO 垂直漂移 delta_z，精确剥离地面/斜坡/高台顶面与已知限高杆/洞顶；
// - 利用与雷达安装角无关的全向极坐标深度图（360x180）实现 O(N_pts + N_active) 射线清除与分级衰减，彻底杜绝拖影。
// 线程契约：由 LocalPerceptionPipeline 在 map callback 线程串行调用。
class DynamicStvlLayer {
public:
    struct Config {
        double voxel_size = 0.05; // 动态 3D 体素边长 (m)
        double nav_max_range = 5.0; // 动态障碍检测最大水平半径 (m)
        double wall_suppress_dist = 0.08; // 距静态障碍 ESDF <= 此值抑制为静态墙体/边沿抖动 (m)
        float ground_margin_base = 0.06f; // 贴地噪声剥离基础余量 (m)
        float ground_margin_per_m = 0.012f; // 距离自适应贴地余量系数 (m/m)
        float dynamic_max_height = 0.85f; // 相对地面动态障碍检测高度上限 (m)
        float overhang_margin = 0.06f; // 低于静态悬空下沿此余量以上视为静态悬空结构 (m)
        float tunnel_clearance_z = 0.22f; // 无 .terrain.bin 时，TUNNEL 语义区高于此高度的回波视为洞顶 (m)
        double fallback_ground_offset = 0.0; // 无 .terrain.bin 时，base_odom Z 相对地面偏置 (m)
        float ray_clear_margin = 0.15f; // 极坐标深度图射线穿透判定余量 (m)
        int miss_clear_frames = 2; // 视场内连续射线穿透此帧数即刻抹除动态体素
        float decay_in_fov_sec = 0.35f; // 视场内未命中衰减时间 (s)
        float decay_blind_sec = 0.45f; // 盲区/身后衰减时间 (s)
        int confirm_min_frames = 2; // 独立动态体素确认所需跨帧命中数
        bool fast_confirm_with_neighbor = true; // 单帧成簇（>=2 点且 >=2 个 3D 邻体素同帧命中）立即确认
        double tilt_jump_deg = 5.0; // 机身倾角跳变门控阈值 (deg)
        int tilt_hold_frames = 3; // 倾角跳变暂停写入/清除的帧数
        float drift_max_abs = 0.35f; // 在线 Z 漂移补偿限幅 (m)
        int drift_min_samples = 30; // 更新 Z 漂移估计的最少有效地面点数
        float dt_min_sec = 0.005f; // 帧间时间步长下限 (s)
        float dt_max_sec = 0.50f; // 帧间时间步长上限 (s)
        float nominal_dt_sec = 0.10f; // 首帧或时间戳无效时的标称帧间隔 (s)
        size_t max_active_voxels = 16384; // 动态活跃体素数上限（防极端噪点导致内存无界增长）
    };

    DynamicStvlLayer();
    explicit DynamicStvlLayer(const Config& cfg);

    auto setConfig(const Config& cfg) -> void;
    auto setClock(utils::Clock clock) -> void {
        clock_ = clock;
    }
    auto config() const -> const Config& {
        return cfg_;
    }

    auto reset() -> void;

    // 处理一帧导航模式世界系点云：
    // - now_sec >= 0 时使用调用方提供的仿真/回放时间 (s)；now_sec < 0 时自动读取注入的 clock_。
    // - 输出仅包含本帧已确认动态障碍的 LocalTerrainFrame（供 DynamicObstacleLayer 整帧覆盖）。
    // - debug_cloud != nullptr 时输出已确认的 3D 动态障碍体素中心；
    // - filtered_cloud != nullptr 时输出本帧通过前端过滤（finite/自滤/min_range/max_range）的 3D 点云。
    auto update(
        std::span<const Eigen::Vector3d> pts_world,
        const SensorPose& pose,
        const Eigen::Quaterniond& q_base_w,
        const SensorModel& sensor,
        const StaticNavContext& static_ctx,
        double now_sec = -1.0,
        std::vector<Eigen::Vector3f>* debug_cloud = nullptr,
        std::vector<Eigen::Vector3f>* filtered_cloud = nullptr
    ) -> LocalTerrainFrame;

    auto estimatedDriftZ() const -> float {
        return delta_z_;
    }

    auto activeVoxelCount() const -> size_t {
        return active_voxels_.size();
    }

private:
    static constexpr int kAzBins = 360;
    static constexpr int kElBins = 180;

    struct VoxelKey {
        int32_t vx = 0;
        int32_t vy = 0;
        int32_t vz = 0;

        auto operator==(const VoxelKey& o) const -> bool = default;
    };

    struct VoxelKeyHash {
        auto operator()(const VoxelKey& k) const noexcept -> size_t {
            uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(k.vx)) * 73856093ULL;
            h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.vy)) * 19349663ULL;
            h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.vz)) * 83492791ULL;
            h ^= (h >> 33U);
            h *= 0xff51afd7ed558ccdULL;
            h ^= (h >> 33U);
            return static_cast<size_t>(h);
        }
    };

    struct DynamicVoxel {
        uint32_t last_hit_frame = 0;
        uint16_t hit_frames = 0;
        uint16_t pts_in_frame = 0;
        uint16_t miss_in_fov = 0;
        float age_sec = 0.0f;
    };

    auto computeEffectiveDt(double now_sec) -> float;
    auto updateDriftEstimate(
        std::span<const Eigen::Vector3d> pts_world,
        const SensorPose& pose,
        const SensorModel& sensor,
        const StaticNavContext& static_ctx
    ) -> void;
    auto estimateRobustDrift(std::vector<float>& samples, float& est) -> bool;
    auto isRayClearedInRangeImage(int az_idx, int el_idx, float voxel_range) const -> bool;

    Config cfg_;
    double inv_voxel_size_ = 20.0;
    // 默认 steady_clock；仿真/回放由上层注入 /clock 或手动时钟。
    utils::Clock clock_ = utils::Clock::steady();

    std::unordered_map<VoxelKey, DynamicVoxel, VoxelKeyHash> active_voxels_;
    std::vector<float> range_img_; // kAzBins * kElBins，0.0f 表示该角格无回波，>0 表示该角格最小测距 (m)
    std::vector<float> drift_samples_;
    std::vector<float> drift_wide_samples_;
    std::vector<float> drift_inliers_;
    std::vector<VoxelKey> erase_scratch_;
    std::vector<VoxelKey> confirmed_voxels_scratch_;
    std::vector<uint64_t> emitted_xy_scratch_;

    uint32_t frame_seq_ = 0;
    float delta_z_ = 0.0f;
    bool has_delta_z_ = false;
    int drift_miss_frames_ = 0;
    const StaticTerrainPrior* last_prior_ptr_ = nullptr;

    bool has_prev_pose_ = false;
    Eigen::Quaterniond prev_q_base_w_ = Eigen::Quaterniond::Identity();
    int tilt_hold_left_ = 0;

    bool has_last_time_ = false;
    bool last_time_was_explicit_ = false;
    double last_explicit_sec_ = 0.0;
    double last_wall_sec_ = 0.0;
};

} // namespace ma_map
