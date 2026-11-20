#pragma once

#include "map/perception/local_terrain_frame.hpp"
#include "map/perception/sensor_model.hpp"
#include "map/perception/static_terrain_builder.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

namespace ma_map {

// 建图模式专用：零先验（不依赖任何静态图或地形先验）稀疏分块 3D 静态建图器。
// 坐标系：严格对齐输入点云世界系（/lio/cloud_world，单位 m）；导出 2D 图原点与体素中心逐格对齐，无半格偏移。
// 线程契约：
// - integrateFrame / reset 由建图线程调用；
// - buildGlobalStaticMap / hasData / stats 内部使用短锁拷贝稀疏列后在锁外运行地形提取，支持服务线程并发保存。
class OnlineTerrainMapper {
public:
    struct Config {
        double resolution = 0.05; // 3D 体素与导出 2D 栅格分辨率 (m)
        double trust_range = 4.0; // 允许写入静态占据命中的最大水平距离 (m)，抑制远距稀疏噪点
        double clear_range = 8.0; // 允许发射 3D 射线扫空动态鬼影的最大距离 (m)
        double clear_stop_margin = 0.15; // 射线在端点前提前停止的距离 (m)，防侵蚀静态墙面
        double min_z = -1.0; // 参与建图的最小世界系 Z (m)
        double max_z = 2.5; // 参与建图的最大世界系 Z (m)
        int static_min_hits = 2; // 判定体素为静态占据的最少跨帧命中数
        float static_clear_ratio = 2.0f; // clears > static_clear_ratio * hits 视为动态过客并剔除
        int ray_clear_min_clears = 2; // 判定体素为射线扫空净空的最少跨帧穿透次数
        int max_clear_rays_per_frame = 4096; // 单帧最大去重扫空射线数（控制每帧耗时上限）
        double preview_radius = 4.5; // 在线局部 2D 预览提取半径 (m)
        double footprint_radius = 0.20; // 机器人底盘脚印地面记录半径 (m)
        double footprint_ground_offset = 0.0; // base_odom Z 相对真实地面的高度偏置 (m)
        double tilt_jump_deg = 5.0; // 帧间机身倾角跳变阈值 (deg)，超过触发暂停写入
        int tilt_hold_frames = 3; // 倾角跳变后的暂停帧数
        size_t max_chunks = 4096; // 16^3 稀疏块容量上限（4096 块 ≈ 256 MB，超限停止分配新块并告警一次）
        StaticTerrainBuilder::Config terrain; // 3D -> 2.5D 静态地形提取参数
    };

    struct Stats {
        size_t chunk_count = 0;
        size_t footprint_cells = 0;
        uint32_t frame_count = 0;
        bool chunk_limit_reached = false;
    };

    OnlineTerrainMapper();
    explicit OnlineTerrainMapper(const Config& cfg);

    auto setConfig(const Config& cfg) -> void;
    auto config() const -> const Config& {
        return cfg_;
    }

    auto reset() -> void;

    // 融入一帧世界系点云与 6-DoF 位姿：
    // - 仅对水平距离 <= trust_range 的点写入命中（每体素每帧最多计 1 次命中，记录连续 [z_min, z_max]）；
    // - 对距离 <= clear_range 的有效回波执行 3D DDA 扫空（仅遍历已分配 chunk，不分配新内存）；
    // - 当 build_local_preview=true 时，同步提取车周 preview_radius 内的 LocalTerrainFrame 用于实时 2D 预览。
    auto integrateFrame(
        std::span<const Eigen::Vector3d> pts_world,
        const SensorPose& pose,
        const Eigen::Quaterniond& q_base_w,
        const SensorModel& sensor,
        bool build_local_preview = true,
        std::vector<Eigen::Vector3f>* preview_cloud = nullptr
    ) -> std::optional<LocalTerrainFrame>;

    // 全量扫描所有已累积 3D 稀疏块与脚印地面，统一通过 StaticTerrainBuilder 构建全局三态 2D 图 + 2.5D 地形先验。
    // 无任何有效静态数据时返回 std::nullopt。
    auto buildGlobalStaticMap() const -> std::optional<StaticTerrainBuilder::BuildResult>;

    auto hasData() const -> bool;
    auto stats() const -> Stats;

private:
    static constexpr int32_t kChunkShift = 4;
    static constexpr int32_t kChunkSize = 16;
    static constexpr int32_t kChunkMask = kChunkSize - 1;
    static constexpr size_t kChunkVolume = 16U * 16U * 16U;

    struct ChunkKey {
        int32_t cx = 0;
        int32_t cy = 0;
        int32_t cz = 0;

        auto operator==(const ChunkKey& o) const -> bool = default;
    };

    struct ChunkKeyHash {
        auto operator()(const ChunkKey& k) const noexcept -> size_t {
            uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(k.cx)) * 73856093ULL;
            h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.cy)) * 19349663ULL;
            h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.cz)) * 83492791ULL;
            h ^= (h >> 33U);
            h *= 0xff51afd7ed558ccdULL;
            h ^= (h >> 33U);
            return static_cast<size_t>(h);
        }
    };

    struct VoxelCell {
        uint16_t hits = 0;
        uint16_t clears = 0;
        uint32_t last_hit_frame = 0;
        uint32_t last_clear_frame = 0;
        int16_t z_min_mm = 32767;
        int16_t z_max_mm = -32768;
    };
    static_assert(sizeof(VoxelCell) == 16, "VoxelCell must stay compact (16 bytes)");

    struct Chunk {
        VoxelCell cells[kChunkVolume];
    };

    auto getOrCreateChunkLocked(const ChunkKey& key) -> Chunk*;
    auto findChunkLocked(const ChunkKey& key) const -> const Chunk*;
    auto findChunkMutableLocked(const ChunkKey& key) -> Chunk*;

    auto castClearRayLocked(
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& target,
        uint32_t frame_seq
    ) -> void;

    auto extractColumnGridLocked(int32_t min_vx, int32_t max_vx, int32_t min_vy, int32_t max_vy) const
        -> StaticTerrainBuilder::VoxelColumnGrid;

    Config cfg_;
    double inv_res_ = 20.0;
    StaticTerrainBuilder builder_;

    mutable std::mutex mtx_;
    std::unordered_map<ChunkKey, std::unique_ptr<Chunk>, ChunkKeyHash> chunks_;
    std::unordered_map<uint64_t, float> footprint_ground_;
    uint32_t frame_seq_ = 0;
    bool has_prev_pose_ = false;
    Eigen::Quaterniond prev_q_base_w_ = Eigen::Quaterniond::Identity();
    int tilt_hold_left_ = 0;
    bool chunk_limit_warned_ = false;
};

} // namespace ma_map
