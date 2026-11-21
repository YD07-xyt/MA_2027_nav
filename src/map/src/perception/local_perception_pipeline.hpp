#pragma once

#include "map/3d_occ_map/config.hpp"
#include "map/3d_occ_map/rog_map.h"
#include "map/layered/layered_map.hpp"
#include "map/layered/map_layers.hpp"
#include "map/perception/dynamic_stvl_layer.hpp"
#include "map/perception/ground_layer.hpp"
#include "map/perception/local_terrain_frame.hpp"
#include "map/perception/online_terrain_mapper.hpp"
#include "map/perception/sensor_model.hpp"
#include "map/perception/terrain_analysis.hpp"
#include "utils/clock.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace ma_map {

// Layer 2：局部 3D 感知管线，支持通过 YAML（rog_map/perception/backend）在双后端间切换：
// - "stvl_prior"（默认）：6-DoF SensorModel + 零先验 OnlineTerrainMapper（建图）+ DynamicStvlLayer（导航）；
// - "legacy_rog"（回退）：既有 ROGMap + GroundLayer + TerrainAnalyzer。
// 仅在 3d_occ_map 内部编译，通过 LocalTerrainFrame 向 Layer 3（LayeredMap）单向输出。
// 线程契约：
// - updateOdom / updateCloud / processPendingFrame 通常由同一 map callback group 串行调用；
// - cloud_ / pc_pose_ 由 input_mtx_ 保护；
// - buildGlobalStaticMap / hasOnlineMappingData 内部线程安全，支持保存服务跨线程调用。
class LocalPerceptionPipeline {
public:
    explicit LocalPerceptionPipeline(const rog_map::Config& cfg);

    auto setMode(MapMode mode) -> void;
    auto resetDynamic() -> void {
        dynamic_stvl_.reset();
    }
    auto setForgetThresholdForMode(bool mapping_mode) -> void;
    auto setClock(utils::Clock clock) -> void;
    auto updateOdom(const rog_map::Pose& pose, double stamp_sec = std::numeric_limits<double>::quiet_NaN()) -> void;
    auto
    updateCloud(const rog_map::PointCloud& cloud, double stamp_sec = std::numeric_limits<double>::quiet_NaN()) -> void;

    // 若存在待处理点云帧，则按当前后端与模式执行 3D 感知处理，输出本帧 LocalTerrainFrame；
    // 无新帧时返回 std::nullopt。
    auto processPendingFrame(const StaticNavContext& static_ctx) -> std::optional<LocalTerrainFrame>;

    auto usesStvlPrior() const -> bool {
        return use_stvl_prior_;
    }

    auto hasOnlineMappingData() const -> bool {
        return online_mapper_.hasData();
    }

    auto buildGlobalStaticMap() const -> std::optional<StaticTerrainBuilder::BuildResult> {
        return online_mapper_.buildGlobalStaticMap();
    }

    auto getLastTerrainMap() const -> utils::vec_E<utils::Vec3f>;
    auto getGroundLayer() const -> const Terrain::GroundLayer& {
        return ground_layer_;
    }
    auto getRogMap() const -> std::shared_ptr<rog_map::ROGMap> {
        return rog_map_ptr_;
    }
    auto getConfig() const -> const rog_map::Config& {
        return cfg_;
    }

private:
    rog_map::Config cfg_;
    bool use_stvl_prior_ = true;
    std::atomic<uint8_t> mode_ {static_cast<uint8_t>(MapMode::MAPPING)};

    // 时间源：默认 steady_clock；use_sim_time 时由 ROS 适配层注入 /clock 原子时间源。
    utils::Clock clock_ = utils::Clock::steady();
    // 里程计新鲜度判断的时钟域安全状态：事件时间只与事件时间比较，接收时间只与本地时钟比较。
    double odom_stamp_sec_ = 0.0;
    bool odom_stamp_is_event_ = false;
    double odom_rx_sec_ = 0.0;

    // 既有 ROGMap / GroundLayer / TerrainAnalyzer（legacy_rog 后端使用；stvl_prior 下仍安全构造以兼容旧查询接口）
    std::shared_ptr<rog_map::ROGMap> rog_map_ptr_;
    Terrain::TerrainAnalyzer terrain_analyzer_;
    Terrain::GroundLayer ground_layer_;

    // 新 stvl_prior 感知后端组件
    SensorModel sensor_model_;
    OnlineTerrainMapper online_mapper_;
    DynamicStvlLayer dynamic_stvl_;

    mutable std::mutex input_mtx_;
    rog_map::PointCloud cloud_;
    rog_map::Pose pc_pose_;
    // 与 cloud_ 同锁保护：本帧事件时间（无有效 header.stamp 时为接收时刻）。
    double pending_cloud_stamp_sec_ = 0.0;
    int unfinished_frame_cnt_ = 0;
    bool map_empty_ = true;

    // 仅由 processPendingFrame 单写者使用的实例级复用缓冲
    rog_map::PointCloud scratch_pc_;
    rog_map::Pose scratch_pose_;
    utils::vec_E<utils::Vec3f> scratch_real_occ_;
    std::vector<Eigen::Vector3d> scratch_pts_;
    double last_empty_warn_t_ = 0.0;

    mutable std::mutex terrain_mtx_;
    utils::vec_E<utils::Vec3f> last_terrain_map_;
};

} // namespace ma_map
