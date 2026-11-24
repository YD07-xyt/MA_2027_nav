#pragma once

#include "map/3d_occ_map/config.hpp"
#include "map/layered/layered_map.hpp"
#include "map/layered/map_layers.hpp"
#include "map/perception/dynamic_stvl_layer.hpp"
#include "map/perception/local_terrain_frame.hpp"
#include "map/perception/online_terrain_mapper.hpp"
#include "map/perception/sensor_model.hpp"
#include "utils/clock.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>

namespace ma_map {

// Layer 2：局部 3D 感知管线（6-DoF SensorModel + 零先验 OnlineTerrainMapper（建图）+ DynamicStvlLayer（导航））。
// 仅在 3d_occ_map 内部编译，通过 LocalTerrainFrame 向 Layer 3（LayeredMap）单向输出。
// 线程契约：
// - updateOdom / updateCloud / processPendingFrame 通常由同一 map callback group 串行调用；
// - odom_pose_ / odom_rcv_ / cloud_ / pc_pose_ 由 input_mtx_ 保护，getRobotPosition 线程安全；
// - buildGlobalStaticMap / hasOnlineMappingData 内部线程安全，支持保存服务跨线程调用。
class LocalPerceptionPipeline {
public:
    explicit LocalPerceptionPipeline(const rog_map::Config& cfg);

    auto setMode(MapMode mode) -> void;
    auto resetDynamic() -> void;
    auto setClock(utils::Clock clock) -> void;
    auto setVizCapture(bool filtered_cloud, bool dynamic_3d) -> void;
    auto updateOdom(const rog_map::Pose& pose, double stamp_sec = std::numeric_limits<double>::quiet_NaN()) -> void;
    auto
    updateCloud(const rog_map::PointCloud& cloud, double stamp_sec = std::numeric_limits<double>::quiet_NaN()) -> void;

    // 若存在待处理点云帧，则按当前模式执行 3D 感知处理，输出本帧 LocalTerrainFrame；
    // 无新帧时返回 std::nullopt。
    auto processPendingFrame(const StaticNavContext& static_ctx) -> std::optional<LocalTerrainFrame>;

    auto hasOnlineMappingData() const -> bool {
        return online_mapper_.hasData();
    }

    auto buildGlobalStaticMap() const -> std::optional<StaticTerrainBuilder::BuildResult> {
        return online_mapper_.buildGlobalStaticMap();
    }

    auto getLastTerrainMap() const -> utils::vec_E<utils::Vec3f>;
    auto getRobotPosition() const -> std::optional<Eigen::Vector3d>;
    auto filteredCloudRevision() const -> uint64_t {
        return filtered_cloud_rev_.load(std::memory_order_acquire);
    }
    auto copyFilteredCloud(std::vector<Eigen::Vector3f>& out) const -> uint64_t;
    auto dynamic3dRevision() const -> uint64_t {
        return dynamic_3d_rev_.load(std::memory_order_acquire);
    }
    auto copyDynamic3dCloud(std::vector<Eigen::Vector3f>& out) const -> uint64_t;
    auto static3dRevision() const -> uint64_t {
        return online_mapper_.revision();
    }
    auto extractStatic3dCloud(
        const Eigen::Vector3d& box_min,
        const Eigen::Vector3d& box_max,
        std::vector<Eigen::Vector3f>& out,
        size_t max_points = 300000
    ) const -> uint64_t;
    auto getConfig() const -> const rog_map::Config& {
        return cfg_;
    }

private:
    rog_map::Config cfg_;
    std::atomic<uint8_t> mode_ {static_cast<uint8_t>(MapMode::MAPPING)};

    // 时间源：默认 steady_clock；use_sim_time 时由 ROS 适配层注入 /clock 原子时间源。
    utils::Clock clock_ = utils::Clock::steady();

    // stvl_prior 感知后端组件
    SensorModel sensor_model_;
    OnlineTerrainMapper online_mapper_;
    DynamicStvlLayer dynamic_stvl_;

    mutable std::mutex input_mtx_;
    // 里程计状态与新鲜度时钟域状态（均由 input_mtx_ 保护）：事件时间只与事件时间比较，接收时间只与本地时钟比较。
    rog_map::Pose odom_pose_ {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
    bool odom_rcv_ = false;
    double odom_stamp_sec_ = 0.0;
    bool odom_stamp_is_event_ = false;
    double odom_rx_sec_ = 0.0;
    rog_map::PointCloud cloud_;
    rog_map::Pose pc_pose_;
    // 与 cloud_ 同锁保护：本帧事件时间（无有效 header.stamp 时为接收时刻）。
    double pending_cloud_stamp_sec_ = 0.0;
    int unfinished_frame_cnt_ = 0;
    bool map_empty_ = true;

    // 仅由 processPendingFrame 单写者使用的实例级复用缓冲
    rog_map::PointCloud scratch_pc_;
    rog_map::Pose scratch_pose_;
    std::vector<Eigen::Vector3d> scratch_pts_;
    double last_empty_warn_t_ = 0.0;

    mutable std::mutex terrain_mtx_;
    utils::vec_E<utils::Vec3f> last_terrain_map_;

    // 按需 3D 调试可视化状态（默认全关；仅由上层按订阅者数量开启，关闭时零分配零拷贝）
    static constexpr uint8_t kVizCaptureFiltered = 1U << 0U;
    static constexpr uint8_t kVizCaptureDynamic3d = 1U << 1U;
    std::atomic<uint8_t> viz_capture_mask_ {0};
    std::vector<Eigen::Vector3f> scratch_filtered_cloud_;
    std::vector<Eigen::Vector3f> scratch_dynamic_3d_;
    mutable std::mutex viz_mtx_;
    std::vector<Eigen::Vector3f> last_filtered_cloud_;
    std::vector<Eigen::Vector3f> last_dynamic_3d_;
    std::atomic<uint64_t> filtered_cloud_rev_ {0};
    std::atomic<uint64_t> dynamic_3d_rev_ {0};
};

} // namespace ma_map
