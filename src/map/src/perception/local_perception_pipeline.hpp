#pragma once

#include "map/3d_occ_map/config.hpp"
#include "map/3d_occ_map/rog_map.h"
#include "map/perception/ground_layer.hpp"
#include "map/perception/local_terrain_frame.hpp"
#include "map/perception/terrain_analysis.hpp"

#include <memory>
#include <mutex>
#include <optional>

namespace ma_map {

// Layer 2：局部 3D 感知管线（封装 ROGMap + GroundLayer + TerrainAnalyzer）。
// 仅在 3d_occ_map 内部编译，通过 LocalTerrainFrame 向 Layer 3（LayeredMap）单向输出。
// 线程契约：
// - updateOdom / updateCloud / processPendingFrame 通常由同一 map callback group 串行调用；
//   cloud_/pc_pose_ 由 input_mtx_ 保护，scratch 缓冲为实例成员（替代旧实现中的线程不安全函数 static）。
class LocalPerceptionPipeline {
public:
    explicit LocalPerceptionPipeline(const rog_map::Config& cfg);

    auto setForgetThresholdForMode(bool mapping_mode) -> void;
    auto updateOdom(const rog_map::Pose& pose) -> void;
    auto updateCloud(const rog_map::PointCloud& cloud) -> void;

    // 若存在待处理点云帧，则更新 3D ROGMap、持久地面层与局部地形分析，输出本帧 LocalTerrainFrame；
    // 无新帧时返回 std::nullopt。
    auto processPendingFrame() -> std::optional<LocalTerrainFrame>;

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
    std::shared_ptr<rog_map::ROGMap> rog_map_ptr_;
    Terrain::TerrainAnalyzer terrain_analyzer_;
    Terrain::GroundLayer ground_layer_;

    mutable std::mutex input_mtx_;
    rog_map::PointCloud cloud_;
    rog_map::Pose pc_pose_;
    int unfinished_frame_cnt_ = 0;
    bool map_empty_ = true;

    // 仅由 processPendingFrame 单写者使用的实例级复用缓冲
    rog_map::PointCloud scratch_pc_;
    rog_map::Pose scratch_pose_;
    utils::vec_E<utils::Vec3f> scratch_real_occ_;
    double last_empty_warn_t_ = 0.0;

    mutable std::mutex terrain_mtx_;
    utils::vec_E<utils::Vec3f> last_terrain_map_;
};

} // namespace ma_map
