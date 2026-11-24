#pragma once
#include "utils/logger.hpp"
#include "path_planning/search/astar.h"
#include "path_planning/search/jps.h"
#include "path_planning/post_processing.h"
#include <optional>
namespace path_planning {

class PathPlanningImpl;

class PathPlanning {
private:
    std::unique_ptr<PathPlanningImpl> impl_;

public:
    PathPlanning();
    ~PathPlanning();                       // ← 显式声明析构函数
    PathPlanning(PathPlanning&&) noexcept;  // 移动构造/赋值也要声明
    PathPlanning& operator=(PathPlanning&&) noexcept;

    // 禁止拷贝（unique_ptr 本身不可拷贝）
    PathPlanning(const PathPlanning&) = delete;
    PathPlanning& operator=(const PathPlanning&) = delete;
public:
    // 设置前端搜索与后处理参数（同步更新 use_jps、safe_threshold 与 jps_start_snap_radius）。
    auto set_param(const PathPostProcessing::PathPostProcessingParams& params) -> void;
    auto set_velocity(const Eigen::Vector3d& start_vel, const Eigen::Vector3d& end_vel) -> void;

    // 显式覆写底层图搜索算法（未调用 set_param 前默认 A*；调用 set_param 后以 params.use_jps 为准）
    auto set_use_jps(bool use_jps) -> void;
    // 同步只读借用外部 GridMap 快照（调用期间须保持 grid_map 有效且不被并发修改）。
    auto path_planning(
        const grid_map::GridMap& grid_map,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        double start_yaw,
        double goal_yaw,
        int timeout_ms
    ) -> std::optional<PathPostProcessing::Trajectory>;
};
}
