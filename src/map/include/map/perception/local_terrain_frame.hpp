#pragma once

#include "utils/eigen_alias.hpp"

#include <Eigen/Core>

namespace ma_map {

// Layer 2（局部 3D 感知管线）向 Layer 3（分层 2D 地图）输出的帧数据结构。
// 所有坐标均为世界坐标系，单位米 (m)。
struct LocalTerrainFrame {
    utils::vec_E<utils::Vec3f> obstacle_cells; // 本帧局部地形分析判定的障碍格中心
    utils::vec_E<utils::Vec3f> free_cells; // 本帧局部地形分析判定的空闲证据格中心
    // 本帧已从 3D 静态占据中抹除且无残留静态占据的格中心（仅用于将旧 OCCUPIED 退回 UNKNOWN）
    utils::vec_E<utils::Vec3f> cleared_cells;
    Eigen::Vector2d robot_pos = Eigen::Vector2d::Zero(); // 本帧机器人世界系 2D 位置 (m)
    double terrain_resolution = 0.05; // 局部地形格边长 (m)
    double robot_footprint_radius = 0.20; // 机器人底盘足迹半径 (m)，建图模式用于消除车底盲区 UNKNOWN
    bool has_robot_pos = false;
    // true 时 free_cells 来自已累积多帧 3D 证据（如 OnlineTerrainMapper），单帧即可清除旧 OCCUPIED
    bool authoritative_free = false;
};

} // namespace ma_map
