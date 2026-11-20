#pragma once

#include "utils/eigen_alias.hpp"

#include <Eigen/Core>

namespace ma_map {

// Layer 2（局部 3D 感知管线）向 Layer 3（分层 2D 地图）输出的帧数据结构。
// 所有坐标均为世界坐标系，单位米 (m)。
struct LocalTerrainFrame {
    utils::vec_E<utils::Vec3f> obstacle_cells; // 本帧局部地形分析判定的障碍格中心
    utils::vec_E<utils::Vec3f> free_cells; // 本帧局部地形分析判定的空闲证据格中心
    Eigen::Vector2d robot_pos = Eigen::Vector2d::Zero(); // 本帧机器人世界系 2D 位置 (m)
    double terrain_resolution = 0.05; // 局部地形格边长 (m)
    double robot_footprint_radius = 0.20; // 机器人底盘足迹半径 (m)，建图模式用于消除车底盲区 UNKNOWN
    bool has_robot_pos = false;
};

} // namespace ma_map
