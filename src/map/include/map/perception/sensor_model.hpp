#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <limits>
#include <span>

namespace ma_map {

// 单帧世界系下的机器人基座与 LiDAR 6-DoF 位姿快照（坐标均在世界系，单位 m）。
struct SensorPose {
    Eigen::Vector3d p_base_w = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_w_base = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_base_w = Eigen::Matrix3d::Identity();

    Eigen::Vector3d p_lidar_w = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_w_lidar = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_lidar_w = Eigen::Matrix3d::Identity();

    auto worldToBase(const Eigen::Vector3d& p_world) const -> Eigen::Vector3d {
        return R_base_w * (p_world - p_base_w);
    }

    auto worldToLidar(const Eigen::Vector3d& p_world) const -> Eigen::Vector3d {
        if (!p_world.allFinite() || !p_lidar_w.allFinite() || !R_lidar_w.allFinite()) {
            return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        }
        return R_lidar_w * (p_world - p_lidar_w);
    }
};

// 6-DoF LiDAR 安装外参、视场与机体自滤模型（无 PCL / ROS 依赖）。
// 坐标系约定（ROS REP-103 右手系：+X 前、+Y 左、+Z 上）：
// - R_base_lidar = Rz(yaw) * Ry(pitch) * Rx(roll)，将雷达系向量变换到 base_odom 系；
// - 右手系绕 +Y 旋转：pitch < 0 时雷达前向 +X 向上仰起（如 pitch = -20 deg 表示上仰 20°），
//   pitch > 0 时雷达前向 +X 向下俯冲。
class SensorModel {
public:
    struct Config {
        Eigen::Vector3d t_base_lidar = Eigen::Vector3d(0.0, 0.0, 0.194); // LiDAR 相对 base_odom 平移 (m, base 系)
        Eigen::Vector3d rpy_deg = Eigen::Vector3d(0.0, -20.0, 0.0); // REP-103 [roll, pitch, yaw] (deg)
        double fov_min_elev_deg = -7.0; // Mid-360 雷达系垂直视场下限 (deg)
        double fov_max_elev_deg = 52.0; // Mid-360 雷达系垂直视场上限 (deg)
        double fov_margin_deg = 3.0; // 视场边界容差 (deg)
        double min_range = 0.20; // 最小有效距离 (m)
        double max_range = 15.0; // 最大视场/量测有效距离 (m)
        Eigen::Vector3d self_filter_min = Eigen::Vector3d(-0.30, -0.30, -0.05); // base 系自滤盒下界 (m)
        Eigen::Vector3d self_filter_max = Eigen::Vector3d(0.30, 0.30, 0.35); // base 系自滤盒上界 (m)
    };

    SensorModel();
    explicit SensorModel(const Config& cfg);

    auto setConfig(const Config& cfg) -> void;
    auto config() const -> const Config& {
        return cfg_;
    }

    auto rotationBaseLidar() const -> const Eigen::Matrix3d& {
        return R_base_lidar_;
    }

    // 由 /lio/base_odom 位姿计算当前帧基座与 LiDAR 的世界系 6-DoF 位姿。
    auto computePose(const Eigen::Vector3d& p_base_w, const Eigen::Quaterniond& q_base_w) const -> SensorPose;

    // 判断世界系点是否落入机器人 base 系自滤盒内（过滤底盘、云台与炮管自遮挡点）。
    auto isSelfHit(const Eigen::Vector3d& p_world, const SensorPose& pose) const -> bool;

    // 判断雷达系点 p_lidar 是否位于当前 LiDAR 的距离与垂直视场内（含 fov_margin_deg 容差）。
    auto inFov(const Eigen::Vector3d& p_lidar) const -> bool;
    auto inFov(const Eigen::Vector3d& p_lidar, double max_range_override) const -> bool;

    // 计算两帧机体 Z 轴夹角 (deg)，用于姿态跳变门控（抗撞坎/上下坡配准抖动）。
    static auto tiltAngleDeg(const Eigen::Quaterniond& q_prev, const Eigen::Quaterniond& q_curr) -> double;

    // 启动一次性外参自检：抽样检验点云在雷达系下的仰角分布是否与 [fov_min, fov_max] 一致；
    // 若超 40% 点落在宽容视场之外则打印一次告警（不阻塞主流程）。
    auto checkExtrinsicsOnce(std::span<const Eigen::Vector3d> pts_world, const SensorPose& pose) const -> void;

    auto resetExtrinsicsCheck() -> void {
        extrinsics_checked_.store(false, std::memory_order_release);
    }

private:
    Config cfg_;
    Eigen::Matrix3d R_base_lidar_ = Eigen::Matrix3d::Identity();
    double min_elev_rad_ = 0.0;
    double max_elev_rad_ = 0.0;
    double min_range_sq_ = 0.0;
    double max_range_sq_ = 0.0;
    mutable std::atomic<bool> extrinsics_checked_ {false};
};

} // namespace ma_map
