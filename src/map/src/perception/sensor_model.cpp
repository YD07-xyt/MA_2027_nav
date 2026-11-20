#include "map/perception/sensor_model.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ma_map {

namespace {

    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

} // namespace

SensorModel::SensorModel() {
    setConfig(Config {});
}

SensorModel::SensorModel(const Config& cfg) {
    setConfig(cfg);
}

auto SensorModel::setConfig(const Config& cfg) -> void {
    cfg_ = cfg;
    const double roll = cfg_.rpy_deg.x() * kDegToRad;
    const double pitch = cfg_.rpy_deg.y() * kDegToRad;
    const double yaw = cfg_.rpy_deg.z() * kDegToRad;

    const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
    R_base_lidar_ = (rz * ry * rx).toRotationMatrix();

    min_elev_rad_ = (cfg_.fov_min_elev_deg - cfg_.fov_margin_deg) * kDegToRad;
    max_elev_rad_ = (cfg_.fov_max_elev_deg + cfg_.fov_margin_deg) * kDegToRad;
    const double r_min = std::max(0.0, cfg_.min_range);
    const double r_max = std::max(r_min, cfg_.max_range);
    min_range_sq_ = r_min * r_min;
    max_range_sq_ = r_max * r_max;
    extrinsics_checked_.store(false, std::memory_order_release);
}

auto SensorModel::computePose(const Eigen::Vector3d& p_base_w, const Eigen::Quaterniond& q_base_w) const -> SensorPose {
    SensorPose pose;
    pose.p_base_w = p_base_w;
    const Eigen::Quaterniond q_norm =
        (q_base_w.squaredNorm() > 1e-12) ? q_base_w.normalized() : Eigen::Quaterniond::Identity();
    pose.R_w_base = q_norm.toRotationMatrix();
    const Eigen::Matrix3d R_bw = pose.R_w_base.transpose();
    pose.R_base_w = R_bw;

    pose.p_lidar_w = pose.p_base_w + pose.R_w_base * cfg_.t_base_lidar;
    pose.R_w_lidar = pose.R_w_base * R_base_lidar_;
    const Eigen::Matrix3d R_lw = pose.R_w_lidar.transpose();
    pose.R_lidar_w = R_lw;
    return pose;
}

auto SensorModel::isSelfHit(const Eigen::Vector3d& p_world, const SensorPose& pose) const -> bool {
    const Eigen::Vector3d p_base = pose.worldToBase(p_world);
    return p_base.x() >= cfg_.self_filter_min.x() && p_base.x() <= cfg_.self_filter_max.x()
        && p_base.y() >= cfg_.self_filter_min.y() && p_base.y() <= cfg_.self_filter_max.y()
        && p_base.z() >= cfg_.self_filter_min.z() && p_base.z() <= cfg_.self_filter_max.z();
}

auto SensorModel::inFov(const Eigen::Vector3d& p_lidar) const -> bool {
    return inFov(p_lidar, cfg_.max_range);
}

auto SensorModel::inFov(const Eigen::Vector3d& p_lidar, double max_range_override) const -> bool {
    const double r_sq = p_lidar.squaredNorm();
    const double r_max_sq = max_range_override * max_range_override;
    if (r_sq < min_range_sq_ || r_sq > r_max_sq) {
        return false;
    }
    const double xy = std::hypot(p_lidar.x(), p_lidar.y());
    const double elev = std::atan2(p_lidar.z(), xy);
    return elev >= min_elev_rad_ && elev <= max_elev_rad_;
}

auto SensorModel::tiltAngleDeg(const Eigen::Quaterniond& q_prev, const Eigen::Quaterniond& q_curr) -> double {
    if (q_prev.squaredNorm() <= 1e-12 || q_curr.squaredNorm() <= 1e-12) {
        return 0.0;
    }
    const Eigen::Matrix3d R_prev = q_prev.normalized().toRotationMatrix();
    const Eigen::Matrix3d R_curr = q_curr.normalized().toRotationMatrix();
    const Eigen::Vector3d z_prev = R_prev.col(2);
    const Eigen::Vector3d z_curr = R_curr.col(2);
    const double cos_ang = std::clamp(z_prev.dot(z_curr), -1.0, 1.0);
    return std::acos(cos_ang) * kRadToDeg;
}

auto SensorModel::checkExtrinsicsOnce(std::span<const Eigen::Vector3d> pts_world, const SensorPose& pose) const
    -> void {
    if (extrinsics_checked_.load(std::memory_order_acquire) || pts_world.size() < 100) {
        return;
    }

    std::vector<double> elevs_deg;
    elevs_deg.reserve(std::min<size_t>(pts_world.size(), 1024));
    const size_t step = std::max<size_t>(1, pts_world.size() / 1024);

    int outside = 0;
    const double lo_deg = cfg_.fov_min_elev_deg - 10.0;
    const double hi_deg = cfg_.fov_max_elev_deg + 10.0;

    for (size_t i = 0; i < pts_world.size(); i += step) {
        const Eigen::Vector3d& pw = pts_world[i];
        if (!pw.allFinite() || isSelfHit(pw, pose)) {
            continue;
        }
        const Eigen::Vector3d pl = pose.worldToLidar(pw);
        const double r = pl.norm();
        if (r < 0.5 || r > 8.0) {
            continue;
        }
        const double elev_deg = std::atan2(pl.z(), std::hypot(pl.x(), pl.y())) * kRadToDeg;
        elevs_deg.push_back(elev_deg);
        if (elev_deg < lo_deg || elev_deg > hi_deg) {
            ++outside;
        }
    }

    if (elevs_deg.size() < 64) {
        return;
    }

    bool expected = false;
    if (!extrinsics_checked_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;
    }

    const double out_ratio = static_cast<double>(outside) / static_cast<double>(elevs_deg.size());
    if (out_ratio > 0.40) {
        std::sort(elevs_deg.begin(), elevs_deg.end());
        const double p5 = elevs_deg[elevs_deg.size() * 5 / 100];
        const double p50 = elevs_deg[elevs_deg.size() / 2];
        const double p95 = elevs_deg[elevs_deg.size() * 95 / 100];
        logger::MaMap->warn(
            "[SensorModel] Extrinsics self-check: {:.1f}% points fall outside [{:.1f}, {:.1f}] deg "
            "(measured lidar elev p5={:.1f}, p50={:.1f}, p95={:.1f} deg; configured rpy_deg=[{:.1f}, {:.1f}, {:.1f}]). "
            "Check sensor.rpy_deg sign (REP-103: pitch=-20 tilts +X upward 20 deg).",
            out_ratio * 100.0,
            lo_deg,
            hi_deg,
            p5,
            p50,
            p95,
            cfg_.rpy_deg.x(),
            cfg_.rpy_deg.y(),
            cfg_.rpy_deg.z()
        );
    }
}

} // namespace ma_map
