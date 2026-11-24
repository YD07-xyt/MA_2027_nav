#pragma once

#include <pcl/impl/point_types.hpp>
#include "common_lib.hpp"
#include "utils/logger.hpp"
#include <utils/eigen_alias.hpp>
#include <utils/yaml_loader.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace rog_map {

using utils::Pose;
typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;

class Config {
public:
    Config() = default;

    Config(const std::string& cfg_path, const std::string& name_space = "rog_map") {
        yaml_loader::YamlLoader loader(cfg_path);

        // ---------- ROS 回调与可视化参数 ----------
        loader.LoadParam(name_space + "/ros_callback/enable", ros_callback_en, true);
        loader.LoadParam(name_space + "/ros_callback/odom_timeout", odom_timeout, 0.05);

        loader.LoadParam(name_space + "/visualization/time_rate", viz_time_rate, 0.0);
        std::vector<double> temp_vis_range;
        loader.LoadParam(name_space + "/visualization/range", temp_vis_range, std::vector<double> {100.0, 60.0, 4.0});
        if (temp_vis_range.size() != 3) {
            throw std::invalid_argument("Visualization range size is not 3!");
        }
        visualization_range = utils::Vec3f(temp_vis_range[0], temp_vis_range[1], temp_vis_range[2]);

        // ---------- 输入点云预过滤参数 ----------
        loader.LoadParam(name_space + "/intensity_thresh", intensity_thresh, -1);
        loader.LoadParam(name_space + "/point_filt_num", point_filt_num, 1);
        if (point_filt_num <= 0) {
            logger::Parameter->warn("[map]point_filt_num should be larger or equal than 1, it is set to 1 now.");
            point_filt_num = 1;
        }

        // ---------- 2D 栅格地图参数 ----------
        // 边界按原点向四个方向的延伸量设置：x ∈ [-x_left, +x_right], y ∈ [-y_left, +y_right]
        loader.LoadParam(name_space + "/grid_map/x_left", grid_map_x_left, 20.5);
        loader.LoadParam(name_space + "/grid_map/x_right", grid_map_x_right, 20.5);
        loader.LoadParam(name_space + "/grid_map/y_left", grid_map_y_left, 8.0);
        loader.LoadParam(name_space + "/grid_map/y_right", grid_map_y_right, 8.0);
        loader.LoadParam(name_space + "/grid_map/resolution", grid_map_resolution, 0.05);
        loader.LoadParam(name_space + "/grid_map/expand_chunk_m", grid_map_expand_chunk_m, 4.0);
        if (!std::isfinite(grid_map_expand_chunk_m) || grid_map_expand_chunk_m <= 0.0) {
            logger::Parameter->warn("[map] invalid grid_map/expand_chunk_m, fallback to 4.0");
            grid_map_expand_chunk_m = 4.0;
        }
        std::vector<double> max_map_size_v;
        loader.LoadParam(name_space + "/grid_map/max_map_size", max_map_size_v, std::vector<double> {80.0, 80.0});
        if (max_map_size_v.size() == 2 && std::isfinite(max_map_size_v[0]) && std::isfinite(max_map_size_v[1])
            && max_map_size_v[0] > 0.0 && max_map_size_v[1] > 0.0)
        {
            grid_map_max_size_x = max_map_size_v[0];
            grid_map_max_size_y = max_map_size_v[1];
        } else {
            logger::Parameter->warn("[map] invalid grid_map/max_map_size, fallback to [80.0, 80.0]");
            grid_map_max_size_x = 80.0;
            grid_map_max_size_y = 80.0;
        }
        loader.LoadParam(name_space + "/grid_map/save_margin_cells", grid_map_save_margin_cells, 2);
        if (grid_map_save_margin_cells < 0) {
            grid_map_save_margin_cells = 0;
        }
        loader.LoadParam(name_space + "/grid_map/unknown_as_occupied", grid_map_unknown_as_occupied, true);

        // ---------- 感知后端（STVL 动态障碍 + 2.5D 静态先验 + 零先验在线 3D 建图）参数 ----------
        loader.LoadParam(name_space + "/perception/robot/footprint_radius", perception_footprint_radius, 0.20);
        loader.LoadParam(
            name_space + "/perception/robot/footprint_ground_offset",
            perception_footprint_ground_offset,
            0.0
        );
        loader.LoadParam(name_space + "/perception/robot/robot_height", perception_robot_height, 0.3f);

        std::vector<double> t_bl_v;
        loader.LoadParam(name_space + "/perception/sensor/t_base_lidar", t_bl_v, std::vector<double> {0.0, 0.0, 0.0});
        if (t_bl_v.size() != 3 || !std::isfinite(t_bl_v[0]) || !std::isfinite(t_bl_v[1]) || !std::isfinite(t_bl_v[2])) {
            throw std::invalid_argument("Invalid rog_map/perception/sensor/t_base_lidar (expected 3 finite doubles)!");
        }
        perception_t_base_lidar = Eigen::Vector3d(t_bl_v[0], t_bl_v[1], t_bl_v[2]);

        std::vector<double> rpy_v;
        loader.LoadParam(name_space + "/perception/sensor/rpy_deg", rpy_v, std::vector<double> {0.0, 0.0, 0.0});
        if (rpy_v.size() != 3 || !std::isfinite(rpy_v[0]) || !std::isfinite(rpy_v[1]) || !std::isfinite(rpy_v[2])) {
            throw std::invalid_argument("Invalid rog_map/perception/sensor/rpy_deg (expected 3 finite doubles)!");
        }
        perception_rpy_deg = Eigen::Vector3d(rpy_v[0], rpy_v[1], rpy_v[2]);

        loader.LoadParam(name_space + "/perception/sensor/fov_min_elev_deg", perception_fov_min_elev_deg, -7.0);
        loader.LoadParam(name_space + "/perception/sensor/fov_max_elev_deg", perception_fov_max_elev_deg, 52.0);
        loader.LoadParam(name_space + "/perception/sensor/fov_margin_deg", perception_fov_margin_deg, 3.0);
        loader.LoadParam(name_space + "/perception/sensor/min_range", perception_sensor_min_range, 0.20);
        loader.LoadParam(name_space + "/perception/sensor/max_range", perception_sensor_max_range, 15.0);

        std::vector<double> sf_min_v;
        std::vector<double> sf_max_v;
        loader.LoadParam(
            name_space + "/perception/sensor/self_filter_min",
            sf_min_v,
            std::vector<double> {-0.30, -0.30, -0.05}
        );
        loader.LoadParam(
            name_space + "/perception/sensor/self_filter_max",
            sf_max_v,
            std::vector<double> {0.30, 0.30, 0.35}
        );
        if (sf_min_v.size() != 3 || sf_max_v.size() != 3) {
            throw std::invalid_argument("Invalid rog_map/perception/sensor/self_filter_{min,max} (expected 3 doubles)!"
            );
        }
        perception_self_filter_min = Eigen::Vector3d(sf_min_v[0], sf_min_v[1], sf_min_v[2]);
        perception_self_filter_max = Eigen::Vector3d(sf_max_v[0], sf_max_v[1], sf_max_v[2]);

        loader.LoadParam(name_space + "/perception/mapping/trust_range", perception_map_trust_range, 4.0);
        loader.LoadParam(name_space + "/perception/mapping/clear_range", perception_map_clear_range, 8.0);
        loader.LoadParam(name_space + "/perception/mapping/clear_stop_margin", perception_map_clear_stop_margin, 0.15);
        loader.LoadParam(name_space + "/perception/mapping/min_z", perception_map_min_z, -1.0);
        loader.LoadParam(name_space + "/perception/mapping/max_z", perception_map_max_z, 2.5);
        loader.LoadParam(name_space + "/perception/mapping/static_min_hits", perception_map_static_min_hits, 2);
        loader
            .LoadParam(name_space + "/perception/mapping/static_clear_ratio", perception_map_static_clear_ratio, 2.0f);
        loader.LoadParam(name_space + "/perception/mapping/preview_radius", perception_map_preview_radius, 4.5);
        loader.LoadParam(name_space + "/perception/mapping/tilt_jump_deg", perception_map_tilt_jump_deg, 5.0);
        loader.LoadParam(name_space + "/perception/mapping/tilt_hold_frames", perception_map_tilt_hold_frames, 3);
        loader.LoadParam(name_space + "/perception/mapping/max_slope_deg", perception_map_max_slope_deg, 25.0f);
        loader.LoadParam(name_space + "/perception/mapping/step_tol", perception_map_step_tol, 0.03f);
        loader.LoadParam(name_space + "/perception/mapping/plane_rms_max", perception_map_plane_rms_max, 0.035f);
        loader.LoadParam(
            name_space + "/perception/mapping/max_surface_thickness",
            perception_map_max_surface_thickness,
            0.11f
        );
        loader.LoadParam(
            name_space + "/perception/mapping/min_overhang_clearance",
            perception_map_min_overhang_clearance,
            0.225f
        );
        loader.LoadParam(
            name_space + "/perception/mapping/overhang_ground_radius",
            perception_map_overhang_ground_radius,
            6
        );

        loader.LoadParam(name_space + "/perception/navigation/max_range", perception_nav_max_range, 5.0);
        loader.LoadParam(
            name_space + "/perception/navigation/wall_suppress_dist",
            perception_nav_wall_suppress_dist,
            0.08
        );
        loader.LoadParam(
            name_space + "/perception/navigation/ground_margin_base",
            perception_nav_ground_margin_base,
            0.06f
        );
        loader.LoadParam(
            name_space + "/perception/navigation/ground_margin_per_m",
            perception_nav_ground_margin_per_m,
            0.012f
        );
        loader.LoadParam(
            name_space + "/perception/navigation/dynamic_max_height",
            perception_nav_dynamic_max_height,
            0.85f
        );
        loader.LoadParam(name_space + "/perception/navigation/overhang_margin", perception_nav_overhang_margin, 0.06f);
        loader.LoadParam(
            name_space + "/perception/navigation/tunnel_clearance_z",
            perception_nav_tunnel_clearance_z,
            0.22f
        );
        loader
            .LoadParam(name_space + "/perception/navigation/ray_clear_margin", perception_nav_ray_clear_margin, 0.15f);
        loader.LoadParam(name_space + "/perception/navigation/miss_clear_frames", perception_nav_miss_clear_frames, 2);
        loader
            .LoadParam(name_space + "/perception/navigation/decay_in_fov_sec", perception_nav_decay_in_fov_sec, 0.35f);
        loader.LoadParam(name_space + "/perception/navigation/decay_blind_sec", perception_nav_decay_blind_sec, 0.45f);
        loader
            .LoadParam(name_space + "/perception/navigation/confirm_min_frames", perception_nav_confirm_min_frames, 2);
        loader.LoadParam(
            name_space + "/perception/navigation/fast_confirm_with_neighbor",
            perception_nav_fast_confirm_with_neighbor,
            true
        );
        loader.LoadParam(name_space + "/perception/navigation/tilt_jump_deg", perception_nav_tilt_jump_deg, 5.0);
        loader.LoadParam(name_space + "/perception/navigation/tilt_hold_frames", perception_nav_tilt_hold_frames, 3);
        loader.LoadParam(name_space + "/perception/navigation/drift_max_abs", perception_nav_drift_max_abs, 0.35f);
        loader.LoadParam(name_space + "/perception/navigation/drift_min_samples", perception_nav_drift_min_samples, 30);
    }

    // ---------- ROS 回调、可视化与输入过滤参数 ----------
    bool ros_callback_en {true};
    double odom_timeout {0.05};
    double viz_time_rate {0.0};
    utils::Vec3f visualization_range {100.0, 60.0, 4.0};
    int intensity_thresh {-1};
    int point_filt_num {1};

    // ---------- 2D grid_map 参数 ----------
    double grid_map_x_left {20.5};
    double grid_map_x_right {20.5};
    double grid_map_y_left {8.0};
    double grid_map_y_right {8.0};
    double grid_map_resolution {0.05};
    double grid_map_expand_chunk_m {4.0};
    double grid_map_max_size_x {80.0};
    double grid_map_max_size_y {80.0};
    int grid_map_save_margin_cells {2};
    bool grid_map_unknown_as_occupied {true};

    // ---------- perception (stvl_prior) 参数 ----------
    double perception_footprint_radius {0.20};
    double perception_footprint_ground_offset {0.0};
    float perception_robot_height {0.3f};

    Eigen::Vector3d perception_t_base_lidar {0.0, 0.0, 0.0};
    Eigen::Vector3d perception_rpy_deg {0.0, 0.0, 0.0};
    double perception_fov_min_elev_deg {-7.0};
    double perception_fov_max_elev_deg {52.0};
    double perception_fov_margin_deg {3.0};
    double perception_sensor_min_range {0.20};
    double perception_sensor_max_range {15.0};
    Eigen::Vector3d perception_self_filter_min {-0.30, -0.30, -0.05};
    Eigen::Vector3d perception_self_filter_max {0.30, 0.30, 0.35};

    double perception_map_trust_range {4.0};
    double perception_map_clear_range {8.0};
    double perception_map_clear_stop_margin {0.15};
    double perception_map_min_z {-1.0};
    double perception_map_max_z {2.5};
    int perception_map_static_min_hits {2};
    float perception_map_static_clear_ratio {2.0f};
    double perception_map_preview_radius {4.5};
    double perception_map_tilt_jump_deg {5.0};
    int perception_map_tilt_hold_frames {3};
    float perception_map_max_slope_deg {25.0f};
    float perception_map_step_tol {0.03f};
    float perception_map_plane_rms_max {0.035f};
    float perception_map_max_surface_thickness {0.11f};
    float perception_map_min_overhang_clearance {0.225f};
    int perception_map_overhang_ground_radius {6};

    double perception_nav_max_range {5.0};
    double perception_nav_wall_suppress_dist {0.08};
    float perception_nav_ground_margin_base {0.06f};
    float perception_nav_ground_margin_per_m {0.012f};
    float perception_nav_dynamic_max_height {0.85f};
    float perception_nav_overhang_margin {0.06f};
    float perception_nav_tunnel_clearance_z {0.22f};
    float perception_nav_ray_clear_margin {0.15f};
    int perception_nav_miss_clear_frames {2};
    float perception_nav_decay_in_fov_sec {0.35f};
    float perception_nav_decay_blind_sec {0.45f};
    int perception_nav_confirm_min_frames {2};
    bool perception_nav_fast_confirm_with_neighbor {true};
    double perception_nav_tilt_jump_deg {5.0};
    int perception_nav_tilt_hold_frames {3};
    float perception_nav_drift_max_abs {0.35f};
    int perception_nav_drift_min_samples {30};
};

} // namespace rog_map
