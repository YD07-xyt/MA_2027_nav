/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <pcl/impl/point_types.hpp>
#include "common_lib.hpp"
#include "utils/logger.hpp"
#include <regex>
#include <utils/yaml_loader.hpp>
#include <utils/eigen_alias.hpp>
#ifndef ORIGIN_AT_CORNER
    #ifndef ORIGIN_AT_CENTER
        #error "Please define either ORIGIN_AT_CORNER or ORIGIN_AT_CENTER, but not both."
    #endif
#endif

#ifdef ORIGIN_AT_CORNER
    #ifdef ORIGIN_AT_CENTER
        #error "Cannot use both ORIGIN_AT_CENTER and ORIGIN_AT_CENTER at the same time. Please define only one."
    #endif
#endif

namespace rog_map {

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;

class Config {
    static std::string replaceCmakeRootDir(const std::string& input) {
#define CMAKE_ROOT_DIR(name) (std::string(std::string(ROOT_DIR) + name))
        std::string replaced;
        std::regex cmakeRootDirRegex(R"(\$\{CMAKE_ROOT_DIR\}/)");
        if (std::regex_search(input, cmakeRootDirRegex)) {
            replaced = std::regex_replace(input, cmakeRootDirRegex, "");
            replaced = CMAKE_ROOT_DIR(replaced);
            logger::Parameter->info("[map]The pcd_name is remapped to {}", replaced);
        } else {
            replaced = input;
        }
        return replaced;
    }

public:
    Config() = default;

    Config(const std::string& cfg_path, const std::string& name_space = "rog_map") {
        yaml_loader::YamlLoader loader(cfg_path);
#ifdef ORIGIN_AT_CORNER
    #ifdef ORIGIN_AT_CENTER
        throw std::runtime_error(" [SlidingMap]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both true!");
    #endif
#endif

#ifndef ORIGIN_AT_CORNER
    #ifndef ORIGIN_AT_CENTER
        throw std::runtime_error(" [SlidingMap]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both false!");
    #endif
#endif

        loader.LoadParam(name_space + "/esdf/resolution", esdf_resolution, 0.2);
        loader.LoadParam(name_space + "/esdf/enable", esdf_en, true);
        std::vector<double> temp_esdf_update_box;
        loader.LoadParam(name_space + "/esdf/local_update_box", temp_esdf_update_box, temp_esdf_update_box);

        if (esdf_en) {
            if (temp_esdf_update_box.size() != 3) {
                throw std::invalid_argument("Fix map origin size is not 3!");
            } else {
                esdf_local_update_box =
                    utils::Vec3f(temp_esdf_update_box[0], temp_esdf_update_box[1], temp_esdf_update_box[2]);
            }
        }

        loader.LoadParam(name_space + "/load_pcd_en", load_pcd_en, false);
        if (load_pcd_en) {
            loader.LoadParam(name_space + "/pcd_name", pcd_name, std::string("map.pcd"));
            pcd_name = replaceCmakeRootDir(pcd_name);
        }

        loader.LoadParam(name_space + "/map_sliding/enable", map_sliding_en, true);
        loader.LoadParam(name_space + "/map_sliding/threshold", map_sliding_thresh, -1.0);

        std::vector<double> temp_fix_origin;
        loader.LoadParam(name_space + "/fix_map_origin", temp_fix_origin, std::vector<double> {0, 0, 0});
        if (temp_fix_origin.size() != 3) {
            throw std::invalid_argument("Fix map origin size is not 3!");
        } else {
            fix_map_origin = utils::Vec3f(temp_fix_origin[0], temp_fix_origin[1], temp_fix_origin[2]);
        }

        loader.LoadParam(name_space + "/frontier_extraction_en", frontier_extraction_en, false);

        loader.LoadParam(name_space + "/ros_callback/enable", ros_callback_en, true);
        loader.LoadParam(name_space + "/ros_callback/cloud_topic", cloud_topic, std::string("/lio/cloud_world"));
        loader.LoadParam(name_space + "/ros_callback/odom_topic", odom_topic, std::string("/lio/odom"));
        loader.LoadParam(name_space + "/ros_callback/odom_timeout", odom_timeout, 0.05);

        loader.LoadParam(name_space + "/visualization/enable", visualization_en, true);
        loader.LoadParam(name_space + "/visualization/use_dynamic_reconfigure", use_dynamic_reconfigure, false);
        loader.LoadParam(name_space + "/visualization/pub_unknown_map_en", pub_unknown_map_en, false);
        loader.LoadParam(name_space + "/visualization/frame_id", frame_id, std::string("world"));
        loader.LoadParam(name_space + "/visualization/time_rate", viz_time_rate, 0.0);
        loader.LoadParam(name_space + "/visualization/frame_rate", viz_frame_rate, 0);
        std::vector<double> temp_vis_range;
        loader.LoadParam(name_space + "/visualization/range", temp_vis_range, std::vector<double> {0, 0, 0});
        if (temp_vis_range.size() != 3) {
            throw std::invalid_argument("Visualization range size is not 3!");
        } else {
            visualization_range = utils::Vec3f(temp_vis_range[0], temp_vis_range[1], temp_vis_range[2]);
            if (visualization_range.minCoeff() <= 0) {
                logger::Parameter->warn("[map]Visualization range is not set, visualization is disabled");
                visualization_en = false;
            }
        }

        loader.LoadParam(name_space + "/resolution", resolution, 0.1);
        loader.LoadParam(name_space + "/inflation_resolution", inflation_resolution, 0.1);
        /// Resize the map to ease indexing
        if (resolution > inflation_resolution) {
            throw std::invalid_argument("The inflation resolution should be equal or larger than the resolution!");
        }
        //    int scale = floor(inflation_resolution / resolution / 2);
        //    inflation_resolution = resolution * (scale * 2 + 1);
        //    ROS_ERROR("The inflation resolution is set to %f", inflation_resolution);

        /* For unk inflation */
        loader.LoadParam(name_space + "/unk_inflation_en", unk_inflation_en, false);
        loader.LoadParam(name_space + "/unk_inflation_step", unk_inflation_step, 1);

        loader.LoadParam(name_space + "/inflation_step", inflation_step, 1);
        loader.LoadParam(name_space + "/intensity_thresh", intensity_thresh, -1);

        std::vector<double> temp_map_size;
        loader.LoadParam(name_space + "/map_size", temp_map_size, std::vector<double> {10, 10, 0});
        if (temp_map_size.size() != 3) {
            throw std::invalid_argument("Map size dimension is not 3!");
        }
        map_size_d = utils::Vec3f(temp_map_size[0], temp_map_size[1], temp_map_size[2]);

        loader.LoadParam(name_space + "/point_filt_num", point_filt_num, 2);
        if (point_filt_num <= 0) {
            logger::Parameter->warn("[map]point_filt_num should be larger or equal than 1, it is set to 1 now.");
            point_filt_num = 1;
        }

        // raycasting
        loader.LoadParam(name_space + "/raycasting/enable", raycasting_en, true);
        loader.LoadParam(name_space + "/raycasting/batch_update_size", batch_update_size, 1);
        if (batch_update_size <= 0) {
            logger::Parameter->warn("[map]batch_update_size should be larger or equal than 1, it is set to 1 now.");
            batch_update_size = 1;
        }
        loader.LoadParam(name_space + "/raycasting/unk_thresh", unk_thresh, 0.70);
        loader.LoadParam(name_space + "/raycasting/p_hit", p_hit, 0.70f);
        loader.LoadParam(name_space + "/raycasting/p_miss", p_miss, 0.70f);
        loader.LoadParam(name_space + "/raycasting/p_min", p_min, 0.12f);
        loader.LoadParam(name_space + "/raycasting/p_max", p_max, 0.97f);
        loader.LoadParam(name_space + "/raycasting/p_occ", p_occ, 0.80f);
        loader.LoadParam(name_space + "/raycasting/p_free", p_free, 0.30f);
        loader.LoadParam(name_space + "/raycasting/p_free", p_free, 0.30f);

        std::vector<double> temp_ray_range;
        loader.LoadParam(name_space + "/raycasting/ray_range", temp_ray_range, std::vector<double> {0.3, 10});
        if (temp_ray_range.size() != 2) {
            throw std::invalid_argument("Ray range size is not 2!");
        }
        raycast_range_min = temp_ray_range[0];
        raycast_range_max = temp_ray_range[1];
        sqr_raycast_range_max = raycast_range_max * raycast_range_max;
        sqr_raycast_range_min = raycast_range_min * raycast_range_min;
        std::vector<double> update_box;
        loader.LoadParam(name_space + "/raycasting/local_update_box", update_box, std::vector<double> {999, 999, 999});
        if (update_box.size() != 3) {
            throw std::invalid_argument("Update box size is not 3!");
        }
        local_update_box_d = utils::Vec3f(update_box[0], update_box[1], update_box[2]);
        // 射线起点 = 机体位姿 + R * sensor_offset（机体系）。输入点云已是世界系，雷达位置只能由外参给出；
        // 缺省 [0,0,0] 即旧行为（射线从 base 位姿出发）。
        std::vector<double> sensor_offset_v;
        loader.LoadParam(name_space + "/raycasting/sensor_offset", sensor_offset_v, std::vector<double> {0, 0, 0});
        if (sensor_offset_v.size() != 3) {
            throw std::invalid_argument("Sensor offset size is not 3!");
        }
        sensor_offset = utils::Vec3f(sensor_offset_v[0], sensor_offset_v[1], sensor_offset_v[2]);

        loader.LoadParam(name_space + "/virtual_ground_height", virtual_ground_height, -0.1);
        loader.LoadParam(name_space + "/virtual_ceil_height", virtual_ceil_height, -0.1);
        loader.LoadParam(name_space + "/forget_threshold", forget_threshold, 10.0);
        loader.LoadParam(name_space + "/nav_forget_threshold", nav_forget_threshold, -1.0);
        loader.LoadParam(name_space + "/forget_keep_radius", forget_keep_radius, 0.0);

        loader.LoadParam(name_space + "/terrain/kernel_size", terrain_kernel_size, 5);
        loader.LoadParam(name_space + "/terrain/max_step_height", terrain_max_step_height, 0.17f);
        loader.LoadParam(name_space + "/terrain/robot_height", terrain_robot_height, 0.3f);
        loader.LoadParam(name_space + "/terrain/steep_threshold", terrain_steep_threshold, 0.19f);
        loader.LoadParam(name_space + "/terrain/min_overhang_clearance", terrain_min_overhang_clearance, 0.225f);
        loader.LoadParam(name_space + "/terrain/overhang_free_voxels", terrain_overhang_free_voxels, 3);
        loader.LoadParam(name_space + "/terrain/overhang_unknown_skip", terrain_overhang_unknown_skip, 1);
        loader.LoadParam(name_space + "/terrain/overhang_ground_radius", terrain_overhang_ground_radius, 6);
        loader.LoadParam(name_space + "/terrain/obstacle_band", terrain_obstacle_band, 0.08f);
        loader.LoadParam(name_space + "/terrain/ground_jump_thresh", terrain_ground_jump_thresh, 0.10f);
        loader.LoadParam(name_space + "/terrain/ground_min_radius", terrain_ground_min_radius, 2);
        // 持久地面层（阶段 1）与基于它的可达生长判定（阶段 2）
        loader.LoadParam(name_space + "/terrain/ground_layer/enable", ground_layer_en, false);
        loader.LoadParam(name_space + "/terrain/ground_layer/size", ground_layer_size, 20.0f);
        loader.LoadParam(name_space + "/terrain/ground_layer/max_range", ground_layer_max_range, 7.5f);
        loader.LoadParam(name_space + "/terrain/ground_layer/drop_votes", ground_layer_drop_votes, 2);
        loader.LoadParam(name_space + "/terrain/ground_layer/rise_votes", ground_layer_rise_votes, 3);
        loader.LoadParam(name_space + "/terrain/ground_layer/rise_max", ground_layer_rise_max, 0.24f);
        loader.LoadParam(name_space + "/terrain/ground_layer/footprint_radius", ground_layer_footprint_radius, 0.20f);
        loader.LoadParam(name_space + "/terrain/ground_layer/footprint_ground_offset", ground_layer_footprint_offset, 0.0f);
        loader.LoadParam(name_space + "/terrain/ground_layer/seed_radius", ground_layer_seed_radius, 1.5f);
        loader.LoadParam(name_space + "/terrain/ground_layer/tilt_jump_deg", ground_layer_tilt_jump_deg, 2.0f);
        loader.LoadParam(name_space + "/terrain/ground_layer/tilt_hold_frames", ground_layer_tilt_hold_frames, 2);
        loader.LoadParam(name_space + "/terrain/ground_layer/drop_min_support", ground_layer_drop_min_support, 2);
        loader.LoadParam(name_space + "/terrain/reach/enable", terrain_reach_en, false);
        loader.LoadParam(name_space + "/terrain/reach/max_slope_deg", terrain_max_slope_deg, 25.0f);
        loader.LoadParam(name_space + "/terrain/reach/step_tol", terrain_reach_step_tol, 0.03f);
        loader.LoadParam(name_space + "/terrain/reach/plane_rms_max", terrain_plane_rms_max, 0.02f);
        loader.LoadParam(name_space + "/terrain/reach/drop_edge_enable", terrain_drop_edge_en, false);
        loader.LoadParam(name_space + "/terrain/reach/drop_thresh", terrain_drop_thresh, 0.10f);
        // 2D 栅格地图边界：原点向 x 左/右、y 左/右各延伸多少米（x ∈ [-x_left, +x_right], y ∈ [-y_left, +y_right]）
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

        // ---------- 新感知后端（STVL 动态障碍 + 2.5D 静态先验 + 零先验在线 3D 建图）参数 ----------
        loader.LoadParam(name_space + "/perception/backend", perception_backend, std::string("stvl_prior"));
        if (perception_backend != "stvl_prior" && perception_backend != "legacy_rog") {
            throw std::invalid_argument(
                "Invalid rog_map/perception/backend: '" + perception_backend + "' (expected 'stvl_prior' or 'legacy_rog')"
            );
        }

        std::vector<double> t_bl_v;
        loader.LoadParam(
            name_space + "/perception/sensor/t_base_lidar",
            t_bl_v,
            std::vector<double> {
                static_cast<double>(sensor_offset.x()),
                static_cast<double>(sensor_offset.y()),
                static_cast<double>(sensor_offset.z()),
            }
        );
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
            throw std::invalid_argument("Invalid rog_map/perception/sensor/self_filter_{min,max} (expected 3 doubles)!");
        }
        perception_self_filter_min = Eigen::Vector3d(sf_min_v[0], sf_min_v[1], sf_min_v[2]);
        perception_self_filter_max = Eigen::Vector3d(sf_max_v[0], sf_max_v[1], sf_max_v[2]);

        loader.LoadParam(name_space + "/perception/mapping/trust_range", perception_map_trust_range, 4.0);
        loader.LoadParam(name_space + "/perception/mapping/clear_range", perception_map_clear_range, 8.0);
        loader.LoadParam(name_space + "/perception/mapping/clear_stop_margin", perception_map_clear_stop_margin, 0.15);
        loader.LoadParam(name_space + "/perception/mapping/min_z", perception_map_min_z, -1.0);
        loader.LoadParam(name_space + "/perception/mapping/max_z", perception_map_max_z, 2.5);
        loader.LoadParam(name_space + "/perception/mapping/static_min_hits", perception_map_static_min_hits, 2);
        loader.LoadParam(name_space + "/perception/mapping/static_clear_ratio", perception_map_static_clear_ratio, 2.0f);
        loader.LoadParam(name_space + "/perception/mapping/preview_radius", perception_map_preview_radius, 4.5);
        loader.LoadParam(name_space + "/perception/mapping/tilt_jump_deg", perception_map_tilt_jump_deg, 5.0);
        loader.LoadParam(name_space + "/perception/mapping/tilt_hold_frames", perception_map_tilt_hold_frames, 3);
        loader.LoadParam(
            name_space + "/perception/mapping/max_slope_deg",
            perception_map_max_slope_deg,
            terrain_max_slope_deg
        );
        loader.LoadParam(name_space + "/perception/mapping/step_tol", perception_map_step_tol, terrain_reach_step_tol);
        loader.LoadParam(name_space + "/perception/mapping/plane_rms_max", perception_map_plane_rms_max, 0.035f);
        loader.LoadParam(
            name_space + "/perception/mapping/max_surface_thickness",
            perception_map_max_surface_thickness,
            0.11f
        );

        loader.LoadParam(name_space + "/perception/navigation/max_range", perception_nav_max_range, 5.0);
        loader.LoadParam(name_space + "/perception/navigation/wall_suppress_dist", perception_nav_wall_suppress_dist, 0.08);
        loader.LoadParam(name_space + "/perception/navigation/ground_margin_base", perception_nav_ground_margin_base, 0.06f);
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
        loader.LoadParam(name_space + "/perception/navigation/tunnel_clearance_z", perception_nav_tunnel_clearance_z, 0.22f);
        loader.LoadParam(name_space + "/perception/navigation/ray_clear_margin", perception_nav_ray_clear_margin, 0.15f);
        loader.LoadParam(name_space + "/perception/navigation/miss_clear_frames", perception_nav_miss_clear_frames, 2);
        loader.LoadParam(name_space + "/perception/navigation/decay_in_fov_sec", perception_nav_decay_in_fov_sec, 0.35f);
        loader.LoadParam(name_space + "/perception/navigation/decay_blind_sec", perception_nav_decay_blind_sec, 0.45f);
        loader.LoadParam(name_space + "/perception/navigation/confirm_min_frames", perception_nav_confirm_min_frames, 2);
        loader.LoadParam(
            name_space + "/perception/navigation/fast_confirm_with_neighbor",
            perception_nav_fast_confirm_with_neighbor,
            true
        );
        loader.LoadParam(name_space + "/perception/navigation/tilt_jump_deg", perception_nav_tilt_jump_deg, 5.0);
        loader.LoadParam(name_space + "/perception/navigation/tilt_hold_frames", perception_nav_tilt_hold_frames, 3);
        loader.LoadParam(name_space + "/perception/navigation/drift_max_abs", perception_nav_drift_max_abs, 0.35f);
        loader.LoadParam(name_space + "/perception/navigation/drift_min_samples", perception_nav_drift_min_samples, 30);

        resetMapSize();

        /// Probabilistic Update
#define logit(x) (log((x) / (1 - (x))))
        l_hit = logit(p_hit);
        l_miss = logit(p_miss);
        l_min = logit(p_min);
        l_max = logit(p_max);
        l_occ = logit(p_occ);
        l_free = logit(p_free);

        int n_free = ceil(l_free / l_miss);
        int n_occ = ceil(l_occ / l_hit);
        logger::info(logger::Parameter,"n_free: {}", n_free);
        logger::info(logger::Parameter,"n_occ: {}", n_occ);
        logger::info(logger::Parameter,"l_hit: {}", l_hit);
        logger::info(logger::Parameter,"l_miss: {}", l_miss);
        logger::info(logger::Parameter,"l_min: {}", l_min);
        logger::info(logger::Parameter,"l_max: {}", l_max);
        logger::info(logger::Parameter,"l_occ: {}", l_occ);
        logger::info(logger::Parameter,"l_free: {}", l_free);

        // init spherical neighbor
        for (int dx = -inflation_step; dx <= inflation_step; dx++) {
            for (int dy = -inflation_step; dy <= inflation_step; dy++) {
                for (int dz = -inflation_step; dz <= inflation_step; dz++) {
                    if (inflation_step == 1 || dx * dx + dy * dy + dz * dz <= inflation_step * inflation_step) {
                        inf_spherical_neighbor.emplace_back(dx, dy, dz);
                    }
                }
            }
        }
        std::sort(
            inf_spherical_neighbor.begin(),
            inf_spherical_neighbor.end(),
            [](const utils::Vec3i& a, const utils::Vec3i& b) {
                return a.x() * a.x() + a.y() * a.y() + a.z() * a.z() < b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
            }
        );

        if (unk_inflation_en) {
            unk_inf_spherical_neighbor.clear();
            // init spherical neighbor
            for (int dx = -unk_inflation_step; dx <= unk_inflation_step; dx++) {
                for (int dy = -unk_inflation_step; dy <= unk_inflation_step; dy++) {
                    for (int dz = -unk_inflation_step; dz <= unk_inflation_step; dz++) {
                        if (unk_inflation_step == 1
                            || dx * dx + dy * dy + dz * dz <= unk_inflation_step * unk_inflation_step) {
                            unk_inf_spherical_neighbor.emplace_back(dx, dy, dz);
                        }
                    }
                }
            }
            std::sort(
                unk_inf_spherical_neighbor.begin(),
                unk_inf_spherical_neighbor.end(),
                [](const utils::Vec3i& a, const utils::Vec3i& b) {
                    return a.x() * a.x() + a.y() * a.y() + a.z() * a.z()
                        < b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
                }
            );
        }

        /* init spherical neighbor for nearest neighbor search */
        constexpr double max_search_dis = 5.0;
        const int max_seach_step = ceil(max_search_dis / resolution);
        for (int dx = -max_seach_step; dx <= max_seach_step; dx++) {
            for (int dy = -max_seach_step; dy <= max_seach_step; dy++) {
                for (int dz = -max_seach_step; dz <= max_seach_step; dz++) {
                    if (dx * dx + dy * dy + dz * dz <= max_seach_step * max_seach_step) {
                        spherical_neighbor.emplace_back(dx, dy, dz);
                    }
                }
            }
        }
        std::sort(
            spherical_neighbor.begin(),
            spherical_neighbor.end(),
            [](const utils::Vec3i& a, const utils::Vec3i& b) {
                return a.x() * a.x() + a.y() * a.y() + a.z() * a.z() < b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
            }
        );
    }

    // add 24.07.18 add esdf
    bool esdf_en {false};
    utils::Vec3f esdf_local_update_box {};
    double esdf_resolution {};

    bool load_pcd_en {false};
    bool use_dynamic_reconfigure {false};
    std::string pcd_name {"map.pcd"};

    double resolution {}, inflation_resolution {};
    int inflation_step {};
    utils::Vec3f local_update_box_d, half_local_update_box_d {};
    utils::Vec3i local_update_box_i, half_local_update_box_i {};
    utils::Vec3f map_size_d, half_map_size_d {};
    utils::Vec3i inf_half_map_size_i {}, half_map_size_i {}, fro_half_map_size_i {};

    /* The inf map virtual ceil should consider the inflation step
         *  the float type ceil and ground height is remap to the lower and upper bound of
         *  the inflation layer respectively
         */
    double virtual_ceil_height {}, virtual_ground_height {};
    int inf_virtual_ceil_height_id_g {}, inf_virtual_ground_height_id_g {};

    bool visualization_en {false}, frontier_extraction_en {false}, raycasting_en {true}, ros_callback_en {false},
        pub_unknown_map_en {false};

    /* Spherical neighbor for inflation*/
    std::vector<utils::Vec3i> inf_spherical_neighbor {};
    std::vector<utils::Vec3i> unk_inf_spherical_neighbor {};
    /* Spherical neighbor for nearest search within x m*/
    std::vector<utils::Vec3i> spherical_neighbor {};

    /* intensity noise filter*/
    int intensity_thresh {};
    /* aster properties */
    std::string frame_id {};
    bool map_sliding_en {true};
    utils::Vec3f fix_map_origin {};
    std::string odom_topic {}, cloud_topic {};
    /* probability update */
    double raycast_range_min {}, raycast_range_max {};
    double sqr_raycast_range_min {}, sqr_raycast_range_max {};
    int point_filt_num {}, batch_update_size {};
    float p_hit {}, p_miss {}, p_min {}, p_max {}, p_occ {}, p_free {};
    float l_hit {}, l_miss {}, l_min {}, l_max {}, l_occ {}, l_free {};

    /* for unknown inflation */
    bool unk_inflation_en {false};
    int unk_inflation_step {0};

    double odom_timeout {};
    utils::Vec3f visualization_range {};
    double viz_time_rate {};
    int viz_frame_rate {};

    double unk_thresh {};
    double map_sliding_thresh {};
    double forget_threshold {};  // 占据体素遗忘衰减时间（秒），≤0 禁用
    double nav_forget_threshold {};  // 导航模式下的遗忘时间（秒），≤0 禁用；建图模式用 forget_threshold
    double forget_keep_radius {};    // 车周多少米内不遗忘（水平距离）

    // ---------- terrain / 2D grid_map 参数 ----------
    int terrain_kernel_size{};
    float terrain_max_step_height{};
    float terrain_robot_height{};
    float terrain_steep_threshold{};
    float terrain_min_overhang_clearance{};
    int terrain_overhang_free_voxels{};
    int terrain_overhang_unknown_skip{};
    int terrain_overhang_ground_radius{};
    float terrain_obstacle_band{};
    float terrain_ground_jump_thresh{};
    int terrain_ground_min_radius{};
    utils::Vec3f sensor_offset {utils::Vec3f::Zero()};
    bool ground_layer_en {false};
    bool terrain_reach_en {false}, terrain_drop_edge_en {false};
    float terrain_max_slope_deg {}, terrain_reach_step_tol {}, terrain_plane_rms_max {}, terrain_drop_thresh {};
    float ground_layer_size {}, ground_layer_max_range {}, ground_layer_rise_max {};
    int ground_layer_drop_votes {}, ground_layer_rise_votes {};
    float ground_layer_footprint_radius {}, ground_layer_footprint_offset {}, ground_layer_seed_radius {};
    float ground_layer_tilt_jump_deg {};
    int ground_layer_tilt_hold_frames {}, ground_layer_drop_min_support {};
    double grid_map_x_left{20.5};
    double grid_map_x_right{20.5};
    double grid_map_y_left{8.0};
    double grid_map_y_right{8.0};
    double grid_map_resolution{0.05};
    double grid_map_expand_chunk_m {4.0};
    double grid_map_max_size_x {80.0};
    double grid_map_max_size_y {80.0};
    int grid_map_save_margin_cells {2};
    bool grid_map_unknown_as_occupied {true};

    // ---------- perception (stvl_prior / legacy_rog) 参数 ----------
    std::string perception_backend {"stvl_prior"};
    Eigen::Vector3d perception_t_base_lidar {0.0, 0.0, 0.194};
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

    void resetMapSize() {
        int inflation_ratio = ceil(inflation_resolution / resolution);

#ifdef ORIGIN_AT_CENTER
        // When discretized in such a way that the origin is at cell center
        // the inflation ratio should be an odd number
        if (inflation_ratio % 2 == 0) {
            inflation_ratio += 1;
        }
        logger::Parameter->info("[map]inflation_ratio: {}", inflation_ratio);
#endif

        inflation_resolution = resolution * inflation_ratio;
        // std::cout << color_text::GREEN << " -- [RM-Config] inflation_resolution: " << inflation_resolution
        //           << color_text::RESET << std::endl;

        half_map_size_d = map_size_d / 2;

        // Size_d is only for calculate index number.
        // 1) we calculate the index number of the inf map
        int max_step = 0;
        if (!unk_inflation_en) {
            max_step = inflation_step;
        } else {
            max_step = std::max(inflation_step, unk_inflation_step);
        }
        inf_half_map_size_i =
            (half_map_size_d / inflation_resolution).cast<int>() + (max_step + 1) * utils::Vec3i::Ones();

        // 2) we calculate the index number of the prob map, which should be smaller than infmap
        half_map_size_i = (inf_half_map_size_i - (max_step + 1) * utils::Vec3i::Ones()) * inflation_ratio;

        // 3) compute the frontier counter map size
        if (frontier_extraction_en) {
            fro_half_map_size_i = half_map_size_i + utils::Vec3i::Constant(1);
        }

        // 4) re-compute the map_size_d, the map_size_d is not that important.
        map_size_d = (half_map_size_i * 2 + utils::Vec3i::Constant(1)).cast<double>() * resolution;
        half_map_size_d = map_size_d / 2;

        // 5) compute the index of raycasting update box
        half_local_update_box_d = local_update_box_d / 2;
        half_local_update_box_i = (half_local_update_box_d / resolution).cast<int>();
        local_update_box_i = half_local_update_box_i * 2 + utils::Vec3i::Constant(1);
        local_update_box_d = local_update_box_i.cast<double>() * resolution;

        // 6) reset the virtual ground and ceil size
#ifdef ORIGIN_AT_CENTER
        inf_virtual_ceil_height_id_g =
            static_cast<int>(virtual_ceil_height / inflation_resolution + SIGN(virtual_ceil_height) * 0.5);
        inf_virtual_ground_height_id_g =
            static_cast<int>(virtual_ground_height / inflation_resolution + SIGN(virtual_ground_height) * 0.5);
        logger::Parameter->info("[map]Init, resetMapSize: ORIGIN_AT_CENTER.");
#endif

#ifdef ORIGIN_AT_CORNER
        inf_virtual_ceil_height_id_g =
            static_cast<int>(floor(virtual_ceil_height / inflation_resolution)) - inflation_step;
        inf_virtual_ground_height_id_g =
            static_cast<int>(floor(virtual_ground_height / inflation_resolution)) + inflation_step;

        logger::Parameter->info("[map]Init, resetMapSize: ORIGIN_AT_CORNER.");
#endif

        virtual_ceil_height = inf_virtual_ceil_height_id_g * inflation_resolution - 0.5 * inflation_resolution;
        virtual_ground_height = inf_virtual_ground_height_id_g * inflation_resolution + 0.5 * inflation_resolution;
        // fmt::print(
        //     " -- [ROG-Map] Init, resetMapSize: virtual_ceil_height: {}, virtual_ground_height: {}.\n",
        //     virtual_ceil_height,
        //     virtual_ground_height
        // );
    }
};
}
