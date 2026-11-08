#pragma once
#include "controller/mpc.h"
#include "replan/fsm_replanner.h"
#include "utils/logger.hpp"
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>
#include <vector>

namespace planner {

// ============================================================
// 3 维工具
// ============================================================

inline Eigen::Vector3d load_vector3d(const YAML::Node& node) {
    if (!node.IsDefined() || !node.IsSequence() || node.size() != 3) {
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

inline Eigen::Matrix3d load_matrix3d(const YAML::Node& node) {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    if (!node.IsDefined()) return mat;

    if (node.IsSequence() && node.size() == 9) {
        // 按行优先填充 3x3
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                mat(i, j) = node[i * 3 + j].as<double>();
            }
        }
    } else if (node.IsSequence() && node.size() == 3) {
        bool nested = true;
        for (int i = 0; i < 3; ++i) {
            if (!node[i].IsSequence() || node[i].size() != 3) {
                nested = false;
                break;
            }
        }

        if (nested) {
            // 3x3 嵌套数组
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    mat(i, j) = node[i][j].as<double>();
                }
            }
        } else {
            // 长度为 3 的列表，按对角矩阵处理
            for (int i = 0; i < 3; ++i) {
                mat(i, i) = node[i].as<double>();
            }
        }
    }

    return mat;
}

// ============================================================
// 2 维工具（新 MPC 控制量 ax, ay）
// ============================================================

inline Eigen::Matrix<double, 2, 1> load_vector2d(const YAML::Node& node) {
    Eigen::Matrix<double, 2, 1> vec = Eigen::Matrix<double, 2, 1>::Zero();
    if (!node.IsDefined() || !node.IsSequence() || node.size() != 2) {
        return vec;
    }
    vec(0) = node[0].as<double>();
    vec(1) = node[1].as<double>();
    return vec;
}

inline Eigen::Matrix<double, 2, 2> load_matrix2d(const YAML::Node& node) {
    Eigen::Matrix<double, 2, 2> mat = Eigen::Matrix<double, 2, 2>::Zero();
    if (!node.IsDefined()) return mat;

    if (node.IsSequence() && node.size() == 4) {
        // 按行优先填充 2x2
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                mat(i, j) = node[i * 2 + j].as<double>();
            }
        }
    } else if (node.IsSequence() && node.size() == 2) {
        bool nested = true;
        for (int i = 0; i < 2; ++i) {
            if (!node[i].IsSequence() || node[i].size() != 2) {
                nested = false;
                break;
            }
        }

        if (nested) {
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    mat(i, j) = node[i][j].as<double>();
                }
            }
        } else {
            // 长度为 2 的列表，按对角矩阵处理
            for (int i = 0; i < 2; ++i) {
                mat(i, i) = node[i].as<double>();
            }
        }
    }

    return mat;
}

// ============================================================
// 4 维工具（新 MPC 状态 x, y, vx, vy）
// ============================================================

inline Eigen::Matrix<double, 4, 1> load_vector4d(const YAML::Node& node) {
    Eigen::Matrix<double, 4, 1> vec = Eigen::Matrix<double, 4, 1>::Zero();
    if (!node.IsDefined() || !node.IsSequence() || node.size() != 4) {
        return vec;
    }
    for (int i = 0; i < 4; ++i) {
        vec(i) = node[i].as<double>();
    }
    return vec;
}

inline Eigen::Matrix<double, 4, 4> load_matrix4d(const YAML::Node& node) {
    Eigen::Matrix<double, 4, 4> mat = Eigen::Matrix<double, 4, 4>::Zero();
    if (!node.IsDefined()) return mat;

    if (node.IsSequence() && node.size() == 16) {
        // 按行优先填充 4x4
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                mat(i, j) = node[i * 4 + j].as<double>();
            }
        }
    } else if (node.IsSequence() && node.size() == 4) {
        bool nested = true;
        for (int i = 0; i < 4; ++i) {
            if (!node[i].IsSequence() || node[i].size() != 4) {
                nested = false;
                break;
            }
        }

        if (nested) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    mat(i, j) = node[i][j].as<double>();
                }
            }
        } else {
            // 长度为 4 的列表，按对角矩阵处理
            for (int i = 0; i < 4; ++i) {
                mat(i, i) = node[i].as<double>();
            }
        }
    }

    return mat;
}

// ============================================================
// 加载 MaSplineOptimizerConfig
// ============================================================

inline void load_stage_config(const YAML::Node& node, ma_spline_opt::StageOptimizerConfig& stage) {
    if (!node) return;

    if (node["rho_energy"]) stage.rho_energy = node["rho_energy"].as<double>();
    if (node["weight_time"]) stage.weight_time = node["weight_time"].as<double>();
    if (node["weight_mean_time"]) stage.weight_mean_time = node["weight_mean_time"].as<double>();
    if (node["mean_lower"]) stage.mean_lower = node["mean_lower"].as<double>();
    if (node["mean_upper"]) stage.mean_upper = node["mean_upper"].as<double>();
    if (node["min_time"]) stage.min_time = node["min_time"].as<double>();
    if (node["weight_min_time"]) stage.weight_min_time = node["weight_min_time"].as<double>();
    if (node["weight_obstacle"]) stage.weight_obstacle = node["weight_obstacle"].as<double>();
    if (node["weight_vel"]) stage.weight_vel = node["weight_vel"].as<double>();
    if (node["weight_acc"]) stage.weight_acc = node["weight_acc"].as<double>();
    if (node["max_iterations"]) stage.max_iterations = node["max_iterations"].as<int>();
    if (node["g_epsilon"]) stage.g_epsilon = node["g_epsilon"].as<double>();
    if (node["lbfgs_mem_size"]) stage.lbfgs_mem_size = node["lbfgs_mem_size"].as<int>();
    if (node["lbfgs_delta"]) stage.lbfgs_delta = node["lbfgs_delta"].as<double>();
}

inline void load_ma_spline_opt_config(const YAML::Node& node, ma_spline_opt::MaSplineOptimizerConfig& param) {
    if (!node) return;

    if (node["safe_distance"]) param.safe_distance = node["safe_distance"].as<double>();
    if (node["v_max"]) param.v_max = node["v_max"].as<double>();
    if (node["a_max"]) param.a_max = node["a_max"].as<double>();
    if (node["integral_num_steps"]) param.integral_num_steps = node["integral_num_steps"].as<int>();

    if (node["weight_yaw_vel"]) param.weight_yaw_vel = node["weight_yaw_vel"].as<double>();
    if (node["weight_yaw_acc"]) param.weight_yaw_acc = node["weight_yaw_acc"].as<double>();
    if (node["yaw_rate_max"]) param.yaw_rate_max = node["yaw_rate_max"].as<double>();
    if (node["yaw_acc_max"]) param.yaw_acc_max = node["yaw_acc_max"].as<double>();

    if (node["stage1"]) load_stage_config(node["stage1"], param.stage1);
    if (node["stage2"]) load_stage_config(node["stage2"], param.stage2);
}



// ============================================================
// 加载 ReplanParam
// ============================================================

inline void load_replan_param(const YAML::Node& node, replan::FsmReplan::ReplanParam& param) {
    if (!node) return;
    if (node["goal_deviation"]) param.goal_deviation = load_vector3d(node["goal_deviation"]);
    if (node["replan_lateral_dev"]) param.replan_lateral_dev = node["replan_lateral_dev"].as<double>();
    if (node["minco_traj_validity_duration"])
        param.minco_traj_validity_duration = node["minco_traj_validity_duration"].as<double>();
    if (node["goal_reached_radius"]) param.goal_reached_radius = node["goal_reached_radius"].as<double>();
    if (node["minco_traj_continuity_threshold"])
        param.minco_traj_continuity_threshold = node["minco_traj_continuity_threshold"].as<double>();
    if (node["projection_search_resolution"])
        param.projection_search_resolution = node["projection_search_resolution"].as<double>();
    if (node["brake_acc"]) param.brake_acc = node["brake_acc"].as<double>();
    if (node["brake_acc_max"]) param.brake_acc_max = node["brake_acc_max"].as<double>();
}

// ============================================================
// 加载 PathPostProcessingParams
// ============================================================
inline void load_path_post_processing_params(
    const YAML::Node& node,
    path_planning::PathPostProcessing::PathPostProcessingParams& param
) {
    if (!node) return;

    if (node["max_traj_num"]) param.max_traj_num = node["max_traj_num"].as<int>();
    if (node["dense_sample_resolution"]) param.dense_sample_resolution = node["dense_sample_resolution"].as<double>();
    if (node["rotation_penalty_weight"]) param.rotation_penalty_weight = node["rotation_penalty_weight"].as<double>();
    if (node["safe_threshold"]) param.safe_threshold = node["safe_threshold"].as<double>();
    if (node["max_vel"]) param.max_vel = node["max_vel"].as<double>();
    if (node["max_acc"]) param.max_acc = node["max_acc"].as<double>();
    if (node["time_resolution"]) param.time_resolution = node["time_resolution"].as<double>();
    if (node["min_traj_num"]) param.min_traj_num = node["min_traj_num"].as<int>();
    if (node["traj_cut_length"]) param.traj_cut_length = node["traj_cut_length"].as<double>();
    if (node["distance_weight"]) param.distance_weight = node["distance_weight"].as<double>();
    if (node["yaw_weight"]) param.yaw_weight = node["yaw_weight"].as<double>();

    // ---------- 云台折叠参数 ----------
    if (node["fold_time"]) param.fold_time = node["fold_time"].as<double>();
    if (node["unfold_time"]) param.unfold_time = node["unfold_time"].as<double>();
    if (node["fold_margin"]) param.fold_margin = node["fold_margin"].as<double>();
    if (node["fold_prep_speed"]) param.fold_prep_speed = node["fold_prep_speed"].as<double>();
    if (node["fold_prep_margin"]) param.fold_prep_margin = node["fold_prep_margin"].as<double>();
}

// ============================================================
// 加载 PlannerConfig
// ============================================================

inline void load_planner_config(const YAML::Node& node, replan::FsmReplan::PlannerConfig& config) {
    if (!node) return;
    if (node["replan_params"]) load_replan_param(node["replan_params"], config.replan_params);
    if (node["ma_spline_opt_params"])
        load_ma_spline_opt_config(node["ma_spline_opt_params"], config.ma_spline_opt_params);
    if (node["path_planning_params"])
        load_path_post_processing_params(node["path_planning_params"], config.path_planning_params);
}

// ============================================================
// 加载新 4 状态 MPC 参数
// 状态: [x, y, vx, vy]
// 控制: [ax, ay]
// ============================================================

inline void load_mpc_param(const YAML::Node& node, control::MpcParam& param) {
    if (!node) return;

    if (node["N"]) param.N = node["N"].as<int>();
    if (node["dt"]) param.dt = node["dt"].as<double>();

    if (node["Q"]) param.Q = load_matrix4d(node["Q"]);
    if (node["QN"]) param.QN = load_matrix4d(node["QN"]);

    if (node["R"]) param.R = load_matrix2d(node["R"]);
    if (node["Rd"]) param.Rd = load_matrix2d(node["Rd"]);

    if (node["x_min"]) param.x_min = load_vector4d(node["x_min"]);
    if (node["x_max"]) param.x_max = load_vector4d(node["x_max"]);

    if (node["u_min"]) param.u_min = load_vector2d(node["u_min"]);
    if (node["u_max"]) param.u_max = load_vector2d(node["u_max"]);
}

// ============================================================
// Config
// ============================================================

struct Config {
    std::string map_topic_name;
    std::string target_topic_name;
    std::string clickedPoint_topic_name;
    std::string odom_topic_name;
    std::string cmd_vel_name;
    std::string save_global_map_path;
    std::string save_map_srv_topic;
    int num_threads;
    std::string frame_id = "world"; // 全局坐标系，所有发布消息统一使用
    // 点击隧道区域
    std::string tunnel_regions_path;
    std::string save_tunnel_regions_srv_topic;

    bool mapping_model; // true: 建图模式，false: 规划模式
    std::string map_params_path;
    std::string global_map_path;

    replan::FsmReplan::PlannerConfig planner_config;
    control::MpcParam mpc_params;

    explicit Config(std::string params_path) {
        YAML::Node config = YAML::LoadFile(params_path);

        if (config["ros2"]) {
            num_threads=config["ros2"]["num_threads"].as<int>();
            if (config["ros2"]["frame_id"]) frame_id = config["ros2"]["frame_id"].as<std::string>();
            map_topic_name = config["ros2"]["map_topic_name"].as<std::string>();
            target_topic_name = config["ros2"]["target_topic_name"].as<std::string>();
            clickedPoint_topic_name = config["ros2"]["clickedPoint_topic_name"].as<std::string>();
            odom_topic_name = config["ros2"]["odom_topic_name"].as<std::string>();
            cmd_vel_name = config["ros2"]["cmd_vel_name"].as<std::string>();
            map_params_path = config["ros2"]["map_params_path"].as<std::string>();
            global_map_path = config["ros2"]["global_map_path"].as<std::string>();
            save_global_map_path = config["ros2"]["save_global_map_path"].as<std::string>();
            save_map_srv_topic = config["ros2"]["save_map_srv_topic"].as<std::string>();
            tunnel_regions_path = config["ros2"]["tunnel_regions_path"].as<std::string>();
            save_tunnel_regions_srv_topic = config["ros2"]["save_tunnel_regions_srv_topic"].as<std::string>();

        }

        if (config["map"]) {
            mapping_model = config["map"]["mapping_model"].as<bool>();
        }

        if (config["planner_config"]) {
            load_planner_config(config["planner_config"], planner_config);
        }

        if (config["mpc_param"]) {
            load_mpc_param(config["mpc_param"], mpc_params);
        }
    }
};

} // namespace planner