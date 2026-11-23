/*
 * NAV 核心管线离线基准测试（无 ROS 依赖、零仓库副作用）。
 *
 * 覆盖模块（按 Google Benchmark 名称前缀分组，可用 --benchmark_filter 按模块筛选）：
 *   - GridMap/*      : ESDF 计算（复用工作区 vs 无工作区）、占据脏检查、快照 ESDF 复用拷贝
 *   - MaMap/*        : 占据不变跳过 ESDF、单格变化完整发布、语义多边形复用 ESDF、单线程/并发快照读取
 *   - PathPlanning/* : A* / JPS 缓存命中、JPS 跨快照缓存重建、前端完整流程（JPS + 后处理）
 *   - TrajOptimize/* : MaSplineTrajectoryOptimizer 两阶段 L-BFGS（RAII 作用域 ESDF）
 *   - FsmReplan/*    : 稳态监控快路径、端到端完整重规划
 *   - Controller/*   : 线性 MPC（OSQP）单步求解
 *
 * 约定：
 *   - 只读读取 images/rmuc2024.png、config/map.yaml、config/planner.yaml，不向仓库目录写图或 CSV。
 *   - 启动基准前执行全模块正确性自检，任一失败立即返回非零退出码；传 --sanity-only 仅跑自检。
 *   - 推荐配合 --benchmark_repetitions=5 --benchmark_out=.cache/bench/<name>.json
 *     --benchmark_out_format=json 与 scripts/compare_bench.py 进行改前/改后对比。
 */

#include "controller/mpc.h"
#include "controller/traj_interface.hpp"
#include "cv2map.hpp"
#include "map/core/grid_map.hpp"
#include "map/core/map_io.hpp"
#include "map/core/static_terrain_prior.hpp"
#include "map/layered/layered_map.hpp"
#include "map/ma_map.hpp"
#include "map/perception/dynamic_stvl_layer.hpp"
#include "map/perception/online_terrain_mapper.hpp"
#include "map/perception/sensor_model.hpp"
#include "map/perception/static_terrain_builder.hpp"
#include "path_planning/path_planning.hpp"
#include "path_planning/post_processing.h"
#include "path_planning/search/astar.h"
#include "path_planning/search/jps.h"
#include "replan/fsm_replanner.h"
#include "traj_optimize/grid_map_esdf.hpp"
#include "traj_optimize/ma_spline_opt/optimizer_config.h"
#include "traj_optimize/ma_spline_opt/traj_optimizer.h"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef ROOT_DIR
    #define ROOT_DIR "./"
#endif

namespace {

constexpr double kResolution = 0.05;
constexpr int kNumQueries = 16;
constexpr double kMinQueryDist = 8.0;
constexpr double kMinAltGoalDist = 4.0;
constexpr int kTimeoutMs = 2000;
constexpr unsigned kSeed = 42;
constexpr int kMpcSamples = 20;

const std::string kRootDir = std::string(ROOT_DIR);
const std::string kMapImagePath = kRootDir + "images/rmuc2024.png";
const std::string kMapConfigPath = kRootDir + "config/map.yaml";
const std::string kPlannerConfigPath = kRootDir + "config/planner.yaml";

struct Query {
    Eigen::Vector2d start;
    Eigen::Vector2d goal;
};

struct MpcSample {
    control::StateVector x0 = control::StateVector::Zero();
    double t_now = 0.0;
};

auto make_robot_state(const Eigen::Vector2d& xy) -> utils::RobotState {
    utils::RobotState st {};
    st.p = utils::Vec3f(xy.x(), xy.y(), 0.0);
    st.v = utils::Vec3f::Zero();
    st.a = utils::Vec3f::Zero();
    st.j = utils::Vec3f::Zero();
    st.yaw = 0.0;
    st.wz = 0.0;
    st.rcv_time = 0.0;
    st.rcv = true;
    st.q = utils::Quatf::Identity();
    return st;
}

// ---------------- 参数读取（键名与 src/ros2/include/ros2/config.hpp 一致；不链接 ros2） ----------------

auto load_vector2d(const YAML::Node& node) -> Eigen::Matrix<double, 2, 1> {
    Eigen::Matrix<double, 2, 1> vec = Eigen::Matrix<double, 2, 1>::Zero();
    if (!node.IsDefined() || !node.IsSequence() || node.size() != 2) {
        return vec;
    }
    vec(0) = node[0].as<double>();
    vec(1) = node[1].as<double>();
    return vec;
}

auto load_vector3d(const YAML::Node& node) -> Eigen::Vector3d {
    if (!node.IsDefined() || !node.IsSequence() || node.size() != 3) {
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

auto load_vector4d(const YAML::Node& node) -> Eigen::Matrix<double, 4, 1> {
    Eigen::Matrix<double, 4, 1> vec = Eigen::Matrix<double, 4, 1>::Zero();
    if (!node.IsDefined() || !node.IsSequence() || node.size() != 4) {
        return vec;
    }
    for (int i = 0; i < 4; ++i) {
        vec(i) = node[i].as<double>();
    }
    return vec;
}

auto load_matrix2d(const YAML::Node& node) -> Eigen::Matrix<double, 2, 2> {
    Eigen::Matrix<double, 2, 2> mat = Eigen::Matrix<double, 2, 2>::Zero();
    if (!node.IsDefined() || !node.IsSequence()) return mat;
    if (node.size() == 4) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                mat(i, j) = node[i * 2 + j].as<double>();
            }
        }
    } else if (node.size() == 2) {
        const bool nested = node[0].IsSequence() && node[0].size() == 2 && node[1].IsSequence() && node[1].size() == 2;
        if (nested) {
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    mat(i, j) = node[i][j].as<double>();
                }
            }
        } else {
            for (int i = 0; i < 2; ++i) {
                mat(i, i) = node[i].as<double>();
            }
        }
    }
    return mat;
}

auto load_matrix4d(const YAML::Node& node) -> Eigen::Matrix<double, 4, 4> {
    Eigen::Matrix<double, 4, 4> mat = Eigen::Matrix<double, 4, 4>::Zero();
    if (!node.IsDefined() || !node.IsSequence()) return mat;
    if (node.size() == 16) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                mat(i, j) = node[i * 4 + j].as<double>();
            }
        }
    } else if (node.size() == 4) {
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
            for (int i = 0; i < 4; ++i) {
                mat(i, i) = node[i].as<double>();
            }
        }
    }
    return mat;
}

auto load_stage_config(const YAML::Node& node, ma_spline_opt::StageOptimizerConfig& stage) -> void {
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

auto load_planner_config(const YAML::Node& root) -> replan::FsmReplan::PlannerConfig {
    const YAML::Node pc = root["planner_config"];
    if (!pc) throw std::runtime_error("planner_config not found in " + kPlannerConfigPath);

    replan::FsmReplan::PlannerConfig cfg {};
    if (const YAML::Node pp = pc["path_planning_params"]) {
        auto& p = cfg.path_planning_params;
        if (pp["safe_threshold"]) p.safe_threshold = pp["safe_threshold"].as<double>();
        if (pp["max_vel"]) p.max_vel = pp["max_vel"].as<double>();
        if (pp["max_acc"]) p.max_acc = pp["max_acc"].as<double>();
        if (pp["time_resolution"]) p.time_resolution = pp["time_resolution"].as<double>();
        if (pp["min_traj_num"]) p.min_traj_num = pp["min_traj_num"].as<int>();
        if (pp["max_traj_num"]) p.max_traj_num = pp["max_traj_num"].as<int>();
        if (pp["traj_cut_length"]) p.traj_cut_length = pp["traj_cut_length"].as<double>();
        if (pp["distance_weight"]) p.distance_weight = pp["distance_weight"].as<double>();
        if (pp["yaw_weight"]) p.yaw_weight = pp["yaw_weight"].as<double>();
        if (pp["dense_sample_resolution"]) p.dense_sample_resolution = pp["dense_sample_resolution"].as<double>();
        if (pp["rotation_penalty_weight"]) p.rotation_penalty_weight = pp["rotation_penalty_weight"].as<double>();
        if (pp["fold_time"]) p.fold_time = pp["fold_time"].as<double>();
        if (pp["unfold_time"]) p.unfold_time = pp["unfold_time"].as<double>();
        if (pp["fold_margin"]) p.fold_margin = pp["fold_margin"].as<double>();
        if (pp["fold_prep_speed"]) p.fold_prep_speed = pp["fold_prep_speed"].as<double>();
        if (pp["fold_prep_margin"]) p.fold_prep_margin = pp["fold_prep_margin"].as<double>();
    }
    if (const YAML::Node opt = pc["ma_spline_opt_params"]) {
        auto& o = cfg.ma_spline_opt_params;
        if (opt["safe_distance"]) o.safe_distance = opt["safe_distance"].as<double>();
        if (opt["v_max"]) o.v_max = opt["v_max"].as<double>();
        if (opt["a_max"]) o.a_max = opt["a_max"].as<double>();
        if (opt["integral_num_steps"]) o.integral_num_steps = opt["integral_num_steps"].as<int>();
        if (opt["weight_yaw_vel"]) o.weight_yaw_vel = opt["weight_yaw_vel"].as<double>();
        if (opt["weight_yaw_acc"]) o.weight_yaw_acc = opt["weight_yaw_acc"].as<double>();
        if (opt["yaw_rate_max"]) o.yaw_rate_max = opt["yaw_rate_max"].as<double>();
        if (opt["yaw_acc_max"]) o.yaw_acc_max = opt["yaw_acc_max"].as<double>();
        if (opt["stage1"]) load_stage_config(opt["stage1"], o.stage1);
        if (opt["stage2"]) load_stage_config(opt["stage2"], o.stage2);
    }
    if (const YAML::Node rp = pc["replan_params"]) {
        auto& r = cfg.replan_params;
        if (rp["goal_deviation"]) r.goal_deviation = load_vector3d(rp["goal_deviation"]);
        if (rp["replan_lateral_dev"]) r.replan_lateral_dev = rp["replan_lateral_dev"].as<double>();
        if (rp["goal_reached_radius"]) r.goal_reached_radius = rp["goal_reached_radius"].as<double>();
        if (rp["projection_search_resolution"]) {
            r.projection_search_resolution = rp["projection_search_resolution"].as<double>();
        }
        if (rp["brake_acc"]) r.brake_acc = rp["brake_acc"].as<double>();
        if (rp["brake_acc_max"]) r.brake_acc_max = rp["brake_acc_max"].as<double>();
        if (rp["start_oob_recover_dist"]) r.start_oob_recover_dist = rp["start_oob_recover_dist"].as<double>();
    }
    return cfg;
}

auto load_mpc_param(const YAML::Node& root) -> control::MpcParam {
    control::MpcParam param {};
    const YAML::Node node = root["mpc_param"];
    if (!node) throw std::runtime_error("mpc_param not found in " + kPlannerConfigPath);
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
    return param;
}

// ---------------- 共享基准夹具 ----------------

struct BenchEnv {
    replan::FsmReplan::PlannerConfig planner_cfg {};
    control::MpcParam mpc_param {};

    // 生产网格尺寸（与 config/map.yaml 对齐：560x400 @ 0.05m）
    std::unique_ptr<ma_map::MaMap> ma_map;
    grid_map::RowMatrixXi ma_occ_a;
    grid_map::RowMatrixXi ma_occ_b;
    std::vector<Eigen::Vector2d> sem_polygon;
    grid_map::GridMap grid_src;
    grid_map::GridMap grid_dst;

    // 规划/优化/重规划地图（images/rmuc2024.png，560x300 @ 0.05m）
    std::unique_ptr<test_utils::Cv2Map> cv_map;
    grid_map::GridMap::ConstPtr gm_a;
    grid_map::GridMap::ConstPtr gm_b;
    std::unique_ptr<minco_opt::GridMapESDF> esdf;

    std::vector<Query> queries;
    std::vector<ma_spline_opt::MaSplineInput> opt_inputs;

    // FsmReplan 交替重规划场景（同起点，两个不同可达目标点）
    utils::RobotState fsm_start {};
    utils::RobotState fsm_goal_a {};
    utils::RobotState fsm_goal_b {};

    // MPC 跟踪场景
    std::shared_ptr<control::MaSplineTrajectoryInterface> mpc_traj;
    std::vector<MpcSample> mpc_samples;
};

auto extract_occupancy_matrix(const grid_map::GridMap& gm) -> grid_map::RowMatrixXi {
    const Eigen::Vector2i n = gm.getVoxelNum();
    const auto occ_span = gm.occData();
    grid_map::RowMatrixXi occ(n.x(), n.y());
    for (int x = 0; x < n.x(); ++x) {
        for (int y = 0; y < n.y(); ++y) {
            occ(x, y) = occ_span[static_cast<size_t>(x * n.y() + y)] != 0 ? 1 : 0;
        }
    }
    return occ;
}

class ScopedSilenceStdout {
public:
    ScopedSilenceStdout() {
        std::cout.flush();
        std::fflush(stdout);
        saved_fd_ = ::dup(STDOUT_FILENO);
        if (saved_fd_ < 0) return;
        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull < 0) {
            ::close(saved_fd_);
            saved_fd_ = -1;
            return;
        }
        if (::dup2(devnull, STDOUT_FILENO) < 0) {
            ::close(saved_fd_);
            saved_fd_ = -1;
        }
        ::close(devnull);
    }

    ~ScopedSilenceStdout() {
        if (saved_fd_ < 0) return;
        std::cout.flush();
        std::fflush(stdout);
        (void)::dup2(saved_fd_, STDOUT_FILENO);
        ::close(saved_fd_);
    }

    ScopedSilenceStdout(const ScopedSilenceStdout&) = delete;
    ScopedSilenceStdout& operator=(const ScopedSilenceStdout&) = delete;

private:
    int saved_fd_ = -1;
};

auto build_env() -> BenchEnv {
    BenchEnv env;
    const YAML::Node planner_yaml = YAML::LoadFile(kPlannerConfigPath);
    env.planner_cfg = load_planner_config(planner_yaml);
    env.mpc_param = load_mpc_param(planner_yaml);

    // 1. 加载 rmuc2024 黑白场地地图
    test_utils::Cv2Map::Options opt {};
    opt.resolution = kResolution;
    env.cv_map = std::make_unique<test_utils::Cv2Map>(kMapImagePath, opt);
    const grid_map::GridMap& base_gm = env.cv_map->grid_map();
    const grid_map::RowMatrixXi cv_occ = extract_occupancy_matrix(base_gm);

    // 构造两份几何/占据一致但 esdf_revision 不同的快照（用于 JPS 缓存命中与重建对比）
    auto gm_a_mut = std::make_shared<grid_map::GridMap>(base_gm);
    auto gm_b_mut = std::make_shared<grid_map::GridMap>(base_gm);
    grid_map::GridMap::EsdfWorkspace init_ws;
    gm_b_mut->overwriteMapAndUpdateEsdf(cv_occ, &init_ws);
    env.gm_a = gm_a_mut;
    env.gm_b = gm_b_mut;
    env.esdf = std::make_unique<minco_opt::GridMapESDF>(env.gm_a);

    // 2. 初始化生产配置 MaMap，并将 rmuc2024 障碍嵌入 560x400 占据矩阵
    {
        // ROGMap 构造期会通过 printf/std::cout 打印参数表，局部静默以保证基准输出与 JSON 格式干净。
        ScopedSilenceStdout silence;
        env.ma_map = std::make_unique<ma_map::MaMap>(kMapConfigPath);
    }
    const auto initial_snap = env.ma_map->get_grid_map();
    if (!initial_snap) {
        throw std::runtime_error("MaMap initial snapshot is null");
    }
    const Eigen::Vector2i ma_n = initial_snap->getVoxelNum();
    env.ma_occ_a = grid_map::RowMatrixXi::Zero(ma_n.x(), ma_n.y());
    const int copy_rows = std::min(static_cast<int>(cv_occ.rows()), ma_n.x());
    const int copy_cols = std::min(static_cast<int>(cv_occ.cols()), ma_n.y());
    for (int x = 0; x < copy_rows; ++x) {
        for (int y = 0; y < copy_cols; ++y) {
            env.ma_occ_a(x, y) = cv_occ(x, y);
        }
    }
    env.ma_occ_b = env.ma_occ_a;
    // 在中心位置翻转单格，使 PublishChanged 每帧都触发真实 ESDF 更新与池槽位切换
    const int flip_x = ma_n.x() / 2;
    const int flip_y = ma_n.y() / 2;
    env.ma_occ_b(flip_x, flip_y) = env.ma_occ_a(flip_x, flip_y) == 0 ? 1 : 0;

    // 语义多边形位于 config/map.yaml 覆盖域 [-6, 22] x [-10, 10] 内
    env.sem_polygon = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};

    // GridMap 单独微基准对象（与 MaMap 同尺寸 560x400）
    const Eigen::Vector2d origin = initial_snap->getOrigin();
    const Eigen::Vector2d size = initial_snap->getMapSize();
    const double res = initial_snap->getResolution();
    env.grid_src.init(-origin.x(), origin.x() + size.x(), -origin.y(), origin.y() + size.y(), res);
    env.grid_dst.init(-origin.x(), origin.x() + size.x(), -origin.y(), origin.y() + size.y(), res);
    env.grid_src.overwriteMapAndUpdateEsdf(env.ma_occ_a, &init_ws);
    env.grid_dst.overwriteMapAndUpdateEsdf(env.ma_occ_b, &init_ws);

    // 3. 固定随机种子采样可达规划查询，并预构建轨迹优化输入
    const double safe = env.planner_cfg.path_planning_params.safe_threshold;
    std::vector<Eigen::Vector2d> free_cells;
    for (int row = 0; row < env.cv_map->height(); ++row) {
        for (int col = 0; col < env.cv_map->width(); ++col) {
            const Eigen::Vector2d p = env.cv_map->pixel_to_world(col, row);
            if (env.gm_a->getDistance(p) >= safe + kResolution) {
                free_cells.push_back(p);
            }
        }
    }
    if (free_cells.empty()) {
        throw std::runtime_error("no collision-free cells in " + kMapImagePath);
    }

    path_planning::PathPlanning frontend;
    frontend.set_param(env.planner_cfg.path_planning_params);
    frontend.set_use_jps(true);
    frontend.set_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    std::mt19937 rng(kSeed);
    std::uniform_int_distribution<size_t> pick(0, free_cells.size() - 1);
    for (int attempt = 0; attempt < kNumQueries * 200 && static_cast<int>(env.queries.size()) < kNumQueries; ++attempt)
    {
        const Query q {free_cells[pick(rng)], free_cells[pick(rng)]};
        if ((q.goal - q.start).norm() < kMinQueryDist) continue;
        auto traj = frontend.path_planning(*env.gm_a, q.start, q.goal, 0.0, 0.0, kTimeoutMs);
        if (!traj.has_value() || traj->timed_trajectory.size() < 2) continue;

        auto opt_in = ma_spline_opt::from_path_planning_trajectory(*traj);
        opt_in.model = ma_spline_opt::OptModel::OMNI_XY;
        opt_in.start_vel = Eigen::Vector2d::Zero();
        opt_in.start_acc = Eigen::Vector2d::Zero();

        env.queries.push_back(q);
        env.opt_inputs.push_back(std::move(opt_in));
    }
    if (static_cast<int>(env.queries.size()) < kNumQueries) {
        throw std::runtime_error("failed to sample enough reachable queries");
    }

    // 4. 为 FsmReplan 选取共用起点的双目标 (goal_a, goal_b)，确保交替调用必触发完整重规划且均成功
    const Eigen::Vector2d fsm_start_xy = env.queries[0].start;
    const Eigen::Vector2d fsm_goal_a_xy = env.queries[0].goal;
    Eigen::Vector2d fsm_goal_b_xy = Eigen::Vector2d::Zero();
    bool found_alt_goal = false;
    for (int attempt = 0; attempt < 500; ++attempt) {
        const Eigen::Vector2d cand = free_cells[pick(rng)];
        if ((cand - fsm_start_xy).norm() < kMinQueryDist) continue;
        if ((cand - fsm_goal_a_xy).norm() < kMinAltGoalDist) continue;
        auto traj = frontend.path_planning(*env.gm_a, fsm_start_xy, cand, 0.0, 0.0, kTimeoutMs);
        if (traj.has_value() && traj->timed_trajectory.size() >= 2) {
            fsm_goal_b_xy = cand;
            found_alt_goal = true;
            break;
        }
    }
    if (!found_alt_goal) {
        throw std::runtime_error("failed to find alternate goal for FsmReplan benchmark");
    }
    env.fsm_start = make_robot_state(fsm_start_xy);
    env.fsm_goal_a = make_robot_state(fsm_goal_a_xy);
    env.fsm_goal_b = make_robot_state(fsm_goal_b_xy);

    // 5. 生成 MPC 基准轨迹及沿轨扰动采样点
    replan::FsmReplan seed_fsm(env.planner_cfg);
    auto seed_res = seed_fsm.plan(env.fsm_goal_a, env.fsm_start, env.gm_a);
    if (!seed_res.has_value() || !seed_res->ma_spline_traj.success) {
        throw std::runtime_error("failed to generate reference trajectory for MPC benchmark");
    }
    env.mpc_traj = std::make_shared<control::MaSplineTrajectoryInterface>(seed_res->ma_spline_traj);
    const double dur = env.mpc_traj->duration();
    if (dur <= 0.1) {
        throw std::runtime_error("MPC reference trajectory duration too short");
    }
    for (int i = 0; i < kMpcSamples; ++i) {
        const double t_now = dur * 0.8 * static_cast<double>(i) / static_cast<double>(kMpcSamples);
        control::TrajectoryTypes::ReferencePoint ref;
        if (!env.mpc_traj->sample(t_now, ref)) {
            throw std::runtime_error("MPC trajectory sample failed");
        }
        MpcSample s;
        s.t_now = t_now;
        s.x0 = ref.state;
        s.x0(0) += (i % 2 == 0) ? 0.05 : -0.05;
        s.x0(1) += (i % 2 == 0) ? -0.04 : 0.04;
        env.mpc_samples.push_back(s);
    }

    return env;
}

// ---------------- 基准前正确性自检 ----------------

auto check(bool cond, const char* msg) -> bool {
    if (!cond) {
        std::fprintf(stderr, "[SANITY FAIL] %s\n", msg);
        return false;
    }
    return true;
}

auto run_sanity_checks(BenchEnv& env) -> bool {
    bool ok = true;

    // 1. GridMap: ESDF 工作区等价性、脏检查与快照复用
    {
        grid_map::GridMap ws_map;
        grid_map::GridMap raw_map;
        const Eigen::Vector2d origin = env.grid_src.getOrigin();
        const Eigen::Vector2d size = env.grid_src.getMapSize();
        const double res = env.grid_src.getResolution();
        ws_map.init(-origin.x(), origin.x() + size.x(), -origin.y(), origin.y() + size.y(), res);
        raw_map.init(-origin.x(), origin.x() + size.x(), -origin.y(), origin.y() + size.y(), res);

        grid_map::GridMap::EsdfWorkspace ws;
        ok &= check(ws_map.overwriteMapAndUpdateEsdf(env.ma_occ_a, &ws), "GridMap overwriteMapAndUpdateEsdf(ws)");
        ok &= check(raw_map.overwriteMapAndUpdateEsdf(env.ma_occ_a, nullptr), "GridMap overwriteMapAndUpdateEsdf(raw)");
        ok &= check(ws_map.equalsOccupancy(env.ma_occ_a), "GridMap equalsOccupancy true on identical occ");
        ok &= check(!ws_map.equalsOccupancy(env.ma_occ_b), "GridMap equalsOccupancy false on modified occ");

        const auto esdf_ws = ws_map.esdfData();
        const auto esdf_raw = raw_map.esdfData();
        ok &= check(esdf_ws.size() == esdf_raw.size() && !esdf_ws.empty(), "GridMap esdfData size match");
        double max_diff = 0.0;
        for (size_t i = 0; i < esdf_ws.size(); ++i) {
            max_diff = std::max(max_diff, std::abs(esdf_ws[i] - esdf_raw[i]));
        }
        ok &= check(max_diff < 1e-9, "GridMap ESDF workspace vs raw bit-consistent");

        grid_map::GridMap reused;
        reused.init(-origin.x(), origin.x() + size.x(), -origin.y(), origin.y() + size.y(), res);
        const uint64_t rev_before = reused.revision();
        ok &= check(reused.copyOccAndEsdfFrom(ws_map), "GridMap copyOccAndEsdfFrom");
        ok &= check(
            reused.esdf_revision() == ws_map.esdf_revision(),
            "GridMap copyOccAndEsdfFrom preserves esdf_revision"
        );
        ok &= check(reused.revision() > rev_before, "GridMap copyOccAndEsdfFrom increments revision");
    }

    // 2. MaMap: 快照发布、脏检查跳过、语义更新不重算 ESDF、旧快照不可变
    {
        env.ma_map->set_global_map(env.ma_occ_a);
        const auto snap1 = env.ma_map->get_grid_map();
        ok &= check(snap1 != nullptr, "MaMap snapshot non-null after set_global_map");
        if (snap1) {
            const uint64_t rev1 = snap1->revision();
            const uint64_t esdf_rev1 = snap1->esdf_revision();

            env.ma_map->set_global_map(env.ma_occ_a);
            const auto snap1_same = env.ma_map->get_grid_map();
            ok &= check(
                snap1_same.get() == snap1.get() && snap1_same->revision() == rev1,
                "MaMap unchanged occupancy skips republish"
            );

            env.ma_map->set_global_map(env.ma_occ_b);
            const auto snap2 = env.ma_map->get_grid_map();
            ok &= check(
                snap2 != nullptr && snap2.get() != snap1.get() && snap2->revision() > rev1
                    && snap2->esdf_revision() != esdf_rev1,
                "MaMap changed occupancy publishes new snapshot with new esdf_revision"
            );
            ok &= check(snap1->equalsOccupancy(env.ma_occ_a), "MaMap retained old snapshot remains immutable");

            const uint64_t rev2 = snap2->revision();
            const uint64_t esdf_rev2 = snap2->esdf_revision();
            env.ma_map->set_semantics_polygon(env.sem_polygon, grid_map::GridMap::Semantics::TUNNEL);
            const auto snap3 = env.ma_map->get_grid_map();
            ok &= check(
                snap3 != nullptr && snap3->revision() > rev2 && snap3->esdf_revision() == esdf_rev2,
                "MaMap set_semantics_polygon increments revision and reuses esdf_revision"
            );
            env.ma_map->set_semantics_polygon(env.sem_polygon, grid_map::GridMap::Semantics::NONE);
        }
    }

    // 3. PathPlanning: A* / JPS / Frontend 正确性与跨快照缓存切换
    {
        path_planning::AStar astar;
        path_planning::JPS jps;
        astar.set_safe_threshold(env.planner_cfg.path_planning_params.safe_threshold);
        jps.set_safe_threshold(env.planner_cfg.path_planning_params.safe_threshold);

        path_planning::PathPlanning frontend;
        frontend.set_param(env.planner_cfg.path_planning_params);
        frontend.set_use_jps(true);
        frontend.set_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        for (size_t i = 0; i < env.queries.size(); ++i) {
            const auto& q = env.queries[i];
            const auto pa = astar.original_astar_search(*env.gm_a, q.start, q.goal, kTimeoutMs);
            const auto pj_a = jps.jps_search(*env.gm_a, q.start, q.goal, kTimeoutMs);
            const auto pj_b = jps.jps_search(*env.gm_b, q.start, q.goal, kTimeoutMs);
            const auto pf = frontend.path_planning(*env.gm_a, q.start, q.goal, 0.0, 0.0, kTimeoutMs);
            ok &= check(pa.size() >= 2, "AStar valid path");
            ok &= check(pj_a.size() >= 2 && pj_b.size() >= 2, "JPS valid path across snapshots");
            ok &= check(
                pf.has_value() && pf->total_time > 0.0 && !pf->optimized_path.empty(),
                "PathPlanning frontend valid trajectory"
            );
            if (!ok) break;
        }
    }

    // 4. TrajOptimize: 作用域 ESDF 优化成功且不穿墙
    {
        ma_spline_opt::MaSplineTrajectoryOptimizer<minco_opt::GridMapESDF> opt;
        opt.set_config(env.planner_cfg.ma_spline_opt_params);
        for (const auto& in: env.opt_inputs) {
            const auto out = opt.optimize(in, *env.esdf);
            ok &= check(
                out.success && out.trajectory.isInitialized() && std::isfinite(out.cost),
                "TrajOptimize success with finite cost"
            );
            const double dur = ma_spline_opt::total_duration(out);
            ok &= check(dur > 0.0, "TrajOptimize positive duration");
            if (ok) {
                double min_esdf = std::numeric_limits<double>::max();
                const int steps = std::max(1, static_cast<int>(std::ceil(dur / 0.05)));
                for (int k = 0; k <= steps; ++k) {
                    const double t = std::min(dur, k * 0.05);
                    Eigen::Vector2d p, v, a;
                    ma_spline_opt::sample_xy(out, t, p, v, a);
                    if (env.esdf->isInside(p.x(), p.y())) {
                        min_esdf = std::min(min_esdf, env.esdf->getDistance(p.x(), p.y()));
                    }
                }
                ok &= check(min_esdf >= -1e-3, "TrajOptimize collision-free (min_esdf >= 0)");
            }
            if (!ok) break;
        }
    }

    // 5. FsmReplan: 空快照保护、监控快路径、目标切换重规划、失败保留旧轨迹
    {
        replan::FsmReplan fsm(env.planner_cfg);

        const auto null_init = fsm.plan(env.fsm_goal_a, env.fsm_start, nullptr);
        ok &= check(
            !null_init.has_value() && null_init.error() == replan::FsmReplan::PathError::PLANNING_FAILED,
            "FsmReplan returns PLANNING_FAILED on null snapshot"
        );

        const auto first = fsm.plan(env.fsm_goal_a, env.fsm_start, env.gm_a);
        ok &= check(
            first.has_value() && first->is_new_trajectory && first->ma_spline_traj.success,
            "FsmReplan initial plan produces new trajectory"
        );

        const auto monitor = fsm.plan(env.fsm_goal_a, env.fsm_start, env.gm_a);
        ok &= check(
            monitor.has_value() && !monitor->is_new_trajectory,
            "FsmReplan steady state hits monitoring fast path"
        );

        const auto null_after = fsm.plan(env.fsm_goal_a, env.fsm_start, nullptr);
        ok &= check(
            !null_after.has_value() && null_after.error() == replan::FsmReplan::PathError::PLANNING_FAILED,
            "FsmReplan rejects null snapshot after valid plan"
        );

        const auto still_monitor = fsm.plan(env.fsm_goal_a, env.fsm_start, env.gm_a);
        ok &= check(
            still_monitor.has_value() && !still_monitor->is_new_trajectory,
            "FsmReplan preserves committed trajectory after null failure"
        );

        const auto replan_b = fsm.plan(env.fsm_goal_b, env.fsm_start, env.gm_a);
        ok &= check(
            replan_b.has_value() && replan_b->is_new_trajectory && replan_b->ma_spline_traj.success,
            "FsmReplan replans on goal_b"
        );

        const auto replan_a = fsm.plan(env.fsm_goal_a, env.fsm_start, env.gm_a);
        ok &= check(
            replan_a.has_value() && replan_a->is_new_trajectory && replan_a->ma_spline_traj.success,
            "FsmReplan replans back on goal_a"
        );
    }

    // 6. Controller: 线性 MPC 求解成功且控制量有限、在限幅内
    {
        control::Mpc<control::MaSplineTrajectoryInterface> mpc(env.mpc_param);
        mpc.set_trajectory(env.mpc_traj);
        for (const auto& s: env.mpc_samples) {
            control::InputVector u_cmd = control::InputVector::Zero();
            const bool solved = mpc.solve(s.x0, s.t_now, u_cmd);
            const bool finite = std::isfinite(u_cmd(0)) && std::isfinite(u_cmd(1));
            const bool bounded = u_cmd(0) >= env.mpc_param.u_min(0) - 1e-3 && u_cmd(0) <= env.mpc_param.u_max(0) + 1e-3
                && u_cmd(1) >= env.mpc_param.u_min(1) - 1e-3 && u_cmd(1) <= env.mpc_param.u_max(1) + 1e-3;
            ok &= check(solved && finite && bounded, "MPC solve valid and within bounds");
            if (!ok) break;
        }
    }

    // 7. 双模式地图架构自检：GridGeometry / Option A 虚拟墙 / MapIO / LayeredMap 动态扩图 / 导航越界闭环
    {
        // 7a. GridGeometry 半开区间与 GridMap Option A 四边界虚拟墙
        const auto geom =
            grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(-2.0, -2.0), Eigen::Vector2i(40, 40), 0.1);
        ok &= check(geom.isValid(), "GridGeometry valid");
        ok &= check(geom.isInside(Eigen::Vector2d(-2.0, -2.0)), "GridGeometry min_boundary inside");
        ok &= check(!geom.isInside(Eigen::Vector2d(2.0, 0.0)), "GridGeometry max_boundary X outside");
        ok &= check(!geom.isInside(Eigen::Vector2d(0.0, 2.0)), "GridGeometry max_boundary Y outside");
        ok &= check(
            !geom.isInside(Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0)),
            "GridGeometry NaN outside"
        );
        ok &= check(
            geom.posToIndex(Eigen::Vector2d(-2.05, -2.05)) == Eigen::Vector2i(-1, -1),
            "GridGeometry floor negative"
        );

        grid_map::GridMap vwall;
        vwall.init(geom, grid_map::OutOfBoundsPolicy::FREE);
        const grid_map::RowMatrixXi all_free = grid_map::RowMatrixXi::Zero(40, 40);
        ok &= check(vwall.overwriteMapAndUpdateEsdf(all_free), "GridMap all-free ESDF");
        ok &= check(
            std::isfinite(vwall.getDistance(Eigen::Vector2d(0.0, 0.0)))
                && vwall.getDistance(Eigen::Vector2d(0.0, 0.0)) > 4.0,
            "GridMap all-free short-circuit finite positive"
        );
        ok &= check(!vwall.isCollision(Eigen::Vector2d(3.0, 0.0), 0.3), "GridMap FREE policy OOB non-collision");

        const uint64_t rev_free = vwall.esdf_revision();
        vwall.setOutOfBoundsPolicy(grid_map::OutOfBoundsPolicy::OCCUPIED);
        ok &= check(vwall.esdf_revision() != rev_free, "GridMap setOutOfBoundsPolicy bumps esdf_revision");

        // 图内四边界虚拟墙距离与向内单位法向
        double d_edge = 0.0;
        Eigen::Vector2d g_edge = Eigen::Vector2d::Zero();
        ok &= check(
            vwall.getDistanceAndGradient(Eigen::Vector2d(-1.85, 0.0), d_edge, g_edge) && std::abs(d_edge - 0.15) < 1e-6
                && (g_edge - Eigen::Vector2d(1.0, 0.0)).norm() < 1e-6,
            "GridMap OCCUPIED left virtual wall distance & normal"
        );
        ok &= check(
            vwall.getDistanceAndGradient(Eigen::Vector2d(1.85, 0.0), d_edge, g_edge) && std::abs(d_edge - 0.15) < 1e-6
                && (g_edge - Eigen::Vector2d(-1.0, 0.0)).norm() < 1e-6,
            "GridMap OCCUPIED right virtual wall distance & normal"
        );
        ok &= check(
            vwall.getDistanceAndGradient(Eigen::Vector2d(0.0, -1.85), d_edge, g_edge) && std::abs(d_edge - 0.15) < 1e-6
                && (g_edge - Eigen::Vector2d(0.0, 1.0)).norm() < 1e-6,
            "GridMap OCCUPIED bottom virtual wall distance & normal"
        );
        ok &= check(
            vwall.getDistanceAndGradient(Eigen::Vector2d(0.0, 1.85), d_edge, g_edge) && std::abs(d_edge - 0.15) < 1e-6
                && (g_edge - Eigen::Vector2d(0.0, -1.0)).norm() < 1e-6,
            "GridMap OCCUPIED top virtual wall distance & normal"
        );

        // 图外（边外、角外、精确上边界）负距离与向内单位法向
        ok &= check(vwall.isCollision(Eigen::Vector2d(2.3, 0.0), 0.3), "GridMap OCCUPIED OOB collision");
        ok &= check(
            vwall.getDistanceAndGradient(Eigen::Vector2d(2.3, 0.0), d_edge, g_edge) && std::abs(d_edge - (-0.3)) < 1e-6
                && (g_edge - Eigen::Vector2d(-1.0, 0.0)).norm() < 1e-6,
            "GridMap OCCUPIED edge OOB negative distance & inward normal"
        );
        ok &= check(
            vwall.getDistanceAndGradient(Eigen::Vector2d(2.3, 2.4), d_edge, g_edge) && std::abs(d_edge - (-0.5)) < 1e-6
                && (g_edge - Eigen::Vector2d(-0.6, -0.8)).norm() < 1e-6,
            "GridMap OCCUPIED corner OOB negative distance & inward normal"
        );
        ok &= check(
            vwall.getDistanceAndGradient(Eigen::Vector2d(2.0, 0.0), d_edge, g_edge) && std::abs(d_edge) < 1e-6
                && (g_edge - Eigen::Vector2d(-1.0, 0.0)).norm() < 1e-6,
            "GridMap OCCUPIED exact max_boundary distance=0 & inward normal"
        );

        const grid_map::RowMatrixXi all_occ = grid_map::RowMatrixXi::Ones(40, 40);
        ok &= check(vwall.overwriteMapAndUpdateEsdf(all_occ), "GridMap all-occupied ESDF");
        ok &= check(
            std::isfinite(vwall.getDistance(Eigen::Vector2d(0.0, 0.0)))
                && vwall.getDistance(Eigen::Vector2d(0.0, 0.0)) < -4.0,
            "GridMap all-occupied short-circuit finite negative"
        );

        // 7b. MapIO: 既有 global_map.pgm 兼容加载 + 三态裁剪/针孔清理/往返精度（仅写系统临时目录并自动清理）
        const std::string legacy_map_path = kRootDir + "map/global_map.pgm";
        if (std::filesystem::exists(legacy_map_path)) {
            const auto legacy = ma_map::MapIO::load(legacy_map_path);
            ok &= check(
                legacy.has_value() && legacy->geometry.voxel_num == Eigen::Vector2i(560, 320)
                    && std::abs(legacy->geometry.resolution - 0.05) < 1e-9
                    && (legacy->geometry.origin - Eigen::Vector2d(-6.0, -8.0)).norm() < 1e-9
                    && legacy->occupied_count > 0 && legacy->free_count > 0,
                "MapIO loads legacy map/global_map.pgm with exact geometry"
            );
        }

        const std::filesystem::path tmp_dir =
            std::filesystem::temp_directory_path() / ("ma_nav_sanity_" + std::to_string(::getpid()));
        struct TempDirGuard {
            std::filesystem::path dir;
            ~TempDirGuard() {
                std::error_code ec;
                std::filesystem::remove_all(dir, ec);
            }
        } tmp_guard {tmp_dir};

        const auto io_geom =
            grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(-2.0, -1.5), Eigen::Vector2i(20, 16), 0.1);
        std::vector<ma_map::CellState> io_cells(io_geom.cellCount(), ma_map::CellState::UNKNOWN);
        // 拒绝保存全 UNKNOWN 图
        const auto empty_save = ma_map::MapIO::save((tmp_dir / "empty").string(), io_geom, io_cells);
        ok &= check(!empty_save.has_value(), "MapIO rejects saving completely unexplored map");

        // 构造局部已探索块 x in [5..12], y in [4..10]
        for (int x = 5; x <= 12; ++x) {
            for (int y = 4; y <= 10; ++y) {
                io_cells[static_cast<size_t>(io_geom.toAddress(x, y))] = ma_map::CellState::FREE;
            }
        }
        // 一个障碍格 + 紧邻障碍的 UNKNOWN（不可被清理）
        io_cells[static_cast<size_t>(io_geom.toAddress(6, 5))] = ma_map::CellState::OCCUPIED;
        io_cells[static_cast<size_t>(io_geom.toAddress(6, 6))] = ma_map::CellState::UNKNOWN;
        // 两个完全被 FREE 包围的内部小 UNKNOWN 针孔（应当被清理）
        io_cells[static_cast<size_t>(io_geom.toAddress(9, 7))] = ma_map::CellState::UNKNOWN;
        io_cells[static_cast<size_t>(io_geom.toAddress(9, 8))] = ma_map::CellState::UNKNOWN;

        ma_map::SaveOptions save_opts;
        save_opts.crop_to_explored = true;
        save_opts.margin_cells = 1;
        save_opts.cleanup_free_pinholes = true;
        save_opts.max_pinhole_cells = 4;
        const auto saved = ma_map::MapIO::save((tmp_dir / "crop_test").string(), io_geom, io_cells, save_opts);
        ok &= check(saved.has_value(), "MapIO cropped trinary save succeeds");
        if (saved) {
            ok &= check(
                saved->saved_geometry.voxel_num == Eigen::Vector2i(10, 9),
                "MapIO cropped dimensions match explored bbox + margin"
            );
            ok &= check(saved->cleaned_pinholes == 2, "MapIO cleaned only FREE-surrounded pinholes");

            const auto reload_occ = ma_map::MapIO::load(saved->pgm_path, ma_map::LoadOptions {true});
            const auto reload_free = ma_map::MapIO::load(saved->pgm_path, ma_map::LoadOptions {false});
            ok &= check(reload_occ.has_value() && reload_free.has_value(), "MapIO reload succeeds");
            if (reload_occ && reload_free) {
                ok &= check(
                    reload_occ->geometry.voxel_num == saved->saved_geometry.voxel_num
                        && (reload_occ->geometry.origin - saved->saved_geometry.origin).norm() < 1e-9,
                    "MapIO round-trip preserves exact geometry and origin"
                );
                const Eigen::Vector2d obs_world = io_geom.indexToPos(Eigen::Vector2i(6, 5));
                const Eigen::Vector2d adj_unk_world = io_geom.indexToPos(Eigen::Vector2i(6, 6));
                const Eigen::Vector2d pinhole_world = io_geom.indexToPos(Eigen::Vector2i(9, 7));
                const auto obs_id = reload_occ->geometry.posToIndex(obs_world);
                const auto unk_id = reload_occ->geometry.posToIndex(adj_unk_world);
                const auto pin_id = reload_occ->geometry.posToIndex(pinhole_world);
                ok &= check(
                    reload_occ->cells[static_cast<size_t>(reload_occ->geometry.toAddress(obs_id))]
                        == ma_map::CellState::OCCUPIED,
                    "MapIO round-trip preserves OCCUPIED world cell"
                );
                ok &= check(
                    reload_occ->cells[static_cast<size_t>(reload_occ->geometry.toAddress(unk_id))]
                            == ma_map::CellState::UNKNOWN
                        && reload_occ->occupancy[static_cast<size_t>(reload_occ->geometry.toAddress(unk_id))] == 1
                        && reload_free->occupancy[static_cast<size_t>(reload_free->geometry.toAddress(unk_id))] == 0,
                    "MapIO unknown_as_occupied controls UNKNOWN occupancy"
                );
                ok &= check(
                    reload_occ->cells[static_cast<size_t>(reload_occ->geometry.toAddress(pin_id))]
                        == ma_map::CellState::FREE,
                    "MapIO pinhole cell restored as FREE"
                );
            }
        }

        // 7c. LayeredMap: 双模式、动态扩图（细胞中心保持 + 语义重光栅化 + 上限拦截）、持久化往返与导航固定边界
        ma_map::LayeredMap::Config lm_cfg;
        lm_cfg.x_left = 1.0;
        lm_cfg.x_right = 1.0;
        lm_cfg.y_left = 1.0;
        lm_cfg.y_right = 1.0;
        lm_cfg.resolution = 0.1;
        lm_cfg.expand_chunk_m = 1.0;
        lm_cfg.max_map_size_x = 5.0;
        lm_cfg.max_map_size_y = 5.0;
        lm_cfg.save_margin_cells = 1;
        lm_cfg.unknown_as_occupied = true;

        ma_map::LayeredMap lm(lm_cfg);
        ok &= check(lm.mode() == ma_map::MapMode::MAPPING, "LayeredMap starts in MAPPING mode");
        const auto s0 = lm.getSnapshot();
        ok &= check(
            s0 && s0->outOfBoundsPolicy() == grid_map::OutOfBoundsPolicy::FREE
                && s0->getVoxelNum() == Eigen::Vector2i(20, 20),
            "LayeredMap initial snapshot FREE policy and 20x20"
        );

        const std::vector<Eigen::Vector2d> tunnel_poly = {
            Eigen::Vector2d(0.1, 0.1),
            Eigen::Vector2d(0.6, 0.1),
            Eigen::Vector2d(0.6, 0.6),
            Eigen::Vector2d(0.1, 0.6),
        };
        ok &= check(
            lm.setSemanticsPolygon(tunnel_poly, grid_map::GridMap::Semantics::TUNNEL),
            "LayeredMap setSemanticsPolygon succeeds"
        );

        // 同帧 obstacle + free 重叠时障碍优先；连续 2 帧 free 才清除旧障碍
        ma_map::LocalTerrainFrame fr1;
        fr1.terrain_resolution = 0.1;
        fr1.has_robot_pos = true;
        fr1.robot_pos = Eigen::Vector2d(0.0, 0.0);
        fr1.robot_footprint_radius = 0.2;
        fr1.obstacle_cells.emplace_back(0.45, 0.45, 0.0);
        fr1.obstacle_cells.emplace_back(-0.45, -0.45, 0.0);
        fr1.free_cells.emplace_back(0.45, 0.45, 0.0);
        fr1.free_cells.emplace_back(0.25, 0.25, 0.0);
        lm.updateFromPerception(fr1);
        auto s1 = lm.getSnapshot();
        Eigen::Vector2i id_obs1;
        s1->posToIndex(Eigen::Vector2d(0.45, 0.45), id_obs1);
        ok &= check(s1->isOccupied(id_obs1), "LayeredMap obstacle prioritized over same-frame free");

        ma_map::LocalTerrainFrame fr2;
        fr2.terrain_resolution = 0.1;
        fr2.free_cells.emplace_back(0.45, 0.45, 0.0);
        lm.updateFromPerception(fr2);
        s1 = lm.getSnapshot();
        s1->posToIndex(Eigen::Vector2d(0.45, 0.45), id_obs1);
        ok &= check(s1->isOccupied(id_obs1), "LayeredMap 1 free frame keeps persistent obstacle");

        lm.updateFromPerception(fr2);
        s1 = lm.getSnapshot();
        s1->posToIndex(Eigen::Vector2d(0.45, 0.45), id_obs1);
        ok &= check(!s1->isOccupied(id_obs1), "LayeredMap 2 consecutive free frames clear persistent obstacle");

        // authoritative_free=true 单帧清障；cleared_cells 仅把旧 OCCUPIED 退回 UNKNOWN（不覆写 FREE/同帧障碍）
        ma_map::LocalTerrainFrame fr_auth_set;
        fr_auth_set.terrain_resolution = 0.1;
        fr_auth_set.obstacle_cells.emplace_back(0.55f, 0.45f, 0.0f);
        fr_auth_set.obstacle_cells.emplace_back(0.65f, 0.45f, 0.0f);
        fr_auth_set.free_cells.emplace_back(0.75f, 0.45f, 0.0f);
        lm.updateFromPerception(fr_auth_set);

        ma_map::LocalTerrainFrame fr_auth_clr;
        fr_auth_clr.terrain_resolution = 0.1;
        fr_auth_clr.authoritative_free = true;
        fr_auth_clr.free_cells.emplace_back(0.55f, 0.45f, 0.0f);
        fr_auth_clr.cleared_cells.emplace_back(0.65f, 0.45f, 0.0f);
        fr_auth_clr.cleared_cells.emplace_back(0.75f, 0.45f, 0.0f);
        lm.updateFromPerception(fr_auth_clr);
        s1 = lm.getSnapshot();
        Eigen::Vector2i id_af, id_cc, id_keep_free;
        s1->posToIndex(Eigen::Vector2d(0.55, 0.45), id_af);
        s1->posToIndex(Eigen::Vector2d(0.65, 0.45), id_cc);
        s1->posToIndex(Eigen::Vector2d(0.75, 0.45), id_keep_free);
        ok &= check(
            !s1->isOccupied(id_af) && !s1->isOccupied(id_cc) && !s1->isOccupied(id_keep_free),
            "LayeredMap authoritative_free clears OCC in 1 frame and cleared_cells resets stale OCC"
        );

        // 感知触发动态扩图（超出 [-1, 1] -> 按 1.0m 块扩展），保留旧障碍与语义多边形
        ma_map::LocalTerrainFrame fr_exp;
        fr_exp.terrain_resolution = 0.1;
        fr_exp.obstacle_cells.emplace_back(1.45, 0.25, 0.0);
        fr_exp.free_cells.emplace_back(1.25, 0.25, 0.0);
        lm.updateFromPerception(fr_exp);
        const auto s_exp = lm.getSnapshot();
        ok &= check(
            s_exp->getVoxelNum().x() >= 30 && s_exp->isInsideMap(Eigen::Vector2d(1.45, 0.25)),
            "LayeredMap perception dynamically expands map"
        );
        Eigen::Vector2i id_old, id_new;
        s_exp->posToIndex(Eigen::Vector2d(-0.45, -0.45), id_old);
        s_exp->posToIndex(Eigen::Vector2d(1.45, 0.25), id_new);
        ok &= check(
            s_exp->isOccupied(id_old) && s_exp->isOccupied(id_new),
            "LayeredMap expansion preserves old & new cells"
        );
        ok &= check(
            s_exp->is_tunnel(Eigen::Vector2d(0.35, 0.35)),
            "LayeredMap expansion re-rasterizes semantics polygon"
        );

        // ensureCovers：上限内扩图成功，超 max_map_size 或 NaN 拒收且不改图几何
        ok &=
            check(lm.ensureCovers(Eigen::Vector2d(-1.5, -1.5)), "LayeredMap ensureCovers expands within max_map_size");
        const auto geom_before_reject = lm.getSnapshot()->geometry();
        ok &=
            check(!lm.ensureCovers(Eigen::Vector2d(100.0, 0.0)), "LayeredMap ensureCovers rejects beyond max_map_size");
        ok &= check(
            !lm.ensureCovers(Eigen::Vector2d(std::numeric_limits<double>::infinity(), 0.0)),
            "LayeredMap ensureCovers rejects non-finite point"
        );
        ok &= check(
            lm.getSnapshot()->geometry() == geom_before_reject,
            "LayeredMap rejected ensureCovers leaves geometry unchanged"
        );

        // 导出已探索地图并切换导航模式加载：严格固定边界 + OCCUPIED 策略 + 动态层逐帧刷新
        const auto lm_saved = lm.saveExploredMap((tmp_dir / "layered_saved").string());
        ok &= check(lm_saved.has_value(), "LayeredMap saveExploredMap succeeds");
        if (lm_saved) {
            lm.setMode(ma_map::MapMode::NAVIGATION);
            const auto lm_loaded = lm.loadStaticMap(lm_saved->pgm_path);
            ok &= check(lm_loaded.has_value(), "LayeredMap loadStaticMap succeeds");
            const auto s_nav = lm.getSnapshot();
            ok &= check(
                s_nav && s_nav->outOfBoundsPolicy() == grid_map::OutOfBoundsPolicy::OCCUPIED
                    && s_nav->geometry().voxel_num == lm_saved->saved_geometry.voxel_num
                    && std::abs(s_nav->geometry().resolution - lm_saved->saved_geometry.resolution) < 1e-9
                    && (s_nav->geometry().origin - lm_saved->saved_geometry.origin).norm() < 1e-9,
                "LayeredMap nav mode inherits exact loaded geometry with OCCUPIED policy"
            );
            ok &= check(
                !lm.ensureCovers(Eigen::Vector2d(50.0, 50.0)) && lm.getSnapshot()->geometry() == s_nav->geometry(),
                "LayeredMap nav mode never expands"
            );
            ma_map::LocalTerrainFrame dyn1;
            dyn1.terrain_resolution = 0.1;
            dyn1.obstacle_cells.emplace_back(0.25, 0.25, 0.0);
            lm.updateFromPerception(dyn1);
            const auto s_dyn1 = lm.getSnapshot();
            Eigen::Vector2i dyn_id;
            s_dyn1->posToIndex(Eigen::Vector2d(0.25, 0.25), dyn_id);
            ok &= check(s_dyn1->isOccupied(dyn_id), "LayeredMap nav mode overlays dynamic obstacle");

            ma_map::LocalTerrainFrame dyn2;
            dyn2.terrain_resolution = 0.1;
            lm.updateFromPerception(dyn2);
            const auto s_dyn2 = lm.getSnapshot();
            s_dyn2->posToIndex(Eigen::Vector2d(0.25, 0.25), dyn_id);
            ok &=
                check(!s_dyn2->isOccupied(dyn_id), "LayeredMap nav mode clears vanished dynamic obstacle immediately");
        }

        // 7d. 导航模式越界闭环：A*/JPS/PostProc/GridMapESDF/FsmReplan（OOB 目标拒收保旧轨 + 起点轻微越界向内恢复）
        auto nav_gm = std::make_shared<grid_map::GridMap>();
        nav_gm->init(3.0, 3.0, 3.0, 3.0, 0.05, grid_map::OutOfBoundsPolicy::OCCUPIED);
        grid_map::RowMatrixXi nav_occ = grid_map::RowMatrixXi::Zero(120, 120);
        // 在中心放一块障碍，四周保留开阔走廊
        for (int x = 55; x <= 64; ++x) {
            for (int y = 55; y <= 64; ++y) {
                nav_occ(x, y) = 1;
            }
        }
        ok &= check(nav_gm->overwriteMapAndUpdateEsdf(nav_occ), "Nav test GridMap ESDF init");

        path_planning::JPS nav_jps;
        nav_jps.set_safe_threshold(0.35);
        ok &= check(
            nav_jps.jps_search(*nav_gm, Eigen::Vector2d(-1.5, -1.5), Eigen::Vector2d(4.0, 0.0), kTimeoutMs).empty(),
            "JPS rejects OOB goal in OCCUPIED nav map"
        );
        ok &= check(
            !nav_jps.jps_search(*nav_gm, Eigen::Vector2d(-1.5, -1.5), Eigen::Vector2d(1.5, 1.5), kTimeoutMs).empty(),
            "JPS finds interior path in OCCUPIED nav map"
        );

        minco_opt::GridMapESDF nav_esdf(nav_gm);
        ok &= check(nav_esdf.outOfBoundsOccupied(), "GridMapESDF reports outOfBoundsOccupied=true");
        ok &= check(nav_esdf.isCollision(3.5, 0.0, 0.3), "GridMapESDF isCollision true on OOB");

        replan::FsmReplan nav_fsm(env.planner_cfg);
        const utils::RobotState nav_start = make_robot_state(Eigen::Vector2d(-1.8, -1.8));
        const utils::RobotState nav_goal_in = make_robot_state(Eigen::Vector2d(1.8, 1.8));
        const utils::RobotState nav_goal_oob = make_robot_state(Eigen::Vector2d(4.0, 0.0));
        const utils::RobotState nav_start_far_oob = make_robot_state(Eigen::Vector2d(-3.8, 0.0));
        const utils::RobotState nav_start_near_oob = make_robot_state(Eigen::Vector2d(-3.12, -1.5));

        const auto nav_init = nav_fsm.plan(nav_goal_in, nav_start, nav_gm);
        ok &= check(
            nav_init.has_value() && nav_init->is_new_trajectory && nav_init->ma_spline_traj.success,
            "FsmReplan nav mode initial interior plan succeeds"
        );

        const auto nav_rej_goal = nav_fsm.plan(nav_goal_oob, nav_start, nav_gm);
        ok &= check(
            !nav_rej_goal.has_value() && nav_rej_goal.error() == replan::FsmReplan::PathError::PLANNING_FAILED,
            "FsmReplan nav mode rejects OOB goal"
        );
        const auto nav_keep_mon = nav_fsm.plan(nav_goal_in, nav_start, nav_gm);
        ok &= check(
            nav_keep_mon.has_value() && !nav_keep_mon->is_new_trajectory,
            "FsmReplan nav mode preserves committed trajectory after OOB goal rejection"
        );

        const auto nav_rej_start = nav_fsm.plan(nav_goal_in, nav_start_far_oob, nav_gm);
        ok &= check(
            !nav_rej_start.has_value() && nav_rej_start.error() == replan::FsmReplan::PathError::PLANNING_FAILED,
            "FsmReplan nav mode rejects start beyond start_oob_recover_dist"
        );

        replan::FsmReplan recover_fsm(env.planner_cfg);
        const auto nav_rec = recover_fsm.plan(nav_goal_in, nav_start_near_oob, nav_gm);
        ok &= check(
            nav_rec.has_value() && nav_rec->is_new_trajectory && nav_rec->ma_spline_traj.success,
            "FsmReplan nav mode recovers slightly OOB start within start_oob_recover_dist"
        );
        if (nav_rec.has_value() && nav_rec->ma_spline_traj.trajectory.isInitialized()) {
            const auto& tr = nav_rec->ma_spline_traj.trajectory.getTrajectory();
            bool all_inside = true;
            for (double t = tr.getStartTime(); t <= tr.getEndTime(); t += 0.05) {
                const Eigen::Vector2d pt = tr.evaluate(t, 0).head<2>();
                if (!nav_gm->isInsideMap(pt)) {
                    all_inside = false;
                    break;
                }
            }
            all_inside &= nav_gm->isInsideMap(tr.evaluate(tr.getEndTime(), 0).head<2>());
            ok &= check(all_inside, "FsmReplan nav recovered trajectory stays strictly inside map");
        }
    }

    // 8. 新感知架构自检：.terrain.bin 先验 / SensorModel / StaticTerrainBuilder / OnlineTerrainMapper / DynamicStvlLayer
    {
        const std::filesystem::path tmp_dir =
            std::filesystem::temp_directory_path() / ("ma_nav_stvl_sanity_" + std::to_string(::getpid()));
        struct TempDirGuard {
            std::filesystem::path dir;
            ~TempDirGuard() {
                std::error_code ec;
                std::filesystem::remove_all(dir, ec);
            }
        } tmp_guard {tmp_dir};

        // 8a. StaticTerrainPrior & MapIO .terrain.bin 往返、裁剪针孔插值与缺失/失配降级
        const auto p_geom =
            grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(-1.0, -1.0), Eigen::Vector2i(20, 16), 0.1);
        std::vector<ma_map::CellState> p_cells(p_geom.cellCount(), ma_map::CellState::UNKNOWN);
        ma_map::StaticTerrainPrior prior = ma_map::StaticTerrainPrior::createEmpty(p_geom);
        for (int ix = 4; ix <= 14; ++ix) {
            for (int iy = 3; iy <= 12; ++iy) {
                const size_t idx = static_cast<size_t>(p_geom.toAddress(ix, iy));
                p_cells[idx] = ma_map::CellState::FREE;
                prior.ground_z[idx] = 0.25f;
            }
        }
        const size_t oh_idx = static_cast<size_t>(p_geom.toAddress(8, 6));
        prior.overhang_min_z[oh_idx] = 0.55f;
        // 内部被 FREE 包围的针孔：MapIO 清理针孔时同步邻域插值 ground_z
        const size_t pin_idx = static_cast<size_t>(p_geom.toAddress(9, 7));
        p_cells[pin_idx] = ma_map::CellState::UNKNOWN;
        prior.ground_z[pin_idx] = std::numeric_limits<float>::quiet_NaN();

        ma_map::SaveOptions psave_opts;
        psave_opts.crop_to_explored = true;
        psave_opts.margin_cells = 1;
        psave_opts.cleanup_free_pinholes = true;
        psave_opts.save_terrain_prior = true;
        const auto psaved = ma_map::MapIO::save((tmp_dir / "prior_rt").string(), p_geom, p_cells, psave_opts, &prior);
        ok &= check(psaved.has_value() && psaved->saved_terrain_prior, "MapIO saves companion .terrain.bin");
        if (psaved) {
            const auto ploaded = ma_map::MapIO::load(psaved->pgm_path);
            ok &= check(
                ploaded.has_value() && ploaded->terrain_prior.has_value() && ploaded->terrain_prior->isValid(),
                "MapIO loads companion .terrain.bin automatically"
            );
            if (ploaded && ploaded->terrain_prior) {
                const Eigen::Vector2d w_oh = p_geom.indexToPos(Eigen::Vector2i(8, 6));
                const Eigen::Vector2d w_pin = p_geom.indexToPos(Eigen::Vector2i(9, 7));
                const float gz_pin = ploaded->terrain_prior->sampleGroundZ(w_pin, 0);
                const float oh_val = ploaded->terrain_prior->sampleOverhangMinZ(w_oh, 0);
                ok &= check(
                    std::isfinite(gz_pin) && std::abs(gz_pin - 0.25f) < 1e-4f,
                    "MapIO .terrain.bin interpolates ground_z on cleaned pinhole"
                );
                ok &= check(
                    std::isfinite(oh_val) && std::abs(oh_val - 0.55f) < 1e-4f,
                    "MapIO .terrain.bin round-trips overhang_min_z exactly"
                );
            }
            // 删除 .terrain.bin 后仍可无阻塞加载静态图
            std::error_code ec;
            std::filesystem::remove(psaved->terrain_prior_path, ec);
            const auto no_prior = ma_map::MapIO::load(psaved->pgm_path);
            ok &= check(
                no_prior.has_value() && !no_prior->terrain_prior.has_value(),
                "MapIO gracefully falls back when .terrain.bin is absent"
            );
            // 写入几何失配的 .terrain.bin 后仍可降级忽略而不失败
            const auto bad_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(5, 5), 0.2);
            const auto bad_prior = ma_map::StaticTerrainPrior::createEmpty(bad_geom);
            ok &= check(
                ma_map::StaticTerrainPrior::save(psaved->terrain_prior_path, bad_prior).has_value(),
                "StaticTerrainPrior save mismatch test"
            );
            const auto mis_prior = ma_map::MapIO::load(psaved->pgm_path);
            ok &= check(
                mis_prior.has_value() && !mis_prior->terrain_prior.has_value(),
                "MapIO ignores geometry-mismatched .terrain.bin gracefully"
            );
        }

        // 8b. SensorModel REP-103 6-DoF 外参、自遮挡盒、视场与倾角跳变
        {
            ma_map::SensorModel::Config scfg;
            scfg.t_base_lidar = Eigen::Vector3d(0.0, 0.0, 0.194);
            scfg.rpy_deg = Eigen::Vector3d(0.0, -20.0, 0.0); // REP-103: pitch=-20° => +X 上仰 20°
            ma_map::SensorModel sm(scfg);
            const ma_map::SensorPose sp =
                sm.computePose(Eigen::Vector3d(1.0, 2.0, 0.0), Eigen::Quaterniond::Identity());
            const Eigen::Vector3d p_w = sp.p_lidar_w + sp.R_w_lidar * Eigen::Vector3d(1.0, 0.0, 0.0);
            const double rad20 = 20.0 * 3.14159265358979323846 / 180.0;
            ok &= check(
                std::abs(p_w.x() - (1.0 + std::cos(rad20))) < 1e-6 && std::abs(p_w.y() - 2.0) < 1e-6
                    && std::abs(p_w.z() - (0.194 + std::sin(rad20))) < 1e-6,
                "SensorModel REP-103 pitch=-20deg pitches +X upward"
            );
            const Eigen::Vector3d rt = sp.worldToLidar(p_w);
            ok &= check((rt - Eigen::Vector3d(1.0, 0.0, 0.0)).norm() < 1e-9, "SensorPose worldToLidar round-trip");
            ok &= check(
                sm.isSelfHit(Eigen::Vector3d(1.1, 2.0, 0.15), sp) && !sm.isSelfHit(Eigen::Vector3d(2.5, 2.0, 0.15), sp),
                "SensorModel base-frame AABB self-filter"
            );
            ok &= check(
                sm.inFov(Eigen::Vector3d(2.0, 0.0, 0.5), 10.0) && !sm.inFov(Eigen::Vector3d(2.0, 0.0, -1.5), 10.0),
                "SensorModel Mid-360 vertical FOV check"
            );
            const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()));
            const Eigen::Quaterniond q_pitch(
                Eigen::AngleAxisd(10.0 * 3.14159265358979323846 / 180.0, Eigen::Vector3d::UnitY())
            );
            ok &= check(
                ma_map::SensorModel::tiltAngleDeg(Eigen::Quaterniond::Identity(), q_yaw) < 1e-6
                    && std::abs(ma_map::SensorModel::tiltAngleDeg(Eigen::Quaterniond::Identity(), q_pitch) - 10.0)
                        < 1e-3,
                "SensorModel tiltAngleDeg ignores pure yaw and measures pitch accurately"
            );
        }

        // 8c. StaticTerrainBuilder: 20° 缓坡 / 40° 陡坡 / 0.20m 高台边沿 / 0.25m 限高杆 / 0.16m 低杆 / 竖直墙
        {
            const auto t_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(40, 20), 0.05);
            ma_map::StaticTerrainBuilder builder;

            // 20° 缓坡（应全 FREE）
            auto ramp20 = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(t_geom, true, false);
            const float tan20 = std::tan(20.0f * 3.14159265f / 180.0f);
            for (int ix = 0; ix < 40; ++ix) {
                const float z = static_cast<float>(ix) * 0.05f * tan20;
                for (int iy = 0; iy < 20; ++iy) {
                    ramp20.occ_spans[static_cast<size_t>(ix * 20 + iy)].push_back({z - 0.10f, z});
                }
            }
            const auto r20 = builder.build(ramp20);
            ok &= check(r20.occupied_count == 0 && r20.free_count == 800, "StaticTerrainBuilder 20deg ramp all FREE");

            // 40° 陡坡（内部应判 OCCUPIED）
            auto ramp40 = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(t_geom, true, false);
            const float tan40 = std::tan(40.0f * 3.14159265f / 180.0f);
            for (int ix = 0; ix < 40; ++ix) {
                const float z = static_cast<float>(ix) * 0.05f * tan40;
                for (int iy = 0; iy < 20; ++iy) {
                    ramp40.occ_spans[static_cast<size_t>(ix * 20 + iy)].push_back({z - 0.10f, z});
                }
            }
            const auto r40 = builder.build(ramp40);
            ok &= check(
                r40.cells[static_cast<size_t>(20 * 20 + 10)] == ma_map::CellState::OCCUPIED,
                "StaticTerrainBuilder 40deg steep slope OCCUPIED"
            );

            // 0.20m 实心高台 + 0.25m 限高杆 + 0.16m 低杆
            auto scene = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(t_geom, true, false);
            for (int ix = 0; ix < 40; ++ix) {
                for (int iy = 0; iy < 20; ++iy) {
                    const size_t idx = static_cast<size_t>(ix * 20 + iy);
                    if (ix >= 25 && ix <= 35 && iy >= 5 && iy <= 15) {
                        // 0.20m 高台
                        scene.occ_spans[idx].push_back({-0.05f, 0.20f});
                    } else {
                        scene.occ_spans[idx].push_back({-0.05f, 0.0f});
                        if (ix >= 8 && ix <= 10 && iy >= 5 && iy <= 15) {
                            // 0.25m 限高杆（净空 0.25 >= 0.225 -> 可通行 + 记录 overhang_min_z）
                            scene.occ_spans[idx].push_back({0.25f, 0.35f});
                        } else if (ix >= 16 && ix <= 18 && iy >= 5 && iy <= 15) {
                            // 0.16m 低杆（净空 0.16 < 0.225 -> 判障碍）
                            scene.occ_spans[idx].push_back({0.16f, 0.24f});
                        }
                    }
                }
            }
            const auto rs = builder.build(scene);
            const size_t plat_in = static_cast<size_t>(30 * 20 + 10);
            const size_t plat_edge = static_cast<size_t>(25 * 20 + 10);
            const size_t plat_inner2 = static_cast<size_t>(27 * 20 + 10);
            const size_t bar_high = static_cast<size_t>(9 * 20 + 10);
            const size_t bar_low = static_cast<size_t>(17 * 20 + 10);
            ok &= check(
                rs.cells[plat_in] == ma_map::CellState::FREE && rs.cells[plat_inner2] == ma_map::CellState::FREE
                    && std::abs(rs.terrain_prior.ground_z[plat_in] - 0.20f) < 1e-4f
                    && rs.cells[plat_edge] == ma_map::CellState::OCCUPIED,
                "StaticTerrainBuilder 0.20m platform top FREE without edge bleed and curb OCCUPIED"
            );
            ok &= check(
                rs.cells[bar_high] == ma_map::CellState::FREE
                    && std::isfinite(rs.terrain_prior.overhang_min_z[bar_high])
                    && std::abs(rs.terrain_prior.overhang_min_z[bar_high] - 0.25f) < 1e-4f,
                "StaticTerrainBuilder 0.25m height-limit bar FREE with accurate overhang_min_z"
            );
            ok &=
                check(rs.cells[bar_low] == ma_map::CellState::OCCUPIED, "StaticTerrainBuilder 0.16m low bar OCCUPIED");

            // 8c-2. 点云模式：宽厚/分层无地面隧道悬空过滤 + 无地面/开放 0.16m 矮杆保持 OCCUPIED
            auto tunnel_scene = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(t_geom, false, true);
            for (int ix = 0; ix < 40; ++ix) {
                for (int iy = 0; iy < 20; ++iy) {
                    const size_t idx = static_cast<size_t>(ix * 20 + iy);
                    if (ix >= 7 && ix <= 21 && iy >= 3 && iy <= 16) {
                        // 宽无地面隧道（宽 15 格 = 0.75m > 2 * overhang_ground_radius）：厚顶 + 分层顶 + 薄顶
                        if (ix <= 11) {
                            tunnel_scene.occ_spans[idx].push_back({0.30f, 0.48f});
                        } else if (ix <= 16) {
                            tunnel_scene.occ_spans[idx].push_back({0.30f, 0.34f});
                            tunnel_scene.occ_spans[idx].push_back({0.42f, 0.46f});
                        } else {
                            tunnel_scene.occ_spans[idx].push_back({0.30f, 0.34f});
                        }
                        tunnel_scene.cleared_spans[idx].push_back({0.02f, 0.28f});
                    } else if (ix >= 25 && ix <= 27 && iy >= 3 && iy <= 16) {
                        // 无地面 0.16m 低杆（下方有射线扫空但净空 0.16 < 0.225 -> 必须保持 OCCUPIED）
                        tunnel_scene.occ_spans[idx].push_back({0.16f, 0.20f});
                        tunnel_scene.cleared_spans[idx].push_back({-0.01f, 0.15f});
                    } else if (ix >= 30 && ix <= 38 && iy >= 3 && iy <= 16) {
                        // 有地面但夹层被射线扫空的 0.16m 宽矮雨棚 -> 必须保持 OCCUPIED（不得被高台恢复误放）
                        tunnel_scene.occ_spans[idx].push_back({-0.01f, 0.01f});
                        tunnel_scene.occ_spans[idx].push_back({0.16f, 0.20f});
                        tunnel_scene.cleared_spans[idx].push_back({0.02f, 0.15f});
                    } else {
                        tunnel_scene.occ_spans[idx].push_back({-0.01f, 0.01f});
                    }
                }
            }
            const auto rt = builder.build(tunnel_scene);
            const size_t tun_thick = static_cast<size_t>(9 * 20 + 10);
            const size_t tun_deep_split = static_cast<size_t>(14 * 20 + 10);
            const size_t tun_thin = static_cast<size_t>(19 * 20 + 10);
            const size_t bar_groundless_low = static_cast<size_t>(26 * 20 + 10);
            const size_t awning_low = static_cast<size_t>(34 * 20 + 10);
            ok &= check(
                rt.cells[tun_thick] == ma_map::CellState::FREE && rt.cells[tun_deep_split] == ma_map::CellState::FREE
                    && rt.cells[tun_thin] == ma_map::CellState::FREE
                    && std::abs(rt.terrain_prior.ground_z[tun_deep_split]) < 0.03f
                    && std::isfinite(rt.terrain_prior.overhang_min_z[tun_deep_split])
                    && std::abs(rt.terrain_prior.overhang_min_z[tun_deep_split] - 0.30f) < 0.03f,
                "StaticTerrainBuilder filters wide thick/split groundless tunnel overhang"
            );
            ok &= check(
                rt.cells[bar_groundless_low] == ma_map::CellState::OCCUPIED
                    && rt.cells[awning_low] == ma_map::CellState::OCCUPIED,
                "StaticTerrainBuilder keeps groundless 0.16m low bar and ray-cleared 0.16m awning OCCUPIED"
            );

            // 8c-3. 点云模式：空心高台底缝透射低地漏点恢复 + 内部单格孔洞修补 + 台沿/柱子/窄低杆保持 OCCUPIED
            auto plat_scene = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(t_geom, false, true);
            for (int ix = 0; ix < 40; ++ix) {
                for (int iy = 0; iy < 20; ++iy) {
                    const size_t idx = static_cast<size_t>(ix * 20 + iy);
                    if (ix >= 4 && ix <= 15 && iy >= 4 && iy <= 15) {
                        if (ix == 4 || ix == 15 || iy == 4 || iy == 15) {
                            // 高台竖直台沿（HARD_OCC）
                            plat_scene.occ_spans[idx].push_back({-0.01f, 0.22f});
                        } else if (ix >= 12 && ix <= 13 && iy >= 12 && iy <= 13) {
                            // 高台内部 2x2 立柱（HARD_OCC）
                            plat_scene.occ_spans[idx].push_back({0.18f, 0.60f});
                        } else if (ix == 9 && iy == 9) {
                            // 仅剩低地漏点的 1 格内部孔洞
                            plat_scene.occ_spans[idx].push_back({-0.01f, 0.01f});
                        } else if (ix == 10 && iy == 10) {
                            // 未观测扫描缝（1 格内部孔洞）
                        } else {
                            // 低地漏点 + 0.20m 高台顶面双层叠置（夹层无射线扫空）
                            plat_scene.occ_spans[idx].push_back({-0.01f, 0.01f});
                            plat_scene.occ_spans[idx].push_back({0.18f, 0.22f});
                        }
                    } else if (ix >= 22 && ix <= 24 && iy >= 4 && iy <= 15) {
                        // 0.15m 窄低杆（即使夹层无扫空，宽度不足 0.35m 仍须保持 OCCUPIED）
                        plat_scene.occ_spans[idx].push_back({-0.01f, 0.01f});
                        plat_scene.occ_spans[idx].push_back({0.16f, 0.20f});
                    } else {
                        plat_scene.occ_spans[idx].push_back({-0.01f, 0.01f});
                    }
                }
            }
            const auto rp = builder.build(plat_scene);
            const size_t hp_in = static_cast<size_t>(7 * 20 + 7);
            const size_t hp_hole_low = static_cast<size_t>(9 * 20 + 9);
            const size_t hp_hole_unobs = static_cast<size_t>(10 * 20 + 10);
            const size_t hp_curb = static_cast<size_t>(4 * 20 + 10);
            const size_t hp_pillar = static_cast<size_t>(12 * 20 + 12);
            const size_t hp_narrow_bar = static_cast<size_t>(23 * 20 + 10);
            ok &= check(
                rp.cells[hp_in] == ma_map::CellState::FREE && rp.cells[hp_hole_low] == ma_map::CellState::FREE
                    && rp.cells[hp_hole_unobs] == ma_map::CellState::FREE
                    && std::abs(rp.terrain_prior.ground_z[hp_in] - 0.20f) < 0.03f
                    && std::abs(rp.terrain_prior.ground_z[hp_hole_low] - 0.20f) < 0.03f
                    && std::abs(rp.terrain_prior.ground_z[hp_hole_unobs] - 0.20f) < 0.03f,
                "StaticTerrainBuilder recovers hollow 0.20m platform with leaked low floor and fills 1-cell interior holes"
            );
            ok &= check(
                rp.cells[hp_curb] == ma_map::CellState::OCCUPIED && rp.cells[hp_pillar] == ma_map::CellState::OCCUPIED
                    && rp.cells[hp_narrow_bar] == ma_map::CellState::OCCUPIED,
                "StaticTerrainBuilder preserves raised-platform curb, interior pillar, and narrow 0.16m bar OCCUPIED"
            );

            // 8c-4. R3/R4/R5 回归：远距限高杆 + 墙边离群噪点 + 隧道出口顶沿延续，同时守住高台矮杆/斜拱/0.30m 落差底线
            {
                const auto g4 =
                    grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(36, 20), 0.05);
                auto sc4 = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(g4, false, true);
                // 地面在 ix in [0..6] (z=-0.22)，中间 [7..17] 无地面（距离 10 格 > overhang_ground_radius=6）
                for (int ix = 0; ix <= 6; ++ix) {
                    for (int iy = 2; iy <= 17; ++iy) {
                        sc4.occ_spans[static_cast<size_t>(ix * 20 + iy)].push_back({-0.23f, -0.21f});
                    }
                }
                // 隧道两侧竖墙 iy=2 和 iy=17
                for (int ix = 14; ix <= 20; ++ix) {
                    sc4.occ_spans[static_cast<size_t>(ix * 20 + 2)].push_back({-0.22f, 0.55f});
                    sc4.occ_spans[static_cast<size_t>(ix * 20 + 17)].push_back({-0.22f, 0.55f});
                }
                // 远距厚限高杆 ix in [16..18], iy in [3..16] (z in [0.08, 0.28], 厚度 0.20 > 0.11 进阶段 1.5A)，
                // 下方净空 [-0.18, 0.05] 被扫空；且在 (17, 3) 放入单格向下漂移离群噪点 (0.01..0.28)
                for (int ix = 16; ix <= 18; ++ix) {
                    for (int iy = 3; iy <= 16; ++iy) {
                        const size_t idx = static_cast<size_t>(ix * 20 + iy);
                        const float zlo = (ix == 17 && iy == 3) ? 0.01f : 0.08f;
                        sc4.occ_spans[idx].push_back({zlo, 0.28f});
                        sc4.cleared_spans[idx].push_back({-0.18f, 0.05f});
                    }
                }
                // 出口最外 1 格顶沿 ix=19（无下方射线扫空），紧邻出口外真地面 ix=20..24 (z=-0.22)
                for (int iy = 3; iy <= 16; ++iy) {
                    sc4.occ_spans[static_cast<size_t>(19 * 20 + iy)].push_back({0.08f, 0.13f});
                }
                for (int ix = 20; ix <= 24; ++ix) {
                    for (int iy = 2; iy <= 17; ++iy) {
                        sc4.occ_spans[static_cast<size_t>(ix * 20 + iy)].push_back({-0.23f, -0.21f});
                    }
                }
                // 0.30m 台阶/落差在 ix in [28..35] (z=+0.08)，边缘 ix=28 必须保持 OCCUPIED
                for (int ix = 28; ix <= 35; ++ix) {
                    for (int iy = 2; iy <= 17; ++iy) {
                        sc4.occ_spans[static_cast<size_t>(ix * 20 + iy)].push_back({0.07f, 0.09f});
                    }
                }
                for (int ix = 25; ix <= 27; ++ix) {
                    for (int iy = 2; iy <= 17; ++iy) {
                        sc4.occ_spans[static_cast<size_t>(ix * 20 + iy)].push_back({-0.23f, -0.21f});
                    }
                }
                const auto r4 = builder.build(sc4);
                ok &= check(
                    r4.cells[static_cast<size_t>(17 * 20 + 10)] == ma_map::CellState::FREE
                        && r4.cells[static_cast<size_t>(17 * 20 + 4)] == ma_map::CellState::FREE
                        && r4.cells[static_cast<size_t>(19 * 20 + 10)] == ma_map::CellState::FREE
                        && r4.cells[static_cast<size_t>(20 * 20 + 10)] == ma_map::CellState::FREE,
                    "StaticTerrainBuilder clears distant noisy thick bar and 1-cell unrayed tunnel exit lip"
                );
                ok &= check(
                    r4.cells[static_cast<size_t>(17 * 20 + 2)] == ma_map::CellState::OCCUPIED
                        && r4.cells[static_cast<size_t>(17 * 20 + 17)] == ma_map::CellState::OCCUPIED
                        && r4.cells[static_cast<size_t>(28 * 20 + 10)] == ma_map::CellState::OCCUPIED,
                    "StaticTerrainBuilder preserves tunnel side walls and 0.30m drop-off high edge OCCUPIED"
                );

                // 负面安全测试：高台(z=0.20) 上方的 0.16m 矮杆(z=0.36) 不得错借台外低地(z=0.0, 距离 8 格) 被误清
                auto sc_plat_low = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(g4, false, true);
                for (int ix = 0; ix <= 4; ++ix) {
                    for (int iy = 0; iy < 20; ++iy) {
                        sc_plat_low.occ_spans[static_cast<size_t>(ix * 20 + iy)].push_back({-0.01f, 0.01f});
                    }
                }
                for (int ix = 12; ix <= 15; ++ix) {
                    for (int iy = 4; iy <= 15; ++iy) {
                        const size_t idx = static_cast<size_t>(ix * 20 + iy);
                        sc_plat_low.occ_spans[idx].push_back({0.36f, 0.42f});
                        sc_plat_low.cleared_spans[idx].push_back({0.22f, 0.34f});
                    }
                }
                const auto r_pl = builder.build(sc_plat_low);
                ok &= check(
                    r_pl.cells[static_cast<size_t>(13 * 20 + 10)] == ma_map::CellState::OCCUPIED,
                    "StaticTerrainBuilder keeps 0.16m low bar on elevated platform OCCUPIED despite distant lower floor"
                );
            }
        }

        // 8d. OnlineTerrainMapper: 零先验建图 + 信任半径截断远距噪点 + 3D 射线清除动态过客 + 坐标严格对齐
        {
            ma_map::SensorModel::Config scfg;
            scfg.t_base_lidar = Eigen::Vector3d(0.0, 0.0, 0.194);
            scfg.rpy_deg = Eigen::Vector3d(0.0, -20.0, 0.0);
            ma_map::SensorModel sm(scfg);

            ma_map::OnlineTerrainMapper::Config mcfg;
            mcfg.resolution = 0.05;
            mcfg.trust_range = 4.0;
            mcfg.clear_range = 8.0;
            mcfg.static_min_hits = 2;
            ma_map::OnlineTerrainMapper mapper(mcfg);

            const Eigen::Vector3d p_base(0.0, 0.0, 0.0);
            const Eigen::Quaterniond q_base = Eigen::Quaterniond::Identity();
            const ma_map::SensorPose sp = sm.computePose(p_base, q_base);

            auto makeFrame = [](bool with_dynamic) {
                std::vector<Eigen::Vector3d> pts;
                // 地面 [0.5, 3.2] x [-1.2, 1.2], z = 0
                for (double x = 0.525; x <= 3.175; x += 0.05) {
                    for (double y = -1.175; y <= 1.175; y += 0.05) {
                        pts.emplace_back(x, y, 0.0);
                    }
                }
                // 静态墙体 x = 3.025, y in [-1.0, 1.0], z in [0.02, 0.72]
                for (double y = -0.975; y <= 0.975; y += 0.05) {
                    for (double z = 0.025; z <= 0.725; z += 0.05) {
                        pts.emplace_back(3.025, y, z);
                    }
                }
                // 动态过客在 (1.525, 0.025, 0.25..0.55)
                if (with_dynamic) {
                    for (double z = 0.225; z <= 0.525; z += 0.05) {
                        pts.emplace_back(1.525, 0.025, z);
                    }
                }
                // 远距噪点在 (6.525, 0.025, 0.30)（超过 trust_range=4.0m，必须被拒收）
                for (int k = 0; k < 5; ++k) {
                    pts.emplace_back(6.525, 0.025, 0.25 + 0.05 * k);
                }
                return pts;
            };

            const auto pts_dyn = makeFrame(true);
            const auto pts_clean = makeFrame(false);
            for (int f = 0; f < 2; ++f) {
                (void)mapper.integrateFrame(pts_dyn, sp, q_base, sm, false);
            }
            for (int f = 0; f < 6; ++f) {
                (void)mapper.integrateFrame(pts_clean, sp, q_base, sm, false);
            }

            const auto built = mapper.buildGlobalStaticMap();
            ok &= check(built.has_value() && built->geometry.isValid(), "OnlineTerrainMapper builds global static map");
            if (built) {
                const auto wall_idx = built->geometry.posToIndex(Eigen::Vector2d(3.025, 0.025));
                const auto dyn_idx = built->geometry.posToIndex(Eigen::Vector2d(1.525, 0.025));
                const auto flr_idx = built->geometry.posToIndex(Eigen::Vector2d(1.025, 0.525));
                ok &= check(
                    built->geometry.isInside(wall_idx)
                        && built->cells[static_cast<size_t>(built->geometry.toAddress(wall_idx))]
                            == ma_map::CellState::OCCUPIED,
                    "OnlineTerrainMapper preserves static wall at exact world coordinate"
                );
                ok &= check(
                    built->geometry.isInside(dyn_idx)
                        && built->cells[static_cast<size_t>(built->geometry.toAddress(dyn_idx))]
                            == ma_map::CellState::FREE,
                    "OnlineTerrainMapper clears transient dynamic obstacle via 3D ray clearing"
                );
                ok &= check(
                    !built->geometry.isInside(Eigen::Vector2d(6.525, 0.025)),
                    "OnlineTerrainMapper rejects distant noise beyond trust_range"
                );
                ok &= check(
                    built->geometry.isInside(flr_idx)
                        && built->cells[static_cast<size_t>(built->geometry.toAddress(flr_idx))]
                            == ma_map::CellState::FREE
                        && std::abs(
                               built->terrain_prior.ground_z[static_cast<size_t>(built->geometry.toAddress(flr_idx))]
                           ) < 0.03f,
                    "OnlineTerrainMapper recovers accurate ground_z"
                );
            }

            // 8d-2. 浅向下掠射防误擦除（含 hits==0 预清除保护）+ 跨 chunk Z 边界无地面悬空按需分配净空块
            {
                ma_map::OnlineTerrainMapper::Config mcfg2 = mcfg;
                mcfg2.max_clear_rays_per_frame = 4000;
                ma_map::OnlineTerrainMapper graze_mapper(mcfg2);

                std::vector<Eigen::Vector3d> graze_only;
                for (double y = -0.475; y <= 0.475; y += 0.05) {
                    // origin.z=0.194 -> target.z=0.150（落差 0.044 < 0.05），浅向下掠射经过 x=1.525, z=0.172
                    graze_only.emplace_back(3.025, y, 0.15);
                }
                std::vector<Eigen::Vector3d> graze_with_plat = graze_only;
                for (double x = 1.025; x <= 2.175; x += 0.05) {
                    for (double y = -0.475; y <= 0.475; y += 0.05) {
                        graze_with_plat.emplace_back(x, y, 0.17);
                    }
                }
                for (int f = 0; f < 6; ++f) {
                    (void)graze_mapper.integrateFrame(graze_only, sp, q_base, sm, false);
                }
                for (int f = 0; f < 3; ++f) {
                    (void)graze_mapper.integrateFrame(graze_with_plat, sp, q_base, sm, false);
                }
                const auto g_built = graze_mapper.buildGlobalStaticMap();
                bool graze_ok = false;
                if (g_built && g_built->geometry.isValid()) {
                    const auto pi = g_built->geometry.posToIndex(Eigen::Vector2d(1.525, 0.025));
                    if (g_built->geometry.isInside(pi)) {
                        const size_t pa = static_cast<size_t>(g_built->geometry.toAddress(pi));
                        graze_ok = (g_built->cells[pa] == ma_map::CellState::FREE)
                            && std::abs(g_built->terrain_prior.ground_z[pa] - 0.17f) < 0.03f;
                    }
                }
                ok &= check(graze_ok, "OnlineTerrainMapper preserves platform top under shallow downward grazing rays");

                ma_map::OnlineTerrainMapper overhang_mapper(mcfg2);
                std::vector<Eigen::Vector3d> ov_pts;
                // 前后地面 z=0，中间 x in [1.60, 2.40) 无地面（对应整个 chunk cx=2, cz=0 零命中）
                for (double x = 0.525; x <= 1.525; x += 0.05) {
                    for (double y = -0.675; y <= 0.675; y += 0.05) {
                        ov_pts.emplace_back(x, y, 0.0);
                    }
                }
                for (double x = 2.425; x <= 3.175; x += 0.05) {
                    for (double y = -0.675; y <= 0.675; y += 0.05) {
                        ov_pts.emplace_back(x, y, 0.0);
                    }
                }
                // 无地面厚顶棚位于 cz=1 (z in [0.85, 1.00], 厚度 0.15 > 0.11)，下方 cz=0 仅靠扫空射线按需分配
                for (double x = 1.625; x <= 2.375; x += 0.05) {
                    for (double y = -0.575; y <= 0.575; y += 0.05) {
                        for (double z = 0.85; z <= 1.001; z += 0.05) {
                            ov_pts.emplace_back(x, y, z);
                        }
                    }
                }
                // 远端背景墙：让穿洞射线穿过 chunk (cx=2, cz=0) 并记录净空
                for (double y = -0.675; y <= 0.675; y += 0.05) {
                    for (double z = 0.025; z <= 0.675; z += 0.05) {
                        ov_pts.emplace_back(3.025, y, z);
                    }
                }
                for (int f = 0; f < 4; ++f) {
                    (void)overhang_mapper.integrateFrame(ov_pts, sp, q_base, sm, false);
                }
                const auto ov_built = overhang_mapper.buildGlobalStaticMap();
                bool ov_ok = false;
                if (ov_built && ov_built->geometry.isValid()) {
                    const auto oi = ov_built->geometry.posToIndex(Eigen::Vector2d(2.025, 0.025));
                    if (ov_built->geometry.isInside(oi)) {
                        const size_t oa = static_cast<size_t>(ov_built->geometry.toAddress(oi));
                        ov_ok = (ov_built->cells[oa] == ma_map::CellState::FREE)
                            && std::abs(ov_built->terrain_prior.ground_z[oa]) < 0.03f
                            && std::isfinite(ov_built->terrain_prior.overhang_min_z[oa])
                            && ov_built->terrain_prior.overhang_min_z[oa] >= 0.80f;
                    }
                }
                ok &= check(
                    ov_ok,
                    "OnlineTerrainMapper allocates lower clear chunk and filters groundless cross-chunk thick overhang"
                );
            }

            // 8d-3. R1/R2 回归：墙沿虚影过滤 + z_min_mm 分位抗向下离群点 + 超预览半径射线清障 2D 同步
            {
                ma_map::OnlineTerrainMapper::Config mcfg3 = mcfg;
                mcfg3.preview_radius = 2.0;
                mcfg3.trust_range = 2.0;
                mcfg3.clear_range = 6.0;
                mcfg3.max_clear_rays_per_frame = 4000;
                ma_map::OnlineTerrainMapper r12_mapper(mcfg3);

                ma_map::LayeredMap::Config lm_sync_cfg;
                lm_sync_cfg.x_left = 1.0;
                lm_sync_cfg.x_right = 6.0;
                lm_sync_cfg.y_left = 2.0;
                lm_sync_cfg.y_right = 2.0;
                lm_sync_cfg.resolution = 0.05;
                ma_map::LayeredMap sync_lm(lm_sync_cfg);
                sync_lm.setMode(ma_map::MapMode::MAPPING);

                const ma_map::SensorPose sp0 = sm.computePose(Eigen::Vector3d(0.0, 0.0, 0.0), q_base);
                // 步骤 1：在 x=1.525 写入 2 帧动态障碍 + 地面，并用局部预览同步进 2D LayeredMap
                std::vector<Eigen::Vector3d> init_pts;
                for (double x = 0.525; x <= 1.925; x += 0.05) {
                    for (double y = -0.525; y <= 0.525; y += 0.05) {
                        init_pts.emplace_back(x, y, 0.0);
                    }
                }
                for (double z = 0.205; z <= 0.405; z += 0.05) {
                    init_pts.emplace_back(1.525, 0.025, z);
                }
                for (int f = 0; f < 2; ++f) {
                    const auto fr = r12_mapper.integrateFrame(init_pts, sp0, q_base, sm, true);
                    if (fr.has_value()) {
                        sync_lm.updateFromPerception(*fr);
                    }
                }
                auto snap_before = sync_lm.getSnapshot();
                Eigen::Vector2i id_dyn_sync;
                snap_before->posToIndex(Eigen::Vector2d(1.525, 0.025), id_dyn_sync);
                const bool dyn_written_2d = snap_before->isOccupied(id_dyn_sync);

                // 步骤 2：机器人后退到 x=-1.5（使 x=1.525 距车 3.025m > preview_radius=2.0m 但在 clear_range=6.0m 内），
                // 发射穿过 (1.525, 0.025, 0.205..0.405) 命中 x=4.0 远墙的射线 6 帧，验证 2D 旧障碍被同步清除
                const ma_map::SensorPose sp_far = sm.computePose(Eigen::Vector3d(-1.5, 0.0, 0.0), q_base);
                std::vector<Eigen::Vector3d> far_clear_pts;
                for (double z = 0.20; z <= 0.62; z += 0.02) {
                    far_clear_pts.emplace_back(4.0, 0.025, z);
                }
                for (int f = 0; f < 6; ++f) {
                    const auto fr = r12_mapper.integrateFrame(far_clear_pts, sp_far, q_base, sm, true);
                    if (fr.has_value()) {
                        sync_lm.updateFromPerception(*fr);
                    }
                }
                auto snap_after = sync_lm.getSnapshot();
                snap_after->posToIndex(Eigen::Vector2d(1.525, 0.025), id_dyn_sync);
                const auto r12_global = r12_mapper.buildGlobalStaticMap();
                bool global_cleared = false;
                if (r12_global && r12_global->geometry.isValid()) {
                    const auto gi = r12_global->geometry.posToIndex(Eigen::Vector2d(1.525, 0.025));
                    if (r12_global->geometry.isInside(gi)) {
                        global_cleared =
                            (r12_global->cells[static_cast<size_t>(r12_global->geometry.toAddress(gi))]
                             == ma_map::CellState::FREE);
                    }
                }
                ok &= check(
                    dyn_written_2d,
                    "OnlineTerrainMapper local preview writes blocking obstacle to 2D LayeredMap"
                );
                ok &= check(global_cleared, "OnlineTerrainMapper 3D ray clears obstacle beyond preview_radius");
                ok &= check(
                    !snap_after->isOccupied(id_dyn_sync),
                    "OnlineTerrainMapper syncs outside-preview ray-erased obstacle into 2D LayeredMap"
                );

                // 步骤 3：R2b 验证单个向下漂移离群点 (0.050m) 不会永久锁死高处体素 z_min_mm，经后续 0.085m 命中可恢复
                ma_map::OnlineTerrainMapper q_mapper(mcfg);
                std::vector<Eigen::Vector3d> base_floor;
                for (double x = 0.525; x <= 2.525; x += 0.05) {
                    for (double y = -0.525; y <= 0.525; y += 0.05) {
                        base_floor.emplace_back(x, y, -0.15);
                    }
                }
                // 第 1 帧在悬空杆处落入 1 个 0.052m 向下离群点（相对 z=-0.15 仅 0.202m < 0.225m）
                auto f_outlier = base_floor;
                f_outlier.emplace_back(1.525, 0.025, 0.052);
                (void)q_mapper.integrateFrame(f_outlier, sp0, q_base, sm, false);
                // 随后 40 帧在 0.088m（相对 z=-0.15 净空 0.238m >= 0.225m）稳定命中
                auto f_bar = base_floor;
                f_bar.emplace_back(1.525, 0.025, 0.088);
                for (int f = 0; f < 40; ++f) {
                    (void)q_mapper.integrateFrame(f_bar, sp0, q_base, sm, false);
                }
                const auto q_built = q_mapper.buildGlobalStaticMap();
                bool q_ok = false;
                if (q_built && q_built->geometry.isValid()) {
                    const auto qi = q_built->geometry.posToIndex(Eigen::Vector2d(1.525, 0.025));
                    if (q_built->geometry.isInside(qi)) {
                        const size_t qa = static_cast<size_t>(q_built->geometry.toAddress(qi));
                        q_ok = (q_built->cells[qa] == ma_map::CellState::FREE)
                            && std::isfinite(q_built->terrain_prior.overhang_min_z[qa])
                            && q_built->terrain_prior.overhang_min_z[qa] >= 0.080f;
                    }
                }
                ok &= check(
                    q_ok,
                    "OnlineTerrainMapper z_min_mm asymmetric quantile recovers from single downward outlier"
                );

                // 步骤 4：R2a 墙沿单格虚影过滤（hits=4, clears=5 仍在 static_clear_ratio=2.0 内，紧邻稠密竖墙时应滤除，真墙保留）
                ma_map::OnlineTerrainMapper wb_mapper(mcfg);
                std::vector<Eigen::Vector3d> wb_base;
                for (double x = 0.525; x <= 2.975; x += 0.05) {
                    for (double y = -0.325; y <= 0.325; y += 0.05) {
                        wb_base.emplace_back(x, y, 0.0);
                    }
                }
                // 沿 y=0.225 的稠密竖墙（x in [1.025, 2.525], z in [0.025, 0.525]）
                for (double x = 1.025; x <= 2.525; x += 0.05) {
                    for (double z = 0.025; z <= 0.525; z += 0.05) {
                        wb_base.emplace_back(x, 0.225, z);
                    }
                }
                // 先在紧邻墙面 1 格处 (1.525, 0.175, 0.225) 写入 4 帧墙沿虚影
                const ma_map::SensorPose sp_wb = sm.computePose(Eigen::Vector3d(0.0, 0.175, 0.0), q_base);
                auto wb_with_ghost = wb_base;
                wb_with_ghost.emplace_back(1.525, 0.175, 0.225);
                for (int f = 0; f < 4; ++f) {
                    (void)wb_mapper.integrateFrame(wb_with_ghost, sp_wb, q_base, sm, false);
                }
                // 再由穿过 (1.525, 0.175, 0.225) 命中远端 x=3.050 的射线穿透 5 帧，同时继续累积真墙至 12 帧
                auto wb_with_ray = wb_base;
                wb_with_ray.emplace_back(3.050, 0.175, 0.256);
                for (int f = 0; f < 5; ++f) {
                    (void)wb_mapper.integrateFrame(wb_with_ray, sp_wb, q_base, sm, false);
                }
                for (int f = 0; f < 3; ++f) {
                    (void)wb_mapper.integrateFrame(wb_base, sp_wb, q_base, sm, false);
                }
                const auto wb_built = wb_mapper.buildGlobalStaticMap();
                bool wb_ok = false;
                if (wb_built && wb_built->geometry.isValid()) {
                    const auto gi = wb_built->geometry.posToIndex(Eigen::Vector2d(1.525, 0.175));
                    const auto wi = wb_built->geometry.posToIndex(Eigen::Vector2d(1.525, 0.225));
                    if (wb_built->geometry.isInside(gi) && wb_built->geometry.isInside(wi)) {
                        const auto gst = wb_built->cells[static_cast<size_t>(wb_built->geometry.toAddress(gi))];
                        const auto wst = wb_built->cells[static_cast<size_t>(wb_built->geometry.toAddress(wi))];
                        wb_ok = (gst == ma_map::CellState::FREE) && (wst == ma_map::CellState::OCCUPIED);
                    }
                }
                ok &= check(wb_ok, "OnlineTerrainMapper filters wall-bleed fringe voxel while preserving dense wall");
            }
        }

        // 8e. DynamicStvlLayer: 静态 ESDF 墙边抑制 + 2.5D 先验 Z 漂移跟踪/悬空剥离 + 移动障碍即时检测与射线清零
        {
            const auto n_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(-1.0, -2.5), Eigen::Vector2i(100, 100), 0.05);
            std::vector<ma_map::CellState> n_cells(n_geom.cellCount(), ma_map::CellState::FREE);
            ma_map::StaticTerrainPrior n_prior = ma_map::StaticTerrainPrior::createEmpty(n_geom);
            std::fill(n_prior.ground_z.begin(), n_prior.ground_z.end(), 0.0f);
            // 静态墙体 x = 3.0m（ix = 80）
            for (int iy = 0; iy < 100; ++iy) {
                n_cells[static_cast<size_t>(n_geom.toAddress(80, iy))] = ma_map::CellState::OCCUPIED;
            }
            // 已知静态悬空限高杆区域 x in [1.0, 1.5], y in [-1.5, -0.5], overhang_min_z = 0.25m
            for (int ix = 40; ix <= 50; ++ix) {
                for (int iy = 20; iy <= 40; ++iy) {
                    n_prior.overhang_min_z[static_cast<size_t>(n_geom.toAddress(ix, iy))] = 0.25f;
                }
            }

            ma_map::SaveOptions ns_opts;
            ns_opts.crop_to_explored = false;
            ns_opts.save_terrain_prior = true;
            const auto n_saved =
                ma_map::MapIO::save((tmp_dir / "stvl_static").string(), n_geom, n_cells, ns_opts, &n_prior);
            ok &= check(n_saved.has_value() && n_saved->saved_terrain_prior, "DynamicStvlLayer test static map saved");

            ma_map::LayeredMap::Config lm_cfg;
            lm_cfg.x_left = 1.0;
            lm_cfg.x_right = 4.0;
            lm_cfg.y_left = 2.5;
            lm_cfg.y_right = 2.5;
            lm_cfg.resolution = 0.05;
            ma_map::LayeredMap nav_lm(lm_cfg);
            nav_lm.setMode(ma_map::MapMode::NAVIGATION);
            const auto n_loaded = nav_lm.loadStaticMap(n_saved->pgm_path);
            ok &= check(
                n_loaded.has_value() && n_loaded->has_terrain_prior,
                "LayeredMap loads static map + terrain prior"
            );
            const ma_map::StaticNavContext sctx = nav_lm.getStaticNavContext();
            ok &= check(
                sctx.static_esdf != nullptr && sctx.terrain_prior != nullptr,
                "StaticNavContext exposes static_esdf and terrain_prior"
            );

            ma_map::SensorModel::Config scfg;
            scfg.t_base_lidar = Eigen::Vector3d(0.0, 0.0, 0.194);
            scfg.rpy_deg = Eigen::Vector3d(0.0, -20.0, 0.0);
            ma_map::SensorModel sm(scfg);
            ma_map::DynamicStvlLayer stvl;

            const Eigen::Vector3d p_base(0.0, 0.0, 0.0);
            const Eigen::Quaterniond q_base = Eigen::Quaterniond::Identity();
            const ma_map::SensorPose sp = sm.computePose(p_base, q_base);
            constexpr double kDriftZ = 0.10;

            auto makeNavCloud = [&](bool with_dyn) {
                std::vector<Eigen::Vector3d> pts;
                // 含 +0.10m Z 漂移的地面点
                for (double x = 0.6; x <= 2.7; x += 0.10) {
                    for (double y = -1.5; y <= 1.5; y += 0.10) {
                        pts.emplace_back(x, y, kDriftZ);
                    }
                }
                // 墙边 0.05m 抖动点（距静态墙 x=3.025 <= wall_suppress_dist=0.08m，应被纯静态 ESDF 抑制）
                for (double y = -1.2; y <= 1.2; y += 0.08) {
                    for (double z = 0.25; z <= 0.65; z += 0.08) {
                        pts.emplace_back(2.97, y, z + kDriftZ);
                    }
                }
                // 静态墙体背景点（作为射线清除背景）
                // 墙顶需覆盖最高动态体素在 x=3.025 处的投影：z_wall = 0.194 + (0.675-0.194)*3.025/1.725
                // = 1.0375m；原 0.85+0.10=0.95m 不与该射线相交，导致顶层体素永远无“后方回波”。
                for (double y = -1.2; y <= 1.2; y += 0.05) {
                    for (double z = 0.15; z <= 1.10; z += 0.05) {
                        pts.emplace_back(3.025, y, z + kDriftZ);
                    }
                }
                // 已知限高杆点（z_corr = 0.26 >= 0.25 - 0.06，应被 terrain_prior 剥离）
                for (double x = 1.05; x <= 1.45; x += 0.05) {
                    for (double y = -1.45; y <= -0.55; y += 0.05) {
                        pts.emplace_back(x, y, 0.26 + kDriftZ);
                    }
                }
                // 动态障碍物簇（位于 (1.8, 0.0)，高度 0.25..0.55）
                if (with_dyn) {
                    for (double x = 1.72; x <= 1.88; x += 0.05) {
                        for (double y = -0.10; y <= 0.10; y += 0.05) {
                            for (double z = 0.25; z <= 0.55; z += 0.05) {
                                pts.emplace_back(x, y, z + kDriftZ);
                            }
                        }
                    }
                }
                return pts;
            };

            const auto nav_static_only = makeNavCloud(false);
            const auto nav_with_dyn = makeNavCloud(true);

            // Frame 1: 纯静态场景 + +0.10m Z 漂移 -> 零误检 + 精确跟踪 Z 漂移
            const auto f1 = stvl.update(nav_static_only, sp, q_base, sm, sctx, 0.0);
            ok &= check(
                f1.obstacle_cells.empty() && std::abs(stvl.estimatedDriftZ() - static_cast<float>(kDriftZ)) < 0.03f,
                "DynamicStvlLayer tracks Z-drift and emits 0 false obstacles on floor/wall-halo/overhang"
            );

            // Frame 2: 动态障碍物出现 -> 单帧 3D 成簇即刻检测
            const auto f2 = stvl.update(nav_with_dyn, sp, q_base, sm, sctx, 0.1);
            ok &= check(
                !f2.obstacle_cells.empty(),
                "DynamicStvlLayer detects moving obstacle immediately via 3D cluster confirm"
            );

            // Frame 3 & 4: 动态障碍物移开 -> 极坐标深度图射线穿透在 2 帧内清零（零拖影）
            (void)stvl.update(nav_static_only, sp, q_base, sm, sctx, 0.2);
            const auto f4 = stvl.update(nav_static_only, sp, q_base, sm, sctx, 0.3);
            ok &= check(
                f4.obstacle_cells.empty() && stvl.activeVoxelCount() == 0,
                "DynamicStvlLayer ray-clears vanished dynamic obstacle within 2 frames (zero ghosting)"
            );
        }
    }

    // 9. 回归自检：边界/退化输入与算法修复（mergeSpans NaN、Z 漂移重捕获、无先验斜坡、ESDF/碰撞边界）
    {
        const float kNaN = std::numeric_limits<float>::quiet_NaN();

        // 9a. StaticTerrainBuilder::mergeSpans：过滤 NaN/Inf，并在排序前归一化倒置上下界
        {
            const auto ms_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(4, 4), 0.05);
            ma_map::StaticTerrainBuilder ms_builder;
            auto ms_grid = ma_map::StaticTerrainBuilder::VoxelColumnGrid::create(ms_geom, true, false);
            for (int ix = 0; ix < 4; ++ix) {
                for (int iy = 0; iy < 4; ++iy) {
                    auto& spans = ms_grid.occ_spans[static_cast<size_t>(ix * 4 + iy)];
                    spans.push_back({-0.05f, 0.0f});
                    spans.push_back({0.10f, 0.0f}); // 倒置：应归一化为 [0.0, 0.10] 后与地面段合并
                    if (ix == 0 && iy == 0) {
                        spans.push_back({kNaN, kNaN});
                        spans.push_back({kNaN, 0.20f});
                    }
                }
            }
            const auto ms_res = ms_builder.build(ms_grid);
            bool ms_ok = ms_res.free_count == 16 && ms_res.occupied_count == 0 && ms_res.unknown_count == 0;
            for (int i = 0; i < 16; ++i) {
                const float gz = ms_res.terrain_prior.ground_z[static_cast<size_t>(i)];
                ms_ok = ms_ok && std::isfinite(gz) && std::abs(gz - 0.10f) < 1e-4f;
            }
            ok &= check(ms_ok, "StaticTerrainBuilder mergeSpans filters NaN/Inf and normalizes inverted spans");
        }

        // 9b. DynamicStvlLayer：LIO Z 突跳 > 0.25m 后 3 帧内用宽门限立即重捕获（不再永久锁死）
        {
            const auto dg_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(-1.0, -1.0), Eigen::Vector2i(100, 100), 0.05);
            auto dprior =
                std::make_shared<ma_map::StaticTerrainPrior>(ma_map::StaticTerrainPrior::createEmpty(dg_geom));
            std::fill(dprior->ground_z.begin(), dprior->ground_z.end(), 0.0f);
            ma_map::StaticNavContext dctx;
            dctx.terrain_prior = dprior;

            ma_map::DynamicStvlLayer::Config dcfg;
            dcfg.drift_max_abs = 0.60f; // 允许 0.30m 突跳落入宽门限
            dcfg.drift_min_samples = 30;
            ma_map::DynamicStvlLayer drift_layer(dcfg);
            ma_map::SensorModel dsm;

            const Eigen::Quaterniond q_id = Eigen::Quaterniond::Identity();
            const ma_map::SensorPose d_sp = dsm.computePose(Eigen::Vector3d::Zero(), q_id);

            auto make_ground = [](double z_off) {
                std::vector<Eigen::Vector3d> pts;
                for (double x = 0.6; x <= 3.0 + 1e-9; x += 0.10) {
                    for (double y = -1.5; y <= 1.5 + 1e-9; y += 0.10) {
                        pts.emplace_back(x, y, z_off);
                    }
                }
                return pts;
            };

            const auto g_base = make_ground(0.02);
            (void)drift_layer.update(g_base, d_sp, q_id, dsm, dctx, 0.0);
            ok &= check(
                std::abs(drift_layer.estimatedDriftZ() - 0.02f) < 0.03f,
                "DynamicStvlLayer acquires initial Z-drift within wide gate"
            );

            const auto g_jump = make_ground(0.32); // +0.30m，超过 0.25m 窄门限但 <= drift_max_abs
            for (int f = 1; f <= 3; ++f) {
                (void)drift_layer.update(g_jump, d_sp, q_id, dsm, dctx, 0.1 * static_cast<double>(f));
            }
            ok &= check(
                std::abs(drift_layer.estimatedDriftZ() - 0.32f) < 0.03f && drift_layer.estimatedDriftZ() > 0.10f,
                "DynamicStvlLayer re-acquires >0.25m Z-drift jump within 3 frames"
            );
        }

        // 9c. DynamicStvlLayer 无先验：±20° 倾斜坡面零误检，且能检测坡面上 0.35m 障碍
        {
            ma_map::DynamicStvlLayer::Config rcfg;
            rcfg.fast_confirm_with_neighbor = true;
            ma_map::DynamicStvlLayer ramp_layer(rcfg);
            ma_map::SensorModel rsm;
            ma_map::StaticNavContext no_prior_ctx; // 无 terrain_prior / static_esdf / snapshot

            constexpr double kPi = 3.14159265358979323846;
            const double rad20 = 20.0 * kPi / 180.0;

            for (const int sign: {-1, 1}) {
                const double theta = static_cast<double>(sign) * rad20;
                const double tan_theta = std::tan(theta);
                const Eigen::Quaterniond q_ramp(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));
                const ma_map::SensorPose ramp_sp = rsm.computePose(Eigen::Vector3d::Zero(), q_ramp);
                const Eigen::Vector3d base_z = ramp_sp.R_w_base.col(2);
                ok &= check(
                    std::abs(base_z.z() - std::cos(rad20)) < 1e-9,
                    "DynamicStvlLayer no-prior ramp test uses 20deg tilted base Z axis"
                );

                // 坡面过原点且与 base Z 轴垂直：z = -tan(theta) * x。
                auto make_ramp = [&](bool with_obstacle) {
                    std::vector<Eigen::Vector3d> pts;
                    for (double x = 0.8; x <= 3.0 + 1e-9; x += 0.08) {
                        for (double y = -1.0; y <= 1.0 + 1e-9; y += 0.10) {
                            pts.emplace_back(x, y, -tan_theta * x);
                        }
                    }
                    if (with_obstacle) {
                        for (int ix = -1; ix <= 1; ++ix) {
                            for (int iy = -1; iy <= 1; ++iy) {
                                for (int iz = 0; iz <= 2; ++iz) {
                                    const double x = 1.8 + 0.05 * static_cast<double>(ix);
                                    const double y = 0.05 * static_cast<double>(iy);
                                    const Eigen::Vector3d base_on_plane(x, y, -tan_theta * x);
                                    pts.push_back(base_on_plane + base_z * (0.35 + 0.05 * static_cast<double>(iz)));
                                }
                            }
                        }
                    }
                    return pts;
                };

                const auto ramp_only = make_ramp(false);
                const auto ramp_with_obstacle = make_ramp(true);

                const auto r0 = ramp_layer.update(ramp_only, ramp_sp, q_ramp, rsm, no_prior_ctx, 0.0);
                ok &= check(
                    r0.obstacle_cells.empty(),
                    sign < 0 ? "DynamicStvlLayer no-prior -20deg ramp emits zero false obstacles"
                             : "DynamicStvlLayer no-prior +20deg ramp emits zero false obstacles"
                );

                (void)ramp_layer.update(ramp_with_obstacle, ramp_sp, q_ramp, rsm, no_prior_ctx, 0.1);
                const auto r2 = ramp_layer.update(ramp_with_obstacle, ramp_sp, q_ramp, rsm, no_prior_ctx, 0.2);
                ok &= check(
                    !r2.obstacle_cells.empty(),
                    sign < 0 ? "DynamicStvlLayer no-prior -20deg ramp detects 0.35m obstacle above tilted ground"
                             : "DynamicStvlLayer no-prior +20deg ramp detects 0.35m obstacle above tilted ground"
                );

                ramp_layer.reset();
            }
        }

        // 9d. GridMap：safe_threshold <= 0 时占据格仍判碰撞，相邻自由格不误判
        {
            const auto g_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(21, 21), 0.1);
            grid_map::GridMap surf;
            surf.init(g_geom, grid_map::OutOfBoundsPolicy::FREE);
            grid_map::RowMatrixXi occ = grid_map::RowMatrixXi::Zero(21, 21);
            occ(10, 10) = 1;
            ok &=
                check(surf.overwriteMapAndUpdateEsdf(occ), "GridMap single-obstacle ESDF for collision boundary test");

            Eigen::Vector2d p_occ;
            Eigen::Vector2d p_free;
            surf.indexToPos(Eigen::Vector2i(10, 10), p_occ);
            surf.indexToPos(Eigen::Vector2i(11, 10), p_free);
            ok &= check(
                std::abs(surf.getDistance(p_occ)) < 1e-12 && surf.isCollision(p_occ, 0.0)
                    && surf.isCollision(p_occ, -1.0),
                "GridMap isCollision(threshold<=0) true on zero-ESDF occupied cell"
            );
            ok &= check(
                surf.getDistance(p_free) > 0.0 && !surf.isCollision(p_free, 0.0),
                "GridMap isCollision(threshold=0) false on adjacent free cell (esdf=+res)"
            );

            // 地图边界格（采样走双线性分支）同样不得漏判
            const auto b_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(11, 11), 0.1);
            grid_map::GridMap border;
            border.init(b_geom, grid_map::OutOfBoundsPolicy::FREE);
            grid_map::RowMatrixXi bocc = grid_map::RowMatrixXi::Zero(11, 11);
            bocc(0, 0) = 1;
            ok &= check(border.overwriteMapAndUpdateEsdf(bocc), "GridMap border-obstacle ESDF");
            Eigen::Vector2d b_occ;
            Eigen::Vector2d b_free;
            border.indexToPos(Eigen::Vector2i(0, 0), b_occ);
            border.indexToPos(Eigen::Vector2i(1, 0), b_free);
            ok &= check(
                border.isCollision(b_occ, 0.0) && !border.isCollision(b_free, 0.0),
                "GridMap border obstacle collision at threshold 0 without adjacent free false positive"
            );
        }

        // 9e. GridMap::isLineOccupancy：精确终点 p2、对角内部障碍与自由线
        {
            const auto g_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(21, 21), 0.1);
            grid_map::GridMap line_map;
            line_map.init(g_geom, grid_map::OutOfBoundsPolicy::FREE);
            grid_map::RowMatrixXi occ = grid_map::RowMatrixXi::Zero(21, 21);
            occ(10, 10) = 1;
            ok &= check(line_map.overwriteMapAndUpdateEsdf(occ), "GridMap line-occupancy ESDF");

            Eigen::Vector2d l_a;
            Eigen::Vector2d l_b;
            line_map.indexToPos(Eigen::Vector2i(5, 10), l_a);
            line_map.indexToPos(Eigen::Vector2i(10, 10), l_b);
            ok &= check(line_map.isLineOccupancy(l_a, l_b), "GridMap isLineOccupancy checks exact endpoint p2");

            Eigen::Vector2d d_a;
            Eigen::Vector2d d_b;
            line_map.indexToPos(Eigen::Vector2i(6, 6), d_a);
            line_map.indexToPos(Eigen::Vector2i(14, 14), d_b);
            ok &=
                check(line_map.isLineOccupancy(d_a, d_b), "GridMap isLineOccupancy catches diagonal interior obstacle");

            Eigen::Vector2d f_a;
            Eigen::Vector2d f_b;
            line_map.indexToPos(Eigen::Vector2i(5, 5), f_a);
            line_map.indexToPos(Eigen::Vector2i(9, 9), f_b);
            ok &= check(!line_map.isLineOccupancy(f_a, f_b), "GridMap isLineOccupancy free line remains clear");
        }

        // 9f. GridMap ESDF：混合图全有限且对角距离精确；全空闲/全占据退化仍有限
        {
            const auto g_geom =
                grid_map::GridGeometry::fromOriginAndSize(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2i(21, 21), 0.1);
            grid_map::GridMap mixed;
            mixed.init(g_geom, grid_map::OutOfBoundsPolicy::FREE);
            grid_map::RowMatrixXi occ = grid_map::RowMatrixXi::Zero(21, 21);
            occ(10, 10) = 1;
            ok &= check(mixed.overwriteMapAndUpdateEsdf(occ), "GridMap mixed ESDF");

            bool all_finite = true;
            for (const double v: mixed.esdfData()) {
                all_finite = all_finite && std::isfinite(v);
            }
            Eigen::Vector2d p00;
            mixed.indexToPos(Eigen::Vector2i(0, 0), p00);
            const double expected = std::sqrt(200.0) * 0.1;
            ok &= check(
                all_finite && std::abs(mixed.getDistance(p00) - expected) < 1e-4,
                "GridMap fillESDF skips +inf sites without NaN and keeps exact diagonal distance"
            );

            grid_map::GridMap deg_free;
            deg_free.init(g_geom, grid_map::OutOfBoundsPolicy::FREE);
            const grid_map::RowMatrixXi deg_free_occ = grid_map::RowMatrixXi::Zero(21, 21);
            ok &= check(deg_free.overwriteMapAndUpdateEsdf(deg_free_occ), "GridMap all-free fillESDF guard");
            bool free_finite = true;
            for (const double v: deg_free.esdfData()) {
                free_finite = free_finite && std::isfinite(v) && v > 0.0;
            }
            ok &= check(free_finite, "GridMap all-free ESDF is finite positive");

            grid_map::GridMap deg_occ;
            deg_occ.init(g_geom, grid_map::OutOfBoundsPolicy::FREE);
            const grid_map::RowMatrixXi deg_occ_grid = grid_map::RowMatrixXi::Ones(21, 21);
            ok &= check(deg_occ.overwriteMapAndUpdateEsdf(deg_occ_grid), "GridMap all-occupied fillESDF guard");
            bool occ_finite = true;
            for (const double v: deg_occ.esdfData()) {
                occ_finite = occ_finite && std::isfinite(v) && v < 0.0;
            }
            ok &= check(occ_finite, "GridMap all-occupied ESDF is finite negative");
        }

        // 9g. SensorModel::setConfig：NaN / 倒置边界净化后仍可安全计算
        {
            const double dNaN = std::numeric_limits<double>::quiet_NaN();
            ma_map::SensorModel::Config bad;
            bad.t_base_lidar = Eigen::Vector3d::Constant(dNaN);
            bad.rpy_deg = Eigen::Vector3d::Constant(dNaN);
            bad.min_range = dNaN;
            bad.max_range = dNaN;
            bad.fov_min_elev_deg = 60.0; // 倒置：min > max
            bad.fov_max_elev_deg = -10.0;
            bad.fov_margin_deg = -5.0; // 负容差必须钳制为非负
            bad.self_filter_min = Eigen::Vector3d::Constant(dNaN);
            bad.self_filter_max = Eigen::Vector3d::Constant(dNaN);

            ma_map::SensorModel bad_sm(bad);
            const auto& bcfg = bad_sm.config();
            ok &= check(
                bcfg.t_base_lidar.allFinite() && bcfg.rpy_deg.allFinite() && bcfg.self_filter_min.allFinite()
                    && bcfg.self_filter_max.allFinite(),
                "SensorModel setConfig sanitizes non-finite extrinsics/self-filter"
            );
            ok &= check(
                std::isfinite(bcfg.min_range) && std::isfinite(bcfg.max_range) && bcfg.min_range <= bcfg.max_range
                    && bcfg.fov_min_elev_deg <= bcfg.fov_max_elev_deg && bcfg.fov_margin_deg >= 0.0,
                "SensorModel setConfig sanitizes NaN/inverted range and FOV bounds"
            );

            const auto bad_pose = bad_sm.computePose(Eigen::Vector3d::Constant(dNaN), Eigen::Quaterniond::Identity());
            ok &= check(
                bad_pose.p_base_w.allFinite() && bad_pose.R_w_base.allFinite() && bad_pose.p_lidar_w.allFinite(),
                "SensorModel computePose guards non-finite base position"
            );
            ok &= check(
                !bad_sm.inFov(Eigen::Vector3d::Constant(dNaN)),
                "SensorModel inFov rejects non-finite lidar point"
            );

            ma_map::SensorModel::Config inverted;
            inverted.min_range = 20.0;
            inverted.max_range = 0.5;
            ma_map::SensorModel inverted_sm(inverted);
            ok &= check(
                inverted_sm.config().min_range <= inverted_sm.config().max_range,
                "SensorModel setConfig clamps inverted range bounds"
            );
        }

        // 9h. utils::Clock / ManualClock / AtomicClockSource 与时钟回退保护
        {
            utils::ManualClock mclk;
            mclk.setNs(1'000'000'000LL);
            const utils::Clock clk = mclk.asClock();
            ok &= check(std::abs(clk.nowSec() - 1.0) < 1e-9, "utils::ManualClock initial time");
            mclk.advanceSec(0.5);
            ok &= check(std::abs(clk.nowSec() - 1.5) < 1e-9, "utils::ManualClock advanceSec");

            utils::AtomicClockSource aclk;
            aclk.storeSec(42.25);
            ok &= check(std::abs(aclk.asClock().nowSec() - 42.25) < 1e-6, "utils::AtomicClockSource store/load");

            ma_map::SensorModel::Config sc;
            sc.t_base_lidar = Eigen::Vector3d(0.0, 0.0, 0.15);
            sc.rpy_deg = Eigen::Vector3d::Zero();
            ma_map::SensorModel sm(sc);
            const auto sp = sm.computePose(Eigen::Vector3d(0.0, 0.0, 0.06), Eigen::Quaterniond::Identity());

            ma_map::DynamicStvlLayer::Config dc;
            dc.decay_in_fov_sec = 0.30f;
            dc.decay_blind_sec = 0.30f;
            dc.fallback_ground_offset = 0.06f;
            ma_map::DynamicStvlLayer dl(dc);
            dl.setClock(clk);

            ma_map::StaticNavContext empty_ctx;
            std::vector<Eigen::Vector3d> obs_pts;
            for (double x = 1.6; x <= 1.8; x += 0.05) {
                for (double y = -0.1; y <= 0.1; y += 0.05) {
                    for (double z = 0.25; z <= 0.45; z += 0.05) {
                        obs_pts.emplace_back(x, y, z);
                    }
                }
            }
            dl.update(obs_pts, sp, Eigen::Quaterniond::Identity(), sm, empty_ctx, -1.0);
            const size_t active_before = dl.activeVoxelCount();
            // 模拟仿真重置时钟回退：体素不应因负 dt 崩溃或异常消失
            mclk.setSec(0.1);
            dl.update({}, sp, Eigen::Quaterniond::Identity(), sm, empty_ctx, -1.0);
            const size_t active_after_back = dl.activeVoxelCount();
            // 回退后再推进超过 decay 时间：应正常按注入时钟衰减清除
            mclk.advanceSec(0.5);
            dl.update({}, sp, Eigen::Quaterniond::Identity(), sm, empty_ctx, -1.0);
            ok &= check(
                active_before > 0 && active_after_back == active_before && dl.activeVoxelCount() == 0,
                "DynamicStvlLayer handles injected ManualClock and backward clock jump"
            );
        }

        // 9i. stvl_prior 3D 可视化按需采集：过滤后点云 / 动态 3D / 建图静态 3D / 2.5D 先验快照版本号
        {
            ma_map::SensorModel::Config sc;
            sc.t_base_lidar = Eigen::Vector3d(0.0, 0.0, 0.2);
            sc.rpy_deg = Eigen::Vector3d::Zero();
            sc.min_range = 0.25;
            sc.max_range = 8.0;
            sc.self_filter_min = Eigen::Vector3d(-0.35, -0.35, -0.10);
            sc.self_filter_max = Eigen::Vector3d(0.35, 0.35, 0.55);
            ma_map::SensorModel sm(sc);
            const auto sp = sm.computePose(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());

            // 9i-1. OnlineTerrainMapper: filtered_cloud 与 extractStaticVoxelCloud + revision
            ma_map::OnlineTerrainMapper::Config mc;
            mc.resolution = 0.05;
            mc.min_z = -0.5;
            mc.max_z = 2.0;
            mc.trust_range = 5.0;
            mc.static_min_hits = 2;
            mc.preview_radius = 4.0;
            ma_map::OnlineTerrainMapper mapper(mc);

            const double dNaN = std::numeric_limits<double>::quiet_NaN();
            std::vector<Eigen::Vector3d> map_pts;
            map_pts.emplace_back(dNaN, 0.0, 0.0); // NaN：必须滤除
            map_pts.emplace_back(0.1, 0.0, 0.2); // 自遮挡：必须滤除
            map_pts.emplace_back(2.0, 0.0, 3.0); // 超 max_z：必须滤除
            map_pts.emplace_back(6.5, 0.0, 0.3); // 超 trust_range：必须滤除
            map_pts.emplace_back(2.0, 0.0, 0.3); // 有效点 1
            map_pts.emplace_back(2.0, 0.5, 0.5); // 有效点 2

            std::vector<Eigen::Vector3f> filtered;
            const uint64_t rev0 = mapper.revision();
            mapper.integrateFrame(map_pts, sp, Eigen::Quaterniond::Identity(), sm, false, nullptr, &filtered);
            const uint64_t rev1 = mapper.revision();
            mapper.integrateFrame(map_pts, sp, Eigen::Quaterniond::Identity(), sm, false, nullptr, &filtered);
            const uint64_t rev2 = mapper.revision();

            ok &= check(
                rev0 == 0 && rev1 > rev0 && rev2 > rev1 && filtered.size() == 2,
                "OnlineTerrainMapper filtered_cloud retains only valid in-range non-self points and bumps revision"
            );

            std::vector<Eigen::Vector3f> static_voxels;
            const uint64_t ext_rev = mapper.extractStaticVoxelCloud(
                Eigen::Vector3d(-1.0, -1.0, -0.5),
                Eigen::Vector3d(4.0, 1.0, 2.0),
                static_voxels,
                1000
            );
            std::vector<Eigen::Vector3f> capped_voxels;
            mapper.extractStaticVoxelCloud(
                Eigen::Vector3d(-1.0, -1.0, -0.5),
                Eigen::Vector3d(4.0, 1.0, 2.0),
                capped_voxels,
                1
            );
            std::vector<Eigen::Vector3f> out_box_voxels;
            mapper.extractStaticVoxelCloud(
                Eigen::Vector3d(10.0, 10.0, 0.0),
                Eigen::Vector3d(12.0, 12.0, 2.0),
                out_box_voxels,
                1000
            );
            ok &= check(
                ext_rev == rev2 && static_voxels.size() == 2 && capped_voxels.size() == 1 && out_box_voxels.empty(),
                "OnlineTerrainMapper extractStaticVoxelCloud honors AABB window and max_points cap"
            );

            mapper.reset();
            std::vector<Eigen::Vector3f> reset_voxels;
            const uint64_t reset_rev = mapper.extractStaticVoxelCloud(
                Eigen::Vector3d(-1.0, -1.0, -0.5),
                Eigen::Vector3d(4.0, 1.0, 2.0),
                reset_voxels,
                1000
            );
            ok &= check(
                reset_rev > rev2 && reset_voxels.empty(),
                "OnlineTerrainMapper reset bumps revision and clears static 3D voxels"
            );

            // 9i-2. DynamicStvlLayer: filtered_cloud 与 debug_cloud
            ma_map::DynamicStvlLayer::Config dc;
            dc.voxel_size = 0.05;
            dc.confirm_min_frames = 2;
            dc.fast_confirm_with_neighbor = true;
            dc.fallback_ground_offset = 0.0;
            ma_map::DynamicStvlLayer stvl(dc);
            ma_map::StaticNavContext empty_ctx;

            std::vector<Eigen::Vector3d> nav_pts;
            nav_pts.emplace_back(dNaN, 0.0, 0.0); // NaN：滤除
            nav_pts.emplace_back(0.05, 0.0, 0.2); // 自遮挡：滤除
            nav_pts.emplace_back(9.5, 0.0, 0.3); // 超 max_range：滤除
            for (double x = 2.0; x <= 2.15; x += 0.05) {
                for (double y = -0.05; y <= 0.05; y += 0.05) {
                    for (double z = 0.25; z <= 0.45; z += 0.05) {
                        nav_pts.emplace_back(x, y, z);
                    }
                }
            }
            const size_t valid_nav_pts = nav_pts.size() - 3;
            std::vector<Eigen::Vector3f> dyn_debug;
            std::vector<Eigen::Vector3f> nav_filtered;
            stvl.update(nav_pts, sp, Eigen::Quaterniond::Identity(), sm, empty_ctx, 1.0, &dyn_debug, &nav_filtered);
            ok &= check(
                nav_filtered.size() == valid_nav_pts && !dyn_debug.empty(),
                "DynamicStvlLayer outputs filtered 3D cloud and confirmed 3D dynamic voxels"
            );
            stvl.reset();
            stvl.update({}, sp, Eigen::Quaterniond::Identity(), sm, empty_ctx, 2.0, &dyn_debug, &nav_filtered);
            ok &= check(
                nav_filtered.empty() && dyn_debug.empty(),
                "DynamicStvlLayer clears filtered_cloud and debug_cloud on empty scan"
            );

            // 9i-3. LayeredMap::getStaticTerrainPrior 轻量快照与版本号
            const auto tmp_dir = std::filesystem::temp_directory_path() / "ma_nav_viz_prior_test";
            std::filesystem::create_directories(tmp_dir);
            const auto tmp_map_base = tmp_dir / "prior_viz_map";
            {
                ma_map::LayeredMap::Config lcfg;
                lcfg.x_left = 2.0;
                lcfg.x_right = 2.0;
                lcfg.y_left = 2.0;
                lcfg.y_right = 2.0;
                lcfg.resolution = 0.1;
                ma_map::LayeredMap lm(lcfg);

                uint64_t prior_rev_init = 999;
                const auto prior_init = lm.getStaticTerrainPrior(&prior_rev_init);
                ok &= check(
                    prior_init == nullptr && prior_rev_init == 0,
                    "LayeredMap getStaticTerrainPrior initially null with rev=0"
                );

                // 构建含地面+悬空的最小 .pgm/.yaml + .terrain.bin 并加载验证 revision 递增
                const auto geom = grid_map::GridGeometry::fromOriginAndSize(
                    Eigen::Vector2d(-2.0, -2.0),
                    Eigen::Vector2i(40, 40),
                    0.1
                );
                std::vector<ma_map::CellState> cells(geom.cellCount(), ma_map::CellState::FREE);
                auto prior = ma_map::StaticTerrainPrior::createEmpty(geom);
                for (int ix = 0; ix < 40; ++ix) {
                    for (int iy = 0; iy < 40; ++iy) {
                        prior.setGroundZ(ix, iy, 0.0f);
                    }
                }
                prior.setOverhangMinZ(20, 20, 1.2f);
                ma_map::SaveOptions sopts;
                sopts.crop_to_explored = false;
                sopts.save_terrain_prior = true;
                const auto saved = ma_map::MapIO::save(tmp_map_base.string(), geom, cells, sopts, &prior);
                const auto loaded =
                    saved ? lm.loadStaticMap(saved->yaml_path) : tl::make_unexpected(std::string("save failed"));
                uint64_t prior_rev_loaded = 0;
                const auto prior_loaded = lm.getStaticTerrainPrior(&prior_rev_loaded);
                ok &= check(
                    saved.has_value() && loaded.has_value() && loaded->has_terrain_prior && prior_loaded != nullptr
                        && prior_loaded->isValid() && prior_rev_loaded > 0
                        && std::isfinite(prior_loaded->overhangMinZAt(20, 20)),
                    "LayeredMap getStaticTerrainPrior returns valid snapshot and increments revision on load"
                );
            }
            std::filesystem::remove_all(tmp_dir);

            // 9i-4. MaMap 端到端按需采集：关闭时零采集，开启后可读取 filtered/dynamic_3d，关闭后立即清缓存
            const auto orig_mode = env.ma_map->mode();
            env.ma_map->set_global_map(grid_map::RowMatrixXi::Zero(env.ma_occ_a.rows(), env.ma_occ_a.cols()));
            env.ma_map->set_mode(ma_map::MapMode::NAVIGATION);
            env.ma_map->set_viz_capture(ma_map::MaMap::VizCaptureFlags {false, false});

            const rog_map::Pose pose_zero {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity()};
            rog_map::PointCloud pc;
            for (double x = 2.0; x <= 2.15; x += 0.05) {
                for (double y = -0.05; y <= 0.05; y += 0.05) {
                    for (double z = 0.25; z <= 0.45; z += 0.05) {
                        pcl::PointXYZI pt;
                        pt.x = static_cast<float>(x);
                        pt.y = static_cast<float>(y);
                        pt.z = static_cast<float>(z);
                        pt.intensity = 10.0f;
                        pc.push_back(pt);
                    }
                }
            }
            // 自遮挡点，开启过滤采集后也必须被剔除
            {
                pcl::PointXYZI self_pt;
                self_pt.x = 0.05f;
                self_pt.y = 0.0f;
                self_pt.z = 0.15f;
                self_pt.intensity = 10.0f;
                pc.push_back(self_pt);
            }
            const size_t expected_filtered_cnt = pc.size() - 1;

            const uint64_t f_rev_off0 = env.ma_map->filtered_cloud_revision();
            const uint64_t d_rev_off0 = env.ma_map->dynamic_3d_revision();
            env.ma_map->update_odom(pose_zero, 100.0);
            env.ma_map->update_cloud(pc, 100.0);
            env.ma_map->update_map();
            std::vector<Eigen::Vector3f> mm_filtered;
            std::vector<Eigen::Vector3f> mm_dyn;
            env.ma_map->copy_filtered_cloud(mm_filtered);
            env.ma_map->copy_dynamic_3d_cloud(mm_dyn);
            ok &= check(
                env.ma_map->filtered_cloud_revision() == f_rev_off0 && env.ma_map->dynamic_3d_revision() == d_rev_off0
                    && mm_filtered.empty() && mm_dyn.empty(),
                "MaMap unsubscribed viz capture keeps zero capture and empty debug caches"
            );

            env.ma_map->set_viz_capture(ma_map::MaMap::VizCaptureFlags {true, true});
            env.ma_map->update_odom(pose_zero, 100.1);
            env.ma_map->update_cloud(pc, 100.1);
            env.ma_map->update_map();
            const uint64_t f_rev_on = env.ma_map->copy_filtered_cloud(mm_filtered);
            const uint64_t d_rev_on = env.ma_map->copy_dynamic_3d_cloud(mm_dyn);
            ok &= check(
                f_rev_on > f_rev_off0 && d_rev_on > d_rev_off0 && mm_filtered.size() == expected_filtered_cnt
                    && !mm_dyn.empty(),
                "MaMap subscribed viz capture populates filtered_cloud and dynamic_3d"
            );

            env.ma_map->set_viz_capture(ma_map::MaMap::VizCaptureFlags {false, false});
            env.ma_map->copy_filtered_cloud(mm_filtered);
            env.ma_map->copy_dynamic_3d_cloud(mm_dyn);
            ok &= check(
                mm_filtered.empty() && mm_dyn.empty(),
                "MaMap disabling viz capture immediately clears cached debug clouds"
            );

            env.ma_map->set_mode(orig_mode);
            env.ma_map->set_global_map(env.ma_occ_a);
        }
    }

    return ok;
}

// ---------------- 注册各模块 Benchmark ----------------

auto register_benchmarks(BenchEnv* env) -> void {
    const auto nq = static_cast<int64_t>(env->queries.size());

    // ---- GridMap ----
    benchmark::RegisterBenchmark("GridMap/UpdateESDF_WithWorkspace", [env](benchmark::State& st) {
        grid_map::GridMap::EsdfWorkspace ws;
        for (auto _: st) {
            const bool changed = env->grid_dst.overwriteMapAndUpdateEsdf(env->ma_occ_a, &ws);
            benchmark::DoNotOptimize(changed);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("GridMap/UpdateESDF_NoWorkspace", [env](benchmark::State& st) {
        for (auto _: st) {
            const bool changed = env->grid_dst.overwriteMapAndUpdateEsdf(env->ma_occ_a, nullptr);
            benchmark::DoNotOptimize(changed);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("GridMap/EqualsOccupancy_Unchanged", [env](benchmark::State& st) {
        for (auto _: st) {
            const bool same = env->grid_src.equalsOccupancy(env->ma_occ_a);
            benchmark::DoNotOptimize(same);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMicrosecond);

    benchmark::RegisterBenchmark("GridMap/CopyOccAndEsdf", [env](benchmark::State& st) {
        for (auto _: st) {
            const bool copied = env->grid_dst.copyOccAndEsdfFrom(env->grid_src);
            benchmark::DoNotOptimize(copied);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMicrosecond);

    // ---- MaMap ----
    benchmark::RegisterBenchmark("MaMap/PublishUnchanged", [env](benchmark::State& st) {
        env->ma_map->set_global_map(env->ma_occ_a);
        for (auto _: st) {
            env->ma_map->set_global_map(env->ma_occ_a);
            const auto snap = env->ma_map->get_grid_map();
            const uint64_t rev = snap->revision();
            benchmark::DoNotOptimize(rev);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMicrosecond);

    benchmark::RegisterBenchmark("MaMap/PublishChanged", [env](benchmark::State& st) {
        env->ma_map->set_global_map(env->ma_occ_a);
        size_t toggle = 0;
        for (auto _: st) {
            env->ma_map->set_global_map((toggle++ & 1U) == 0U ? env->ma_occ_b : env->ma_occ_a);
            const auto snap = env->ma_map->get_grid_map();
            const uint64_t rev = snap->revision();
            benchmark::DoNotOptimize(rev);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("MaMap/SetSemanticsPolygon", [env](benchmark::State& st) {
        env->ma_map->set_global_map(env->ma_occ_a);
        env->ma_map->set_semantics_polygon(env->sem_polygon, grid_map::GridMap::Semantics::NONE);
        size_t toggle = 0;
        for (auto _: st) {
            const auto sem =
                (toggle++ & 1U) == 0U ? grid_map::GridMap::Semantics::TUNNEL : grid_map::GridMap::Semantics::NONE;
            env->ma_map->set_semantics_polygon(env->sem_polygon, sem);
            const auto snap = env->ma_map->get_grid_map();
            const uint64_t rev = snap->revision();
            benchmark::DoNotOptimize(rev);
        }
        env->ma_map->set_semantics_polygon(env->sem_polygon, grid_map::GridMap::Semantics::NONE);
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMicrosecond);

    benchmark::RegisterBenchmark("MaMap/GetSnapshot", [env](benchmark::State& st) {
        env->ma_map->set_global_map(env->ma_occ_a);
        for (auto _: st) {
            const auto snap = env->ma_map->get_grid_map();
            const uint64_t rev = snap->revision();
            const size_t n = snap->esdfData().size();
            benchmark::DoNotOptimize(rev);
            benchmark::DoNotOptimize(n);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kNanosecond);

    benchmark::RegisterBenchmark("MaMap/ConcurrentGetSnapshot", [env](benchmark::State& st) {
        env->ma_map->set_global_map(env->ma_occ_a);
        std::atomic<bool> stop {false};
        std::atomic<uint64_t> published {0};
        std::thread writer([&]() {
            size_t toggle = 0;
            while (!stop.load(std::memory_order_acquire)) {
                env->ma_map->set_global_map((toggle++ & 1U) == 0U ? env->ma_occ_b : env->ma_occ_a);
                published.fetch_add(1, std::memory_order_release);
            }
        });
        while (published.load(std::memory_order_acquire) == 0) {
            std::this_thread::yield();
        }

        for (auto _: st) {
            const auto snap = env->ma_map->get_grid_map();
            if (!snap) {
                st.SkipWithError("concurrent get_grid_map returned null");
                break;
            }
            const uint64_t rev = snap->revision();
            const size_t n = snap->esdfData().size();
            benchmark::DoNotOptimize(rev);
            benchmark::DoNotOptimize(n);
        }

        stop.store(true, std::memory_order_release);
        writer.join();
        st.SetItemsProcessed(st.iterations());
        st.counters["writer_publishes"] = static_cast<double>(published.load(std::memory_order_relaxed));
    })->Unit(benchmark::kNanosecond);

    // ---- PathPlanning ----
    benchmark::RegisterBenchmark("PathPlanning/AStar_CacheHit", [env, nq](benchmark::State& st) {
        path_planning::AStar astar;
        astar.set_safe_threshold(env->planner_cfg.path_planning_params.safe_threshold);
        // 预热缓存，稳定测量命中路径
        for (const auto& q: env->queries) {
            (void)astar.original_astar_search(*env->gm_a, q.start, q.goal, kTimeoutMs);
        }
        for (auto _: st) {
            for (const auto& q: env->queries) {
                auto path = astar.original_astar_search(*env->gm_a, q.start, q.goal, kTimeoutMs);
                benchmark::DoNotOptimize(path.data());
            }
        }
        st.SetItemsProcessed(st.iterations() * nq);
        st.counters["queries"] = static_cast<double>(nq);
    })->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("PathPlanning/JPS_CacheHit", [env, nq](benchmark::State& st) {
        path_planning::JPS jps;
        jps.set_safe_threshold(env->planner_cfg.path_planning_params.safe_threshold);
        for (const auto& q: env->queries) {
            (void)jps.jps_search(*env->gm_a, q.start, q.goal, kTimeoutMs);
        }
        for (auto _: st) {
            for (const auto& q: env->queries) {
                auto path = jps.jps_search(*env->gm_a, q.start, q.goal, kTimeoutMs);
                benchmark::DoNotOptimize(path.data());
            }
        }
        st.SetItemsProcessed(st.iterations() * nq);
        st.counters["queries"] = static_cast<double>(nq);
    })->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("PathPlanning/JPS_CacheRebuild", [env, nq](benchmark::State& st) {
        path_planning::JPS jps;
        jps.set_safe_threshold(env->planner_cfg.path_planning_params.safe_threshold);
        size_t toggle = 0;
        for (auto _: st) {
            for (const auto& q: env->queries) {
                const grid_map::GridMap& gm = (toggle++ & 1U) == 0U ? *env->gm_a : *env->gm_b;
                auto path = jps.jps_search(gm, q.start, q.goal, kTimeoutMs);
                benchmark::DoNotOptimize(path.data());
            }
        }
        st.SetItemsProcessed(st.iterations() * nq);
        st.counters["queries"] = static_cast<double>(nq);
    })->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("PathPlanning/Frontend_JPS_PostProc", [env, nq](benchmark::State& st) {
        path_planning::PathPlanning frontend;
        frontend.set_param(env->planner_cfg.path_planning_params);
        frontend.set_use_jps(true);
        frontend.set_velocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        for (auto _: st) {
            for (const auto& q: env->queries) {
                auto traj = frontend.path_planning(*env->gm_a, q.start, q.goal, 0.0, 0.0, kTimeoutMs);
                benchmark::DoNotOptimize(traj);
            }
        }
        st.SetItemsProcessed(st.iterations() * nq);
        st.counters["queries"] = static_cast<double>(nq);
    })->Unit(benchmark::kMillisecond);

    // ---- TrajOptimize ----
    benchmark::RegisterBenchmark("TrajOptimize/MaSpline_ScopedEsdf", [env, nq](benchmark::State& st) {
        ma_spline_opt::MaSplineTrajectoryOptimizer<minco_opt::GridMapESDF> opt;
        opt.set_config(env->planner_cfg.ma_spline_opt_params);
        for (auto _: st) {
            for (const auto& in: env->opt_inputs) {
                auto out = opt.optimize(in, *env->esdf);
                benchmark::DoNotOptimize(out.cost);
            }
        }
        st.SetItemsProcessed(st.iterations() * nq);
        st.counters["queries"] = static_cast<double>(nq);
    })->Unit(benchmark::kMillisecond);

    // ---- FsmReplan ----
    benchmark::RegisterBenchmark("FsmReplan/MonitorFastPath", [env](benchmark::State& st) {
        replan::FsmReplan fsm(env->planner_cfg);
        const auto init = fsm.plan(env->fsm_goal_a, env->fsm_start, env->gm_a);
        if (!init.has_value() || !init->is_new_trajectory) {
            st.SkipWithError("FsmReplan initial plan failed");
            return;
        }
        for (auto _: st) {
            const auto res = fsm.plan(env->fsm_goal_a, env->fsm_start, env->gm_a);
            if (!res.has_value() || res->is_new_trajectory) {
                st.SkipWithError("FsmReplan unexpectedly triggered replan on monitor path");
                break;
            }
            const int state_val = static_cast<int>(res->path_state);
            benchmark::DoNotOptimize(state_val);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMicrosecond);

    benchmark::RegisterBenchmark("FsmReplan/FullReplan", [env](benchmark::State& st) {
        replan::FsmReplan fsm(env->planner_cfg);
        size_t toggle = 0;
        for (auto _: st) {
            const utils::RobotState& goal = (toggle++ & 1U) == 0U ? env->fsm_goal_a : env->fsm_goal_b;
            const auto res = fsm.plan(goal, env->fsm_start, env->gm_a);
            if (!res.has_value() || !res->is_new_trajectory) {
                st.SkipWithError("FsmReplan full replan failed or skipped");
                break;
            }
            const double cost = res->ma_spline_traj.cost;
            benchmark::DoNotOptimize(cost);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMillisecond);

    // ---- Controller ----
    benchmark::RegisterBenchmark("Controller/Mpc_Solve", [env](benchmark::State& st) {
        control::Mpc<control::MaSplineTrajectoryInterface> mpc(env->mpc_param);
        mpc.set_trajectory(env->mpc_traj);
        size_t idx = 0;
        for (auto _: st) {
            const auto& sample = env->mpc_samples[idx % env->mpc_samples.size()];
            ++idx;
            control::InputVector u_cmd = control::InputVector::Zero();
            const bool solved = mpc.solve(sample.x0, sample.t_now, u_cmd);
            if (!solved) {
                st.SkipWithError("MPC solve failed");
                break;
            }
            benchmark::DoNotOptimize(u_cmd.data());
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMicrosecond);

    // ---- Perception (新感知后端 15k 点实测单帧延迟) ----
    benchmark::RegisterBenchmark("Perception/DynamicStvl_Update_15kPts", [env](benchmark::State& st) {
        ma_map::SensorModel::Config scfg;
        scfg.t_base_lidar = Eigen::Vector3d(0.0, 0.0, 0.194);
        scfg.rpy_deg = Eigen::Vector3d(0.0, -20.0, 0.0);
        ma_map::SensorModel sm(scfg);
        ma_map::DynamicStvlLayer stvl;

        const auto snap = env->gm_a;
        const auto geom = snap->geometry();
        auto prior = std::make_shared<ma_map::StaticTerrainPrior>(ma_map::StaticTerrainPrior::createEmpty(geom));
        std::fill(prior->ground_z.begin(), prior->ground_z.end(), 0.0f);
        ma_map::StaticNavContext sctx;
        sctx.static_esdf = snap;
        sctx.terrain_prior = prior;
        sctx.current_snapshot = snap;

        const Eigen::Vector3d p_base(6.0, 4.0, 0.0);
        const Eigen::Quaterniond q_base = Eigen::Quaterniond::Identity();
        const ma_map::SensorPose sp = sm.computePose(p_base, q_base);

        // 构造 15,000 点真实密度 Mid-360 帧（地面 + 周边结构 + 移动动态障碍物）
        std::vector<Eigen::Vector3d> pts;
        pts.reserve(15000);
        for (int i = 0; i < 15000; ++i) {
            const double az = (static_cast<double>(i % 360) * 3.14159265358979323846) / 180.0;
            const double r = 0.6 + 0.18 * static_cast<double>((i / 360) % 40);
            const double z = (i % 15 == 0) ? 0.35 : 0.01 * std::sin(static_cast<double>(i));
            pts.emplace_back(p_base.x() + r * std::cos(az), p_base.y() + r * std::sin(az), z);
        }

        double t_sim = 0.0;
        for (auto _: st) {
            t_sim += 0.1;
            const auto fr = stvl.update(pts, sp, q_base, sm, sctx, t_sim);
            benchmark::DoNotOptimize(fr.obstacle_cells.size());
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("Perception/OnlineMapper_Integrate_15kPts", [](benchmark::State& st) {
        ma_map::SensorModel::Config scfg;
        scfg.t_base_lidar = Eigen::Vector3d(0.0, 0.0, 0.194);
        scfg.rpy_deg = Eigen::Vector3d(0.0, -20.0, 0.0);
        ma_map::SensorModel sm(scfg);
        ma_map::OnlineTerrainMapper mapper;

        const Eigen::Vector3d p_base(0.0, 0.0, 0.0);
        const Eigen::Quaterniond q_base = Eigen::Quaterniond::Identity();
        const ma_map::SensorPose sp = sm.computePose(p_base, q_base);

        std::vector<Eigen::Vector3d> pts;
        pts.reserve(15000);
        for (int i = 0; i < 15000; ++i) {
            const double az = (static_cast<double>(i % 360) * 3.14159265358979323846) / 180.0;
            const double r = 0.6 + 0.18 * static_cast<double>((i / 360) % 40);
            const double z = (r > 3.5) ? (0.1 + 0.02 * static_cast<double>(i % 30)) : 0.0;
            pts.emplace_back(r * std::cos(az), r * std::sin(az), z);
        }

        // 预热分配一次稀疏块，测稳态单帧集成 + 3D DDA 扫空 + 局部预览耗时
        (void)mapper.integrateFrame(pts, sp, q_base, sm, true);
        for (auto _: st) {
            const auto fr = mapper.integrateFrame(pts, sp, q_base, sm, true);
            benchmark::DoNotOptimize(fr);
        }
        st.SetItemsProcessed(st.iterations());
    })->Unit(benchmark::kMillisecond);
}

} // namespace

int main(int argc, char** argv) {
    // 提取自定义参数（不传给 Google Benchmark）
    bool sanity_only = false;
    std::vector<char*> gbench_argv;
    gbench_argv.reserve(static_cast<size_t>(argc));
    for (int i = 0; i < argc; ++i) {
        if (std::string(argv[i]) == "--sanity-only") {
            sanity_only = true;
        } else {
            gbench_argv.push_back(argv[i]);
        }
    }
    int gbench_argc = static_cast<int>(gbench_argv.size());

    // 静默内部日志，避免污染计时与 JSON 标准输出
    spdlog::set_level(spdlog::level::off);

    benchmark::Initialize(&gbench_argc, gbench_argv.data());
    if (benchmark::ReportUnrecognizedArguments(gbench_argc, gbench_argv.data())) {
        return 1;
    }

    BenchEnv env;
    try {
        env = build_env();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[BENCH INIT ERROR] %s\n", e.what());
        return 1;
    }

    if (!run_sanity_checks(env)) {
        std::fprintf(stderr, "[SANITY] FAILED: aborting before running benchmarks.\n");
        return 1;
    }
    std::fprintf(stderr, "[SANITY] PASS (GridMap, MaMap, PathPlanning, TrajOptimize, FsmReplan, Controller)\n");

    if (sanity_only) {
        return 0;
    }

    register_benchmarks(&env);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
