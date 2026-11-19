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
#include "map/grid_map.hpp"
#include "map/ma_map.hpp"
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
