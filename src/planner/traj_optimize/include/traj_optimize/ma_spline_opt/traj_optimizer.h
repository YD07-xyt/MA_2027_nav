#pragma once

#include "SplineTrajectory/SplineOptimizer.hpp"
#include "optimizer_config.h"
#include "cost.hpp"
#include "traj_optimize/esdf_Interface.hpp"
#include "utils/expected.hpp"
#include "utils/lbfgs.hpp"
#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ma_spline_opt {

// ============================================================================
// 主优化器
// ============================================================================
template <typename ESDF>
class MaSplineTrajectoryOptimizer {
public:
    using Opt2D = SplineTrajectory::SplineOptimizer<2>;
    using Opt3D = SplineTrajectory::SplineOptimizer<3>;

    void set_config(const MaSplineOptimizerConfig& config) {
        config_ = config;
    }

    void set_esdf_interface(const ESDF* esdf) {
        esdf_ = esdf;
    }

    // ========================================================================
    // 统一入口
    // ========================================================================
    auto optimize(const MaSplineInput& input) -> MAsplineOutput;
    // 同步只读借用 esdf，返回或异常退出前自动清空内部指针，避免悬挂指向临时适配器。
    auto optimize(const MaSplineInput& input, const ESDF& esdf) -> MAsplineOutput;

    auto check_trajectory_collision(const MAsplineOutput& out, const ESDF* esdf, double safe_distance)->bool;

private:
    enum MaOptError{
        TIMED_TRAJ_NULL,
        LBFGS_OPT_FAILED,
        SYNC_WORKING_STATE_FAILED,
    };
    using output= tl::expected<MAsplineOutput,MaOptError>;

private:
    SplineTrajectory::QuinticSplineND<2> last_xy_spline_;
    bool has_last_traj_ = false;

private:
    // ========================================================================
    // 2D xy 优化 L-BFGS 回调
    // ========================================================================
    struct CallbackCtx {
        Opt2D* optimizer = nullptr;
        Opt2D::OptimizationContext* ctx = nullptr;

        using Spec =
            decltype(Opt2D::makeEvaluateSpec(std::declval<TimeCost&>(), std::declval<RobotIntegralCost<2,ESDF>&>()));

        Spec* spec = nullptr;
    };

    static double cost_callback_2d(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        auto* c = static_cast<CallbackCtx*>(instance);
        return c->optimizer->evaluatePrepared(*c->ctx, x, g, *c->spec);
    }
    auto optimize_2d_stage(
        const SplineTrajectory::SplineOptimizer<2>::ProblemDefinition& problem,
        const StageOptimizerConfig& stage,
        bool split_esdf,
        Eigen::VectorXd& x,
        double& final_cost
    ) -> bool;
    // ========================================================================
    // 2D xy 轨迹优化
    // 直接使用 timed_trajectory，不降采样
    // 点密度由上游 time_resolution 控制
    // ========================================================================
    auto optimize_xy(const MaSplineInput& input) -> output;

private:
    struct CallbackCtx3D {
        Opt3D* optimizer = nullptr;
        Opt3D::OptimizationContext* ctx = nullptr;

        using Spec =
            decltype(Opt3D::makeEvaluateSpec(std::declval<TimeCost&>(), std::declval<RobotIntegralCost<3,ESDF>&>()));

        Spec* spec = nullptr;
    };
    static double cost_callback_3d(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
        auto* c = static_cast<CallbackCtx3D*>(instance);
        return c->optimizer->evaluatePrepared(*c->ctx, x, g, *c->spec);
    }
    auto optimize_3d_stage(
        const SplineTrajectory::SplineOptimizer<3>::ProblemDefinition& problem,
        const StageOptimizerConfig& stage,
        bool split_esdf,
        Eigen::VectorXd& x,
        double& final_cost
    ) -> bool;

    // ========================================================================
    // 联合优化 (x,y,yaw)
    // ========================================================================
    auto optimize_xy_yaw_joint(const MaSplineInput& input) -> output;

private:
    MaSplineOptimizerConfig config_;
    const ESDF* esdf_ = nullptr;

    Opt2D optimizer_2d_;
    Opt2D::OptimizationContext ctx_2d_;

    Opt3D optimizer_3d_;
    Opt3D::OptimizationContext ctx_3d_;
};

inline auto extend_xy_with_zero_yaw(const SplineTrajectory::QuinticSplineND<2>& xy_spline)
    -> SplineTrajectory::QuinticSplineND<3> {
    if (!xy_spline.isInitialized()) return {};

    const auto& xy_points = xy_spline.getSpacePoints();
    Eigen::Matrix<double, Eigen::Dynamic, 3> xy_yaw_points(xy_points.rows(), 3);
    xy_yaw_points.leftCols<2>() = xy_points;
    xy_yaw_points.col(2).setZero();

    const auto& bc2 = xy_spline.getBoundaryConditions();
    SplineTrajectory::BoundaryConditions<3> bc3;
    bc3.start_velocity << bc2.start_velocity.x(), bc2.start_velocity.y(), 0.0;
    bc3.start_acceleration << bc2.start_acceleration.x(), bc2.start_acceleration.y(), 0.0;
    bc3.end_velocity << bc2.end_velocity.x(), bc2.end_velocity.y(), 0.0;
    bc3.end_acceleration << bc2.end_acceleration.x(), bc2.end_acceleration.y(), 0.0;

    return SplineTrajectory::QuinticSplineND<3>(
        xy_spline.getTimeSegments(),
        xy_yaw_points,
        xy_spline.getStartTime(),
        bc3
    );
}

template <typename ESDF>
auto MaSplineTrajectoryOptimizer<ESDF>::check_trajectory_collision(
    const MAsplineOutput& out,
    const ESDF* esdf,
    double safe_distance
) ->bool {
    if (!out.success || !out.trajectory.isInitialized()) return false;

    const auto& traj = out.trajectory.getTrajectory();

    for (double t = traj.getStartTime(); t <= traj.getEndTime() + 1e-6; t += 0.05) {
        Eigen::Vector2d p = traj.evaluate(t, 0).head<2>();

        if (esdf && esdf->isInside(p.x(), p.y())) {
            if (esdf->getDistance(p.x(), p.y()) < safe_distance) return false;
        }
    }

    return true;
}
template<typename ESDF>
auto MaSplineTrajectoryOptimizer<ESDF>::optimize(const MaSplineInput& input, const ESDF& esdf) -> MAsplineOutput {
    struct ScopedEsdfReset {
        const ESDF*& slot;
        ~ScopedEsdfReset() {
            slot = nullptr;
        }
    };
    esdf_ = &esdf;
    ScopedEsdfReset reset_guard {esdf_};
    return optimize(input);
}

template <typename ESDF>
auto MaSplineTrajectoryOptimizer<ESDF>::optimize(const MaSplineInput& input) -> MAsplineOutput {
    switch (input.model) {
        case OMNI_XY_YAW_JOINT: {
            auto result = optimize_xy_yaw_joint(input);

            if (!result) {
                MAsplineOutput out;
                out.success = false;
                return out;
            }

            return std::move(result.value());
        }

        case OMNI_XY: {
            auto result = optimize_xy(input);

            if (!result) {
                MAsplineOutput out;
                out.success = false;
                return out;
            }

            return std::move(result.value());
        }
    }

    return MAsplineOutput {};
}

// ========================================================================
// 2D xy 轨迹优化
// 直接使用 timed_trajectory，不降采样
// 点密度由上游 time_resolution 控制
// ========================================================================
template <typename ESDF>
auto MaSplineTrajectoryOptimizer<ESDF>::optimize_xy(const MaSplineInput& input) -> output {
    MAsplineOutput out;

    const auto& timed = input.timed_trajectory;
    if (timed.size() < 2) {
        logger::warn(logger::traj_opt, "timed tarj null");
        return tl::make_unexpected(MaOptError::TIMED_TRAJ_NULL);
    }
    // 构造 problem
    std::vector<double> time_points;
    Eigen::Matrix<double, Eigen::Dynamic, 2> waypoints(timed.size(), 2);
    for (size_t i = 0; i < timed.size(); ++i) {
        time_points.push_back(timed[i].t);
        waypoints.row(i) = timed[i].pos.transpose();
    }

    SplineTrajectory::BoundaryConditions<2> bc;
    bc.start_velocity = input.start_vel;
    bc.end_velocity = input.end_vel;
    bc.start_acceleration = input.start_acc;
    bc.end_acceleration = input.end_acc;

    const int N = static_cast<int>(timed.size()) - 1;

    SplineTrajectory::OptimizationMask mask;
    mask.time.assign(N, 1);
    mask.waypoints.assign(N + 1, 1);
    mask.waypoints.front() = 0;
    mask.waypoints.back() = 0;

    auto problem = Opt2D::makeProblemFromTimePoints(time_points, waypoints, bc, mask);
    Eigen::VectorXd x;

    double cost_stage1 = 0.0;
    if (!optimize_2d_stage(problem, config_.stage1, false, x, cost_stage1)) {
        logger::warn(logger::traj_opt, "lbfgs opt1 failed");
        return tl::make_unexpected(MaOptError::LBFGS_OPT_FAILED);
    }

    const Eigen::VectorXd x_stage1 = x;
    double cost_stage2 = 0.0;
    if (!optimize_2d_stage(problem, config_.stage2, true, x, cost_stage2)) {
        logger::warn(logger::traj_opt, "lbfgs opt2 failed, keep stage1");
        x = x_stage1;
        cost_stage2 = cost_stage1;
    }

    // 同步最终结果
    auto status = optimizer_2d_.synchronizeWorkingState(ctx_2d_, x);
    if (!status) {
        logger::warn(logger::traj_opt, "synchronize owrk state failed");
        return tl::make_unexpected(MaOptError::SYNC_WORKING_STATE_FAILED);
    }
    SplineTrajectory::QuinticSplineND<2> xy_spline = optimizer_2d_.getWorkingSpline(ctx_2d_);
    out.trajectory = extend_xy_with_zero_yaw(xy_spline);
    out.time_segments = ctx_2d_.runtime.state.times;
    out.start_time = ctx_2d_.runtime.state.start_time;
    out.cost = cost_stage2;
    out.success = out.trajectory.isInitialized();

    return out;
}

template <typename ESDF>
auto MaSplineTrajectoryOptimizer<ESDF>::optimize_2d_stage(
    const SplineTrajectory::SplineOptimizer<2>::ProblemDefinition& problem,
    const StageOptimizerConfig& stage,
    bool split_esdf,
    Eigen::VectorXd& x,
    double& final_cost
) -> bool {
    Opt2D::OptimizerConfig opt_cfg1;
    opt_cfg1.rho_energy = stage.rho_energy;
    opt_cfg1.integral_num_steps = config_.integral_num_steps;

    optimizer_2d_.setConfig(opt_cfg1);
    auto status1 = optimizer_2d_.prepareContext(problem, ctx_2d_);
    if (!status1) {
        return false;
    }
    if (x.size() == 0) {
        x = optimizer_2d_.generateInitialGuess(ctx_2d_);
    } else if (x.size() != optimizer_2d_.getDimension(ctx_2d_) || !x.allFinite()) {
        return false;
    }

    TimeCost time_cost1;
    time_cost1.w_time = stage.weight_time;
    time_cost1.w_mean = stage.weight_mean_time;
    time_cost1.mean_lower = stage.mean_lower;
    time_cost1.mean_upper = stage.mean_upper;
    time_cost1.w_min_time = stage.weight_min_time;
    time_cost1.min_time = stage.min_time;

    RobotIntegralCost<2, ESDF> integral_cost1;
    integral_cost1.esdf = esdf_;
    integral_cost1.split_esdf = split_esdf;
    integral_cost1.w_obs = stage.weight_obstacle;
    integral_cost1.safe_distance = config_.safe_distance;
    integral_cost1.w_vel = stage.weight_vel;
    integral_cost1.w_acc = stage.weight_acc;
    integral_cost1.v_max = config_.v_max;
    integral_cost1.a_max = config_.a_max;

    auto spec1 = Opt2D::makeEvaluateSpec(time_cost1, integral_cost1);
    CallbackCtx cb {&optimizer_2d_, &ctx_2d_, &spec1};

    lbfgs::lbfgs_parameter_t params;
    params.max_iterations = stage.max_iterations;
    params.g_epsilon = stage.g_epsilon;
    params.mem_size = stage.lbfgs_mem_size;
    params.past = 3;
    params.delta = stage.lbfgs_delta;
    params.min_step = 1e-32;

    const int result = lbfgs::lbfgs_optimize(
        x,
        final_cost,
        &MaSplineTrajectoryOptimizer::cost_callback_2d,
        nullptr,
        nullptr,
        &cb,
        params
    );
    if(result<0){
        logger::error(logger::traj_opt, "L-BFGS failed: {}",lbfgs::lbfgs_strerror(result));
    }
    return result >= 0 && std::isfinite(final_cost) && x.allFinite();
};

template <typename ESDF>
auto MaSplineTrajectoryOptimizer<ESDF>::optimize_3d_stage(
    const SplineTrajectory::SplineOptimizer<3>::ProblemDefinition& problem,
    const StageOptimizerConfig& stage,
    bool split_esdf,
    Eigen::VectorXd& x,
    double& final_cost
) -> bool {
    Opt3D::OptimizerConfig opt_cfg;
    opt_cfg.rho_energy = stage.rho_energy;
    opt_cfg.integral_num_steps = config_.integral_num_steps;

    optimizer_3d_.setConfig(opt_cfg);
    auto status = optimizer_3d_.prepareContext(problem, ctx_3d_);

    if (!status) return false;

    if (x.size() == 0) {
        x = optimizer_3d_.generateInitialGuess(ctx_3d_);
    } else if (x.size() != optimizer_3d_.getDimension(ctx_3d_) || !x.allFinite()) {
        return false;
    }

    //使用 RobotIntegralCost<3>
    /**p.head<2>() 用于 ESDF；
        v.head<2>() 用于 XY 速度约束；
        a.head<2>() 用于 XY 加速度约束；
        v[2] 用于 yaw rate；
        a[2] 用于 yaw acceleration。 
    */
    RobotIntegralCost<3, ESDF> integral_cost;

    integral_cost.esdf = esdf_;
    integral_cost.split_esdf = split_esdf;
    integral_cost.w_obs = stage.weight_obstacle;
    integral_cost.safe_distance = config_.safe_distance;

    integral_cost.w_vel = stage.weight_vel;
    integral_cost.w_acc = stage.weight_acc;
    integral_cost.v_max = config_.v_max;
    integral_cost.a_max = config_.a_max;

    integral_cost.w_yaw_vel = config_.weight_yaw_vel;

    integral_cost.w_yaw_acc = config_.weight_yaw_acc;

    integral_cost.yaw_rate_max = config_.yaw_rate_max;

    integral_cost.yaw_acc_max = config_.yaw_acc_max;

    //时间代价保持不变
    TimeCost time_cost;

    time_cost.w_time = stage.weight_time;
    time_cost.w_mean = stage.weight_mean_time;
    time_cost.mean_lower = stage.mean_lower;
    time_cost.mean_upper = stage.mean_upper;
    time_cost.w_min_time = stage.weight_min_time;
    time_cost.min_time = stage.min_time;

    auto spec = Opt3D::makeEvaluateSpec(time_cost, integral_cost);

    CallbackCtx3D callback_ctx;
    callback_ctx.optimizer = &optimizer_3d_;
    callback_ctx.ctx = &ctx_3d_;
    callback_ctx.spec = &spec;

    lbfgs::lbfgs_parameter_t params;
    params.max_iterations = stage.max_iterations;
    params.g_epsilon = stage.g_epsilon;
    params.mem_size = stage.lbfgs_mem_size;
    params.past = 3;
    params.delta = stage.lbfgs_delta;
    params.min_step = 1e-32;

    const int result = lbfgs::lbfgs_optimize(
        x,
        final_cost,
        &MaSplineTrajectoryOptimizer::cost_callback_3d,
        nullptr,
        nullptr,
        &callback_ctx,
        params
    );

    return result >= 0 && std::isfinite(final_cost) && x.allFinite();
}

template <typename ESDF>
auto MaSplineTrajectoryOptimizer<ESDF>::optimize_xy_yaw_joint(const MaSplineInput& input) -> output {
    MAsplineOutput out;

    //构造三维 waypoint
    const auto& timed = input.timed_trajectory;

    if (timed.size() < 2) {
        return tl::make_unexpected(MaOptError::TIMED_TRAJ_NULL);
    }
    std::vector<double> time_points;
    time_points.reserve(timed.size());

    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints(timed.size(), 3);

    const auto yaw_unwrapped = build_unwrapped_yaw(timed);

    for (size_t i = 0; i < timed.size(); ++i) {
        time_points.push_back(timed[i].t);
        waypoints(i, 0) = timed[i].pos.x();
        waypoints(i, 1) = timed[i].pos.y();
        waypoints(i, 2) = yaw_unwrapped[i];
    }

    //构造三维边界条件
    SplineTrajectory::BoundaryConditions<3> bc;

    bc.start_velocity << input.start_vel.x(), input.start_vel.y(), input.start_yaw_rate;

    bc.start_acceleration << input.start_acc.x(), input.start_acc.y(), input.start_yaw_acc;

    bc.end_velocity << input.end_vel.x(), input.end_vel.y(), input.end_yaw_rate;

    bc.end_acceleration << input.end_acc.x(), input.end_acc.y(), input.end_yaw_acc;

    //构造优化 mask
    const int N = static_cast<int>(timed.size()) - 1;

    SplineTrajectory::OptimizationMask mask;
    mask.time.assign(N, 1);
    mask.waypoints.assign(N + 1, 1);

    // 起点、终点固定
    mask.waypoints.front() = 0;
    mask.waypoints.back() = 0;

    //构建问题
    auto problem = Opt3D::makeProblemFromTimePoints(time_points, waypoints, bc, mask);
    Eigen::VectorXd x;

    double cost_stage1 = 0.0;
    if (!optimize_3d_stage(problem, config_.stage1, false, x, cost_stage1)) {
        return tl::make_unexpected(MaOptError::LBFGS_OPT_FAILED);
    }

    const Eigen::VectorXd x_stage1 = x;
    double cost_stage2 = 0.0;
    if (!optimize_3d_stage(problem, config_.stage2, true, x, cost_stage2)) {
        logger::warn(logger::traj_opt, "lbfgs opt2 failed, keep stage1");
        x = x_stage1;
        cost_stage2 = cost_stage1;
    }

    auto status = optimizer_3d_.synchronizeWorkingState(ctx_3d_, x);
    if (!status) {
        return tl::make_unexpected(MaOptError::SYNC_WORKING_STATE_FAILED);
    }
    out.trajectory = optimizer_3d_.getWorkingSpline(ctx_3d_);
    out.time_segments = ctx_3d_.runtime.state.times;
    out.start_time = ctx_3d_.runtime.state.start_time;
    out.cost = cost_stage2;
    out.success = out.trajectory.isInitialized();
    return out;
}
} // namespace ma_spline_opt