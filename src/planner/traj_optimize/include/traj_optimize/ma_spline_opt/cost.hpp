#pragma once
#include "traj_optimize/esdf_Interface.hpp"
#include "SplineTrajectory/SplineOptimizer.hpp"


#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace ma_spline_opt {

template <int DIM, typename ESDF>
struct RobotIntegralCost
{
    const ESDF* esdf = nullptr;

    double w_obs = 8000.0;
    double safe_distance = 0.3;

    double w_vel = 40.0;
    double w_acc = 40.0;
    double v_max = 3.0;
    double a_max = 5.0;

    double w_yaw_vel = 10.0;
    double w_yaw_acc = 5.0;
    double yaw_rate_max = 1.0;
    double yaw_acc_max = 2.0;

    // false: stage1 使用完整 ESDF 梯度。true: stage2 去掉切向后再探测一步。
    bool split_esdf = false;

    static inline void smoothedL1(double x, double &f, double &df)
    {
        constexpr double pe = 1e-2;
        const double f3c = 1.0 / (pe * pe);
        const double f4c = -0.5 * f3c / pe;

        if (x <= 0.0)
        {
            f = 0.0;
            df = 0.0;
        }
        else if (x < pe)
        {
            f = (f4c * x + f3c) * x * x * x;
            df = (4.0 * f4c * x + 3.0 * f3c) * x * x;
        }
        else
        {
            f = x - 0.5 * pe;
            df = 1.0;
        }
    }

    double operator()(
        const SplineTrajectory::IntegralPointInfo& /*point*/,
        const Eigen::Matrix<double, DIM, 1>& p,
        const Eigen::Matrix<double, DIM, 1>& v,
        const Eigen::Matrix<double, DIM, 1>& a,
        const Eigen::Matrix<double, DIM, 1>& /*j*/,
        const Eigen::Matrix<double, DIM, 1>& /*s*/,
        Eigen::Matrix<double, DIM, 1>& gp,
        Eigen::Matrix<double, DIM, 1>& gv,
        Eigen::Matrix<double, DIM, 1>& ga,
        Eigen::Matrix<double, DIM, 1>& /*gj*/,
        Eigen::Matrix<double, DIM, 1>& /*gs*/,
        double& /*gt*/) const
    {
        gp.setZero();
        gv.setZero();
        ga.setZero();

        double cost = 0.0;

        if constexpr (DIM >= 2)
        {
            // ========== ESDF 避障 ==========
            Eigen::Vector2d pos = p.template head<2>();
            double d = 0.0;
            Eigen::Vector2d grad_dist = Eigen::Vector2d::Zero();

            // 单次查询同时取距离与梯度：
            // - FREE 越界策略：图外返回 false，不做避障惩罚；
            // - OCCUPIED 越界策略：图内含四边界虚拟墙距离，图外返回负距离与向内单位法向。
            if (esdf && esdf->getDistanceAndGradient(pos.x(), pos.y(), d, grad_dist)) {
                const double violation = safe_distance - d;
                const bool inside = esdf->isInside(pos.x(), pos.y());

                if (!split_esdf || !inside) {
                    double f, df;
                    smoothedL1(violation, f, df);
                    cost += w_obs * f;
                    gp.template head<2>() += w_obs * df * (-1.0) * grad_dist;
                }
                else if (violation > 0.0)
                {
                    // stage2：只在违反安全距离时介入。代价始终是真实违反量 safe - d，
                    // 势谷判断只改梯度大小，不改代价，避免代价跳变和穿墙时惩罚被压小。
                    // 去掉沿速度方向的分量，只保留横向推力。
                    Eigen::Vector2d grad_pos = grad_dist;
                    const Eigen::Vector2d vel = v.template head<2>();
                    const double vel_norm = vel.norm();
                    constexpr double kMinNorm = 1e-6;
                    if (vel_norm > kMinNorm)
                    {
                        const Eigen::Vector2d tangent = vel / vel_norm;
                        grad_pos -= grad_dist.dot(tangent) * tangent;
                    }

                    const double orth_norm = grad_pos.norm();
                    constexpr double kThreshold = 0.5;
                    if (orth_norm > kMinNorm)
                    {
                        const Eigen::Vector2d direction = grad_pos / orth_norm;
                        // 一步不超过一个栅格，差分才接近真实斜率。
                        const double step = std::max(esdf->resolution(), 1e-3);
                        const Eigen::Vector2d ahead = pos + direction * step;
                        if (esdf->isInside(ahead.x(), ahead.y()) || esdf->outOfBoundsOccupied()) {
                            const double d_ahead = esdf->getDistance(ahead.x(), ahead.y());
                            const double step_grad = (d_ahead - d) / step;
                            if (step_grad < kThreshold)
                            {
                                // 再走会进势谷：方向保持横向梯度，模长用 sqrt 增强，避免梯度消失。
                                grad_pos = direction * std::sqrt(orth_norm);
                            }
                            else
                            {
                                // 这一步是自由的：只把梯度归一到 1。
                                grad_pos = direction;
                            }
                        }
                    }
                    else
                    {
                        // 横向分量几乎为零：没有横向信息，退回完整 ESDF 梯度。
                        grad_pos = grad_dist;
                    }

                    double f, df;
                    smoothedL1(violation, f, df);
                    cost += w_obs * f;
                    gp.template head<2>() += w_obs * df * (-1.0) * grad_pos;
                }
            }

            // ========== 速度约束 ==========
            double v2 = v.template head<2>().squaredNorm();
            double f, df;
            smoothedL1(v2 - v_max * v_max, f, df);
            cost += w_vel * f;
            gv.template head<2>() += w_vel * df * 2.0 * v.template head<2>();

            // ========== 加速度约束 ==========
            double a2 = a.template head<2>().squaredNorm();
            smoothedL1(a2 - a_max * a_max, f, df);
            cost += w_acc * f;
            ga.template head<2>() += w_acc * df * 2.0 * a.template head<2>();
        }

        if constexpr (DIM == 3)
        {
            double yaw_rate = v[2];
            double yaw_acc = a[2];

            double f, df;
            smoothedL1(std::fabs(yaw_rate) - yaw_rate_max, f, df);
            cost += w_yaw_vel * f;
            gv[2] += w_yaw_vel * df * (yaw_rate > 0.0 ? 1.0 : -1.0);

            smoothedL1(std::fabs(yaw_acc) - yaw_acc_max, f, df);
            cost += w_yaw_acc * f;
            ga[2] += w_yaw_acc * df * (yaw_acc > 0.0 ? 1.0 : -1.0);
        }

        return cost;
    }
};

struct TimeCost
{
    double w_time = 20.0;
    double w_mean = 40.0;
    double mean_lower = 0.9;
    double mean_upper = 1.1;

    double w_min_time = 100.0;
    double min_time = 0.1;

    double operator()(
        const std::vector<double>& Ts,
        Eigen::VectorXd& grad) const
    {
        grad.setZero(Ts.size());

        double cost = 0.0;

        for (size_t i = 0; i < Ts.size(); ++i)
        {
            cost += w_time * Ts[i];
            grad[i] += w_time;

            // 最小时间约束
            if (Ts[i] < min_time)
            {
                double diff = Ts[i] - min_time;
                cost += w_min_time * diff * diff;
                grad[i] += 2.0 * w_min_time * diff;
            }
        }

        // 平均时间约束
        if (Ts.size() > 1)
        {
            double avg = 0.0;
            for (double T : Ts) avg += T;
            avg /= static_cast<double>(Ts.size());

            double lower = avg * mean_lower;
            double upper = avg * mean_upper;

            for (size_t i = 0; i < Ts.size(); ++i)
            {
                if (Ts[i] < lower)
                {
                    double diff = Ts[i] - lower;
                    cost += w_mean * diff * diff;

                    for (size_t j = 0; j < Ts.size(); ++j)
                    {
                        double d = (i == j ? 1.0 : 0.0) - mean_lower / Ts.size();
                        grad[j] += w_mean * 2.0 * diff * d;
                    }
                }
                else if (Ts[i] > upper)
                {
                    double diff = Ts[i] - upper;
                    cost += w_mean * diff * diff;

                    for (size_t j = 0; j < Ts.size(); ++j)
                    {
                        double d = (i == j ? 1.0 : 0.0) - mean_upper / Ts.size();
                        grad[j] += w_mean * 2.0 * diff * d;
                    }
                }
            }
        }

        return cost;
    }
};

} // namespace ma_spline_opt