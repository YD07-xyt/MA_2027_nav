#pragma once
/**
 * 观测点选择 + 可见性几何（不持有状态，可单独测试）。
 *
 * 对应 Elastic-Tracker env.hpp 中的 findVisiblePath 停止条件 + visible_pair：
 *   1. 弹簧点 s = c + d_des·(robot - c)/|robot - c| 作为种子；
 *   2. 种子不安全/不可见时，按“到弹簧点距离”最小，在 [camera_min_dist, camera_max_dist] 的环上搜索
 *      安全且可见的点（替代原版的 6 邻域 A*，避免逐格搜索的开销）；
 *   3. 以选中点所在半径扫描可见扇形 [t_l, t_r]，离边缘不足 theta_clearance 时推向扇形内部。
 *
 * 暂时处理：遮挡按“2D 栅格地图中所有占据格都挡视线”计算，台阶、矮墙等低矮障碍也会被当成遮挡（偏保守）。
 *          若传入的是膨胀后的规划地图，遮挡会进一步偏保守。
 */
#include "map/core/grid_map.hpp"
#include "path_planning/target_tracking/tracking_types.h"
#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace path_planning::target_tracking {

/// 视线检测时忽略的圆形区域（目标本体会被 mid360 建成障碍，需从遮挡中排除）
struct IgnoreCircle {
    Eigen::Vector2d center;
    double radius;
};

class ViewpointSelector {
public:
    auto set_param(const TargetTrackingParams& params) -> void { params_ = params; }

    /**
     * @brief 视线 p0 → p1 是否畅通
     * los_clearance == 0：栅格 DDA 遍历，经过占据格即遮挡；
     * los_clearance  > 0：沿线采样 ESDF，距离 < los_clearance 即遮挡。
     * 地图外区域视为未知、不遮挡（与 GridMap::isCollision 的约定一致）。
     */
    auto line_of_sight(
        const grid_map::GridMap& map,
        const Eigen::Vector2d& p0,
        const Eigen::Vector2d& p1,
        const std::vector<IgnoreCircle>& ignore = {}
    ) const -> bool;

    /// 观测点自身是否安全：在地图内且 ESDF ≥ viewpoint_clearance
    auto is_safe(const grid_map::GridMap& map, const Eigen::Vector2d& p) const -> bool;

    /**
     * @brief 匀速外推目标轨迹，遇到障碍截断（之后停在最后的自由位置）
     *
     * 暂时处理：只做匀速外推 + 障碍截断，不预测目标绕障/转弯；
     *          后续可替换为 Elastic-Tracker 的加速度空间搜索（prediction.hpp）。
     *
     * @param p0, v0  t=0 时刻目标状态（最近一次有效观测时刻）
     * @param t0      第一个输出点对应的时刻（通常是“当前时间 - 观测时间”）
     * @return 时刻 t0, t0+dt, ..., t0+tracking_dur 的位置
     */
    auto predict(
        const grid_map::GridMap& map,
        const Eigen::Vector2d& p0,
        const Eigen::Vector2d& v0,
        double t0
    ) const -> std::vector<Eigen::Vector2d>;

    /**
     * @brief 在观测中心 center 周围选择观测点
     * @param center   观测中心（预测的目标位置）
     * @param robot    机器人当前位置（决定弹簧方向）
     * @param fallback_dir 机器人与中心重合时使用的弹簧方向 (rad)
     * @param ignore   视线检测忽略区域（目标本体）
     */
    auto select(
        const grid_map::GridMap& map,
        const Eigen::Vector2d& center,
        const Eigen::Vector2d& robot,
        double fallback_dir,
        const std::vector<IgnoreCircle>& ignore
    ) const -> ViewpointInfo;

private:
    /// 观测点候选是否可用：安全 + 到中心视线畅通
    auto valid_viewpoint(
        const grid_map::GridMap& map,
        const Eigen::Vector2d& p,
        const Eigen::Vector2d& center,
        const std::vector<IgnoreCircle>& ignore
    ) const -> bool;

    /// 以 center 为圆心、radius 为半径，从 angle 向两侧扫描可见扇形（同 visible_pair，只看视线）
    auto visible_arc(
        const grid_map::GridMap& map,
        const Eigen::Vector2d& center,
        double radius,
        double angle,
        const std::vector<IgnoreCircle>& ignore,
        double& t_l,
        double& t_r
    ) const -> void;

    TargetTrackingParams params_;
};

/// 角度归一化到 [-pi, pi]
inline auto normalize_angle(double a) -> double {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

} // namespace path_planning::target_tracking
