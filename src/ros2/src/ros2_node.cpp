#include "ros2/ros2_node.h"
#include <fstream>
#include "map/core/grid_map.hpp"
#include "controller/mpc.h"
#include "replan/fsm_replanner.h"
#include "utils/logger.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace planner {
GlobalPlanner2d::GlobalPlanner2d(rclcpp::Node::SharedPtr nh_, std::string params_path):
    config(params_path),
    nh(nh_),
    frame_id(config.frame_id),
    mapInitialized(false),
    visualizer(nh_),
    //fsm_(config.fsm_config),
    ma_map_(std::make_shared<ma_map::MaMap>(config.map_params_path)),
    nav_state(replan::FsmReplan::PathState::IDLE),
    mpc_(config.mpc_params),
    fold_guard_(config.fold_params) {
    fsm_replanner.set_param(config.planner_config);

    // 时间源：use_sim_time=true 时全程跟随 /clock（仿真/回放）；false 使用 steady_clock（真机）。
    if (!nh->has_parameter("use_sim_time")) {
        nh->declare_parameter<bool>("use_sim_time", false);
    }
    nh->get_parameter("use_sim_time", use_sim_time_);
    if (use_sim_time_) {
        sim_clock_sub_ = nh->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock",
            rclcpp::ClockQoS(),
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                sim_clock_source_.storeNs(rclcpp::Time(msg->clock).nanoseconds());
            }
        );
        clock_ = sim_clock_source_.asClock();
        logger::info(logger::ros2, "use_sim_time=true, clock follows /clock");
    } else {
        clock_ = utils::Clock::steady();
    }
    ma_map_->set_clock(clock_);

    map_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    planner_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 2. 订阅者配置选项
    rclcpp::SubscriptionOptions opts_map;
    opts_map.callback_group = map_cb_group_;
    rclcpp::SubscriptionOptions opts_odom;
    opts_odom.callback_group = map_cb_group_; // 里程计/建图位姿也更新地图，放入地图组避免同时写
    rclcpp::SubscriptionOptions opts_target;
    opts_target.callback_group = control_cb_group_; // 目标点很轻量，放控制组

    // 3. 创建订阅者（绑定回调组）
    map_sub_ = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        config.map_topic_name,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { GlobalPlanner2d::map_callback(msg); },
        opts_map
    );

    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
        config.odom_topic_name,
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { GlobalPlanner2d::odom_callback(msg); },
        opts_odom
    );

    // 建图位姿可选走独立话题（如 /lio/odom：与 cloud 同一后验状态/时间戳）；控制仍用 odom_topic_name。
    if (!config.map_odom_topic_name.empty()) {
        map_odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
            config.map_odom_topic_name,
            rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { GlobalPlanner2d::map_odom_callback(msg); },
            opts_odom
        );
    }
    logger::info(
        logger::ros2,
        "map odom topic: {} (control odom topic: {})",
        config.map_odom_topic_name.empty() ? config.odom_topic_name : config.map_odom_topic_name,
        config.odom_topic_name
    );

    target_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
        config.target_topic_name,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { target_callback(msg); },
        opts_target
    );

    fold_feedback_sub_ = nh->create_subscription<std_msgs::msg::Bool>(
        config.fold_feedback_topic_name,
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { fold_feedback_callback(msg); },
        opts_target
    );

    // 4. 定时器（同样绑定回调组）
    planner_timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(33),
        [this]() { planner_callback(); },
        planner_cb_group_ // 规划器独立组
    );

    controller_timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() { controller_callback(); },
        control_cb_group_ // 控制器独立组
    );

    pub_viz_timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { pub_callback(); },
        map_cb_group_ // 可视化与地图/里程计同组串行，绝不阻塞 100Hz MPC 控制回调
    );

    // 接收 rviz2 "Publish Point" 工具点选的点（geometry_msgs/PointStamped，默认 /clicked_point），
    // 每 4 个点连成一个封闭四边形区域并发布到 /ma_nav/clicked_regions 可视化
    clickedPoint_sub_ = nh->create_subscription<geometry_msgs::msg::PointStamped>(
        config.clickedPoint_topic_name,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { clicked_point_callback(msg); }
    );
    cmd_vel_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>(config.cmd_vel_name, 10);
    clicked_region_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/ma_nav/clicked_regions", 10);
    fold_pub_ = nh->create_publisher<std_msgs::msg::Bool>(config.fold_cmd_topic_name, 10);
    nav_feedback_pub_ = nh->create_publisher<std_msgs::msg::Int16>("/ma_nav/nav_feedback", 10);
    logger::info(
        logger::ros2,
        "fold topics: cmd={}, feedback={}",
        config.fold_cmd_topic_name,
        config.fold_feedback_topic_name
    );

    // 保存/加载全局地图的 ROS2 服务
    save_map_srv_ = nh->create_service<std_srvs::srv::Trigger>(
        config.save_map_srv_topic,
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
            auto saved = ma_map_->save_explored_map(config.save_global_map_path);
            if (saved) {
                logger::info(
                    logger::ros2,
                    "Saved global map {}x{} (res={:.3f}, origin=({:.2f},{:.2f}), occ={}, free={}, unk={}) -> {}",
                    saved->saved_geometry.voxel_num.y(),
                    saved->saved_geometry.voxel_num.x(),
                    saved->saved_geometry.resolution,
                    saved->saved_geometry.origin.x(),
                    saved->saved_geometry.origin.y(),
                    saved->occupied_count,
                    saved->free_count,
                    saved->unknown_count,
                    saved->pgm_path
                );
                res->success = true;
                res->message = "Global map saved to " + saved->pgm_path;
            } else {
                logger::error(logger::ros2, "Failed to save global map: {}", saved.error());
                res->success = false;
                res->message = saved.error();
            }
        }
    );
    save_tunnel_regions_srv_ = nh->create_service<std_srvs::srv::Trigger>(
        config.save_tunnel_regions_srv_topic,
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
            save_clicked_regions(config.tunnel_regions_path);
            res->success = true;
            res->message = "Tunnel regions saved to " + config.tunnel_regions_path;
        }
    );
    ma_map_->set_mapping_model(config.mapping_model);
    if (!config.mapping_model) {
        load_global_map(config.global_map_path);
        load_clicked_regions(config.tunnel_regions_path);
    }
}

auto GlobalPlanner2d::get_current_pose() const -> std::optional<utils::RobotState> {
    std::lock_guard<std::mutex> lock(pose_mtx_);
    return current_pose;
}

auto GlobalPlanner2d::get_current_xytheta() const -> std::optional<Eigen::Vector3d> {
    std::lock_guard<std::mutex> lock(pose_mtx_);
    return current_XYTheta;
}

void GlobalPlanner2d::set_current_state(const utils::RobotState& pose, const Eigen::Vector3d& xytheta) {
    std::lock_guard<std::mutex> lock(pose_mtx_);
    current_pose = pose;
    current_XYTheta = xytheta;
}

auto GlobalPlanner2d::get_goal_pose() const -> std::optional<utils::RobotState> {
    std::lock_guard<std::mutex> lock(nav_task_mtx_);
    return goal_pose;
}

void GlobalPlanner2d::finish_nav_task(replan::FsmReplan::PathState state) {
    {
        std::lock_guard<std::mutex> lock(nav_task_mtx_);
        nav_state = state;
        goal_pose.reset();
        plan_start_time_sec_.reset();
    }
    nav_feedback_pub_->publish(pub_nav_feedback());
}

void GlobalPlanner2d::fold_feedback_callback(const std_msgs::msg::Bool::SharedPtr& msg) {
    fold_guard_.on_feedback(msg->data, clock_.nowSec());
}

void GlobalPlanner2d::publish_fold_cmd_if_due(const control::FoldDecision& decision, double now_sec) {
    const double period = 1.0 / std::max(config.fold_params.cmd_pub_rate, 1e-3);
    const bool heartbeat_due = !last_fold_cmd_pub_sec_.has_value() || now_sec < last_fold_cmd_pub_sec_.value()
        || (now_sec - last_fold_cmd_pub_sec_.value()) >= period;
    if (decision.cmd_changed || heartbeat_due) {
        std_msgs::msg::Bool fold_msg;
        fold_msg.data = decision.fold_cmd;
        fold_pub_->publish(fold_msg);
        last_fold_cmd_pub_sec_ = now_sec;
        if (decision.cmd_changed) {
            logger::info(
                logger::ros2,
                "云台折叠指令切换: {} (confirmed_folded={}, feedback_stale={}, gate_active={})",
                decision.fold_cmd ? "FOLD" : "UNFOLD",
                decision.confirmed_folded,
                decision.feedback_stale,
                decision.gate_active
            );
        }
    }
}

void GlobalPlanner2d::controller_callback() {
    const auto current_xytheta = get_current_xytheta();
    const auto current_pose_snapshot = get_current_pose();
    if (!current_xytheta.has_value()) return;
    if (!current_pose_snapshot.has_value()) return;

    const double now_sec = clock_.nowSec();
    const Eigen::Vector2d pos(current_xytheta->x(), current_xytheta->y());

    // 1. 消费 planner 组下发的最新不可变轨迹快照（control 组独占更新 mpc_/t_track_/急停状态）
    std::shared_ptr<control::MaSplineTrajectoryInterface> incoming_traj;
    uint64_t incoming_gen = 0;
    {
        std::lock_guard<std::mutex> lock(traj_handoff_mtx_);
        if (pending_traj_gen_ != active_traj_gen_) {
            incoming_traj = pending_traj_;
            incoming_gen = pending_traj_gen_;
        }
    }
    if (incoming_gen != 0 && incoming_traj && incoming_traj->valid()) {
        ma_traj_interface_ = std::move(incoming_traj);
        active_traj_gen_ = incoming_gen;
        mpc_.set_trajectory(ma_traj_interface_);
        t_track_ = ma_traj_interface_->nearest_time(pos);
        if (ma_traj_interface_->brake_emergency()) {
            const auto brake_ms = static_cast<int64_t>(ma_traj_interface_->brake_duration() * 1000.0);
            emergency_stop_until_sec_ = now_sec + static_cast<double>(brake_ms) * 1e-3;
            logger::warn(logger::ros2, "brake distance not enough, emergency stop {} ms", brake_ms);
        } else {
            emergency_stop_until_sec_.reset();
        }
    }

    const bool has_valid_traj = ma_traj_interface_ && ma_traj_interface_->valid();

    // 2. 更新急停状态与轨迹跟踪游标（保证 FoldGuard 读取到与本周期一致的 t_track_）
    bool in_emergency_stop = false;
    if (has_valid_traj && emergency_stop_until_sec_.has_value()) {
        if (now_sec < emergency_stop_until_sec_.value()) {
            in_emergency_stop = true;
        } else {
            // 急停结束：跳过刹车段，从样条上离机器人最近的点开始跟踪
            emergency_stop_until_sec_.reset();
            t_track_ = ma_traj_interface_->nearest_time_from(pos, ma_traj_interface_->brake_duration());
        }
    }

    const double dt_ctrl =
        last_control_time_sec_.has_value() ? std::clamp(now_sec - last_control_time_sec_.value(), 0.0, 0.05) : 0.0;
    last_control_time_sec_ = now_sec;
    if (has_valid_traj && !in_emergency_stop) {
        // 刹车段内按时间推进游标（反向样条与刹车段位置重合，按位置搜索会卡在刹停点附近）；之后按位置更新
        if (t_track_ < ma_traj_interface_->brake_duration()) {
            t_track_ += dt_ctrl;
        } else {
            t_track_ = ma_traj_interface_->update_track_time(pos, t_track_);
        }
    }

    // 3. 闭环云台折叠状态机与虚拟门禁（无轨迹/急停期间照常运行，保障电平心跳、出洞抬升与洞口锁定）
    const auto fold_map = ma_map_->get_grid_map();
    const auto fold_decision =
        fold_guard_.update(fold_map.get(), has_valid_traj ? ma_traj_interface_.get() : nullptr, t_track_, pos, now_sec);
    publish_fold_cmd_if_due(fold_decision, now_sec);

    const auto warn_fold_gate_throttled = [&]() {
        if (now_sec < last_fold_warn_sec_ || (now_sec - last_fold_warn_sec_) >= 1.0) {
            last_fold_warn_sec_ = now_sec;
            logger::warn(
                logger::ros2,
                "云台门禁限速/停靠等待: feedback_stale={}, fold_timeout={}, max_speed={:.2f} m/s",
                fold_decision.feedback_stale,
                fold_decision.fold_timeout,
                fold_decision.max_speed
            );
        }
    };
    if (fold_decision.fold_timeout || (fold_decision.gate_active && fold_decision.feedback_stale)) {
        warn_fold_gate_throttled();
    }

    if (!has_valid_traj) {
        return;
    }

    // 急停或门禁硬截停（身处禁入区未到位 / 已抵达停车缓冲边界）：直接发零速（不经过 MPC）
    if (in_emergency_stop || (fold_decision.gate_active && fold_decision.max_speed <= 1e-6)) {
        if (!in_emergency_stop) {
            warn_fold_gate_throttled();
        }
        geometry_msgs::msg::Twist zero;
        cmd_vel_pub_->publish(zero);
        last_vx_world_cmd_ = 0.0;
        last_vy_world_cmd_ = 0.0;
        has_last_mpc_cmd_ = true;
        return;
    }

    const double t_now = t_track_;

    // yaw 只用于速度坐标变换
    const double yaw = current_xytheta->z();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    // 当前 MPC 状态速度：优先用上一帧 MPC 指令，避免里程计噪声/超速导致 infeasible
    double vx_world_state = 0.0;
    double vy_world_state = 0.0;

    if (has_last_mpc_cmd_) {
        // 使用上一帧 MPC 的 world 速度指令
        vx_world_state = last_vx_world_cmd_;
        vy_world_state = last_vy_world_cmd_;
    } else {
        // 第一帧：用里程计速度，但限幅到合理范围，避免一开始就 infeasible
        // Super-LIO /lio/base_odom 的 twist.linear 是世界系速度（ESKF 中 v 由 R*acc+g 积分），
        // child_frame_id=base_link 只是标注，不需要再按 yaw 旋转。
        const double vx_world_odom = current_pose_snapshot->v.x();
        const double vy_world_odom = current_pose_snapshot->v.y();

        // 限幅到 MPC 速度约束内
        constexpr double kMaxVel = 3.0;
        vx_world_state = std::clamp(vx_world_odom, -kMaxVel, kMaxVel);
        vy_world_state = std::clamp(vy_world_odom, -kMaxVel, kMaxVel);
    }

    // 4 状态 MPC: [x, y, vx, vy]
    control::StateVector x0;
    x0 << current_xytheta->x(), current_xytheta->y(), vx_world_state, vy_world_state;

    control::InputVector u_cmd;
    std::vector<control::StateVector> predicted_states;
    std::vector<control::InputVector> predicted_inputs;

    if (!mpc_.solve(x0, t_now, u_cmd, predicted_states, predicted_inputs)) {
        // 调试时打印 x0，确认是不是速度导致 infeasible
        logger::warn(logger::ros2, "MPC solve failed, x0=[{:.3f}, {:.3f}, {:.3f}, {:.3f}]", x0(0), x0(1), x0(2), x0(3));
        return;
    }

    // 用预测的下一时刻状态作为速度指令
    if (predicted_states.size() < 2) {
        return;
    }

    // 4 状态索引: 0=x, 1=y, 2=vx, 3=vy；施加洞口虚拟门禁制动速度包络并回写给下一帧 MPC x0
    Eigen::Vector2d vel_world(predicted_states[1](2), predicted_states[1](3));
    if (control::FoldGuard::apply_velocity_gate(vel_world, fold_decision)) {
        warn_fold_gate_throttled();
    }
    last_vx_world_cmd_ = vel_world.x();
    last_vy_world_cmd_ = vel_world.y();
    has_last_mpc_cmd_ = true;

    // world -> body 输出
    geometry_msgs::msg::Twist twist;
    twist.linear.x = c * vel_world.x() + s * vel_world.y();
    twist.linear.y = -s * vel_world.x() + c * vel_world.y();
    twist.angular.z = 0.0; // 不控制 yaw

    // 异常保护
    if (twist.linear.x > 20 || twist.linear.y > 20 || twist.angular.z > 20) {
        logger::warn(logger::ros2, "x:{},y:{},w:{}", twist.linear.x, twist.linear.y, twist.angular.z);
        return;
    }
    if (twist.linear.x < -20 || twist.linear.y < -20 || twist.angular.z < -20) {
        logger::warn(logger::ros2, "x:{},y:{},w:{}", twist.linear.x, twist.linear.y, twist.angular.z);
        return;
    }
    if (std::isnan(twist.linear.x) || std::isnan(twist.linear.y) || std::isnan(twist.angular.z)) {
        logger::warn(logger::ros2, "x:{},y:{},w:{}", twist.linear.x, twist.linear.y, twist.angular.z);
        return;
    }

    cmd_vel_pub_->publish(twist);
}

void GlobalPlanner2d::planner_callback() {
    GlobalPlanner2d::plan_omni();
}
void GlobalPlanner2d::plan_omni() {
    const auto goal_pose_snapshot = get_goal_pose();
    if (!goal_pose_snapshot.has_value()) {
        logger::debug(logger::ros2, "no goal");
        return;
    }
    const auto current_pose_snapshot = get_current_pose();
    if (!current_pose_snapshot.has_value()) {
        logger::debug(logger::ros2, "no current_pose");
        return;
    }
    const double now = clock_.nowSec();
    double deadline = 0.0;
    {
        std::lock_guard<std::mutex> lock(nav_task_mtx_);
        if (!plan_start_time_sec_.has_value()) {
            plan_start_time_sec_ = now;
        }
        deadline = plan_start_time_sec_.value() + 20.0;
    }
    if (now >= deadline) {
        logger::warn(logger::ros2, "navigation task exceeded 20s -> nav:failed");
        finish_nav_task(replan::FsmReplan::PathState::FAILED);
        return;
    }

    const double dist_to_goal = (current_pose_snapshot->p.head<2>() - goal_pose_snapshot->p.head<2>()).norm();
    if (dist_to_goal < config.planner_config.replan_params.goal_reached_radius) {
        // 到达后收尾，避免重复触发
        finish_nav_task(replan::FsmReplan::PathState::SUCCESSED);
        return;
    }

    // 本次规划全程使用同一份地图快照（双缓冲），地图线程的更新不会改写它。
    const auto plan_map = ma_map_->get_grid_map();
    auto result = fsm_replanner.plan(goal_pose_snapshot.value(), current_pose_snapshot.value(), plan_map);
    const double finished_at = clock_.nowSec();
    if (finished_at >= deadline) {
        logger::warn(logger::ros2, "planning result arrived after 20s -> nav:failed");
        finish_nav_task(replan::FsmReplan::PathState::FAILED);
        return;
    }
    if (!result) {
        std::lock_guard<std::mutex> lock(nav_task_mtx_);
        nav_state = replan::FsmReplan::PathState::RUNNING;
        return;
    }
    auto path_result = result.value();

    // 规划失败保留当前任务，下一帧继续重试；20 秒期限由上方统一处理。
    if (path_result.path_state != replan::FsmReplan::PathState::SUCCESSED) {
        logger::debug(logger::ros2, "PathState not SUCCESSED, waiting...");
        std::lock_guard<std::mutex> lock(nav_task_mtx_);
        nav_state = replan::FsmReplan::PathState::RUNNING;
        return;
    }

    // 交接前再次检查，过期结果不能下发给控制器。
    if (clock_.nowSec() >= deadline) {
        logger::warn(logger::ros2, "planning result expired before handoff -> nav:failed");
        finish_nav_task(replan::FsmReplan::PathState::FAILED);
        return;
    }

    visualizer.PubGlobalPath(path_result.planning_traj.raw_path, frame_id);
    visualizer.PubOptPath(path_result.ma_spline_traj.trajectory, frame_id);

    std::vector<Eigen::Vector3d> route;
    for (auto point: path_result.planning_traj.optimized_path) {
        route.emplace_back(point.x(), point.y(), 0.0);
    }

    // 新轨迹下发：通过不可变快照原子交接给 control 组，由控制回调独占更新 MPC、游标与急停状态。
    if (path_result.is_new_trajectory && path_result.ma_spline_traj.success
        && path_result.ma_spline_traj.trajectory.isInitialized())
    {
        auto new_traj = std::make_shared<control::MaSplineTrajectoryInterface>(path_result.ma_spline_traj);
        {
            std::lock_guard<std::mutex> lock(traj_handoff_mtx_);
            pending_traj_ = std::move(new_traj);
            ++pending_traj_gen_;
        }

        // 可视化（保留隧道段高亮，不再输出旧静态折叠事件点）
        visualizer.visualizeTunnelAndFold(path_result.ma_spline_traj, route, *plan_map, {}, {}, frame_id);
    }
    auto msg = pub_nav_feedback();

    nav_feedback_pub_->publish(msg);
}
auto GlobalPlanner2d::pub_nav_feedback() -> std_msgs::msg::Int16 {
    std_msgs::msg::Int16 msg;
    replan::FsmReplan::PathState state;
    {
        std::lock_guard<std::mutex> lock(nav_task_mtx_);
        state = nav_state;
    }
    switch (state) {
        case replan::FsmReplan::PathState::IDLE:
            msg.data = 0;
            logger::info(logger::ros2, "nav_feedback:msg:IDLE");
            break;
        case replan::FsmReplan::PathState::RUNNING:
            msg.data = 1;
            //logger::info(logger::ros2, "nav_feedback:msg:RUNNING");
            break;
        case replan::FsmReplan::PathState::FAILED:
            msg.data = 2;
            logger::info(logger::ros2, "nav_feedback:msg:FAILED");
            break;
        case replan::FsmReplan::PathState::SUCCESSED:
            logger::info(logger::ros2, "nav_feedback:msg:SUCCESSED");
            msg.data = 3;
            break;
    }
    return msg;
};
// 输入消息坐标系与配置的全局 frame_id 不一致时告警一次（本节点不做 TF 变换，坐标按原值使用）
static void warn_frame_mismatch_once(
    bool& warned, const std::string& topic, const std::string& msg_frame, const std::string& expect
) {
    if (warned || msg_frame.empty() || msg_frame == expect) return;
    warned = true;
    logger::warn(
        logger::ros2,
        "Topic {} frame_id '{}' != configured ros2.frame_id '{}'; no TF transform is applied, check planner.yaml",
        topic, msg_frame, expect
    );
}

// /lio/odom 为 world->IMU 的后验状态；建图用的 base 位姿需去掉 IMU 相对 base 的外参：
// T_w_base = T_w_imu * inverse(T_base_imu)，参数 [x,y,z,roll,pitch,yaw] 单位 m/deg。
static auto build_T_base_imu(const Eigen::Matrix<double, 6, 1>& p) -> Eigen::Isometry3d {
    static constexpr double kDegToRad = M_PI / 180.0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = (Eigen::AngleAxisd(p(5) * kDegToRad, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(p(4) * kDegToRad, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(p(3) * kDegToRad, Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
    T.translation() = p.head<3>();
    return T;
}

void GlobalPlanner2d::odom_callback(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    warn_frame_mismatch_once(odom_frame_warned_, config.odom_topic_name, msg->header.frame_id, frame_id);
    const auto& quat = msg->pose.pose.orientation;
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    // 在局部构造完整快照后一次性替换，避免控制/规划线程看到半更新的位姿。
    utils::RobotState pose_state {};
    pose_state.p.x() = msg->pose.pose.position.x;
    pose_state.p.y() = msg->pose.pose.position.y;
    pose_state.p.z() = msg->pose.pose.position.z;
    const auto& twist = msg->twist.twist;
    pose_state.v.x() = twist.linear.x;
    pose_state.v.y() = twist.linear.y;
    pose_state.v.z() = twist.linear.z;
    pose_state.yaw = yaw;
    pose_state.wz = twist.angular.z;
    const Eigen::Vector3d xytheta(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    set_current_state(pose_state, xytheta);

    // 建图位姿走独立话题时，控制状态仍由本回调更新，地图改由 map_odom_callback 更新。
    if (config.map_odom_topic_name.empty()) {
        rog_map::Pose pose;
        pose.first = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        pose.second = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );

        const double stamp_sec = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
            ? rclcpp::Time(msg->header.stamp).seconds()
            : std::numeric_limits<double>::quiet_NaN();
        ma_map_->update_odom(pose, stamp_sec);
    }
};

void GlobalPlanner2d::map_odom_callback(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    warn_frame_mismatch_once(map_odom_frame_warned_, config.map_odom_topic_name, msg->header.frame_id, frame_id);
    const auto& p = msg->pose.pose.position;
    const auto& quat = msg->pose.pose.orientation;
    Eigen::Isometry3d T_w_imu = Eigen::Isometry3d::Identity();
    T_w_imu.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    T_w_imu.linear() = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).normalized().toRotationMatrix();
    const Eigen::Isometry3d T_w_base = T_w_imu * build_T_base_imu(config.map_odom_imu_in_base).inverse();

    rog_map::Pose pose;
    pose.first = T_w_base.translation();
    pose.second = Eigen::Quaterniond(T_w_base.linear());

    const double stamp_sec = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
        ? rclcpp::Time(msg->header.stamp).seconds()
        : std::numeric_limits<double>::quiet_NaN();
    ma_map_->update_odom(pose, stamp_sec);
};
void GlobalPlanner2d::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    warn_frame_mismatch_once(cloud_frame_warned_, config.map_topic_name, msg->header.frame_id, frame_id);
    rog_map::PointCloud pc;
    pcl::fromROSMsg(*msg, pc);
    const double stamp_sec = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
        ? rclcpp::Time(msg->header.stamp).seconds()
        : std::numeric_limits<double>::quiet_NaN();
    ma_map_->update_cloud(pc, stamp_sec);
    ma_map_->update_map();
    const auto grid_map = ma_map_->get_grid_map();
    if (grid_map) {
        visualizer.visualize_occupied_map(*grid_map, frame_id);
    }
    if (config.mapping_model || static_map_loaded_) {
        mapInitialized = true;
    } else if (!static_map_missing_warned_) {
        // 导航模式下静态先验图加载失败：不得把地图标记为已初始化，否则目标点会被静默接受。
        static_map_missing_warned_ = true;
        logger::warn(logger::ros2, "Static global map not loaded, keep mapInitialized=false (check global_map_path)");
    }
}
void GlobalPlanner2d::pub_callback() {
    // Use viz_time_rate from map config as publish frequency (Hz)
    if (ma_map_) {
        double viz_rate = ma_map_->get_config().viz_time_rate;
        if (viz_rate > 0) {
            const double now_sec = clock_.nowSec();
            if (last_pub_sec_ >= 0.0 && now_sec >= last_pub_sec_) {
                const double elapsed = now_sec - last_pub_sec_;
                if (elapsed < 1.0 / viz_rate) {
                    return;
                }
            }
            last_pub_sec_ = now_sec;
        }
    }
    if (!ma_map_) return;

    // 纯可视化回调：不包含 cmd_vel/折叠/任务状态等控制关键逻辑。
    // 先同步 stvl_prior 3D 调试点云按需采集开关（退订时立即关闭采集并释放缓存）；
    // 无订阅者时不取快照、不构建 marker；有订阅时才按 revision 缓存决定是否重绘。
    visualizer.sync_stvl_viz_capture(*ma_map_);
    if (visualizer.map_viz_active()) {
        visualizer.map_viz_callback(*ma_map_, frame_id);
    }
    if (mapInitialized && visualizer.needs_grid_map_snapshot()) {
        // 取当前快照（与地图线程不共享同一个 shared_ptr 变量，避免并发读写指针本身）
        const auto viz_map = ma_map_->get_grid_map();
        if (viz_map) {
            visualizer.visualize_occupied_grid(*viz_map, frame_id);
            visualizer.visualize_esdf_grid(*viz_map, frame_id);
        }
    }
}

void GlobalPlanner2d::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr& msg) {
    // logger::info(
    //     logger::ros2,
    //     "Received target pose with position ({:2f},{:2f},{:2f})",
    //     msg->pose.position.x,
    //     msg->pose.position.y,
    //     msg->pose.position.z
    // );
    warn_frame_mismatch_once(target_frame_warned_, config.target_topic_name, msg->header.frame_id, frame_id);
    if (!mapInitialized) {
        logger::warn(logger::ros2, "map no init");
        return;
    }
    const Eigen::Vector2d goal_xy(msg->pose.position.x, msg->pose.position.y);
    if (!goal_xy.allFinite()) {
        logger::warn(logger::ros2, "Reject non-finite goal pose");
        return;
    }
    const Eigen::Vector3d goal(goal_xy.x(), goal_xy.y(), 1.0);
    if (old_goal_pose_ == goal) {
        //logger::warn(logger::ros2,"old_goal_pose_==goal");
        return;
    }
    if (config.mapping_model) {
        // 建图模式：允许对图外目标按块扩图（受 max_map_size 约束）；超出上限则拒收。
        if (!ma_map_->ensure_covers(goal_xy)) {
            logger::warn(
                logger::ros2,
                "Reject goal ({:.2f}, {:.2f}): outside mapping max_map_size",
                goal_xy.x(),
                goal_xy.y()
            );
            return;
        }
    } else {
        // 导航模式：严格在已加载静态图内，图外目标直接拒收且保留当前在途任务。
        const auto snap = ma_map_->get_grid_map();
        if (!snap || !snap->isInsideMap(goal_xy)) {
            logger::warn(
                logger::ros2,
                "Reject goal ({:.2f}, {:.2f}): outside loaded navigation map bounds",
                goal_xy.x(),
                goal_xy.y()
            );
            return;
        }
    }
    utils::RobotState temp_goal_pose;
    temp_goal_pose.p = goal;
    temp_goal_pose.yaw = atan2(msg->pose.orientation.z, msg->pose.orientation.w) * 2.0;
    // goal/nav_state/plan_start_time_sec_ 会被 planner 组并发读取，短临界区内整体更新。
    {
        std::lock_guard<std::mutex> lock(nav_task_mtx_);
        goal_pose = temp_goal_pose;
        nav_state = replan::FsmReplan::PathState::RUNNING;
        //logger::info(logger::ros2, "set goal success");
        plan_start_time_sec_ = clock_.nowSec();
    }
    old_goal_pose_ = goal;
}

void GlobalPlanner2d::publish_clicked_regions() {
    visualization_msgs::msg::MarkerArray regions;
    regions.markers = clicked_regions_;
    clicked_region_pub_->publish(regions);
}
void GlobalPlanner2d::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr& msg) {
    clicked_points_.push_back(msg->point);
    logger::info(
        logger::ros2,
        "Received rviz2 clicked point ({:.2f},{:.2f},{:.2f}), accumulated {}/4",
        msg->point.x,
        msg->point.y,
        msg->point.z,
        clicked_points_.size()
    );

    if (clicked_points_.size() < 4) {
        return;
    }

    // 当前 4 个点作为一个区域
    std::vector<geometry_msgs::msg::Point> polygon = clicked_points_;
    std::string region_name = "region_" + std::to_string(clicked_region_polygons_.size());

    // 在地图语义层标记为隧道
    {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);

        std::vector<Eigen::Vector2d> eigen_polygon;
        for (const auto& p: polygon) {
            eigen_polygon.emplace_back(p.x, p.y);
        }

        // 写到地图后台缓冲并发布新快照（快照只读）
        ma_map_->set_semantics_polygon(eigen_polygon, grid_map::GridMap::Semantics::TUNNEL);
    }

    // 生成可视化 marker
    visualization_msgs::msg::Marker region;
    region.header.stamp = nh->now();
    region.header.frame_id = frame_id;
    region.ns = "clicked_region";
    region.id = static_cast<int>(clicked_regions_.size());
    region.type = visualization_msgs::msg::Marker::LINE_STRIP;
    region.action = visualization_msgs::msg::Marker::ADD;
    region.pose.orientation.w = 1.0;
    region.scale.x = 0.05;
    region.color.r = 0.0;
    region.color.g = 1.0;
    region.color.b = 0.0;
    region.color.a = 1.0;

    for (const auto& p: polygon) {
        region.points.push_back(p);
    }
    region.points.push_back(polygon.front());

    clicked_region_names_.push_back(region_name);
    clicked_region_polygons_.push_back(polygon);
    clicked_regions_.push_back(region);
    clicked_points_.clear();

    publish_clicked_regions();

    // 每次画完一个区域后自动保存
    save_clicked_regions(config.tunnel_regions_path);

    logger::info(logger::ros2, "Published clicked region #{} name={}", region.id, region_name);
}

void GlobalPlanner2d::load_global_map(const std::string& map_path) {
    auto loaded = ma_map_->load_static_map(map_path);
    if (!loaded) {
        static_map_loaded_ = false;
        logger::error(logger::ros2, "Failed to load global map {}: {}", map_path, loaded.error());
        return;
    }
    static_map_loaded_ = true;
    mapInitialized = true;
    logger::info(
        logger::ros2,
        "Loaded static global map {} ({}x{}, res={:.3f}, origin=({:.2f},{:.2f}), occ={}, free={}, unk={})",
        map_path,
        loaded->geometry.voxel_num.y(),
        loaded->geometry.voxel_num.x(),
        loaded->geometry.resolution,
        loaded->geometry.origin.x(),
        loaded->geometry.origin.y(),
        loaded->occupied_count,
        loaded->free_count,
        loaded->unknown_count
    );
}

void GlobalPlanner2d::save_global_map(const std::string& map_dir) {
    auto saved = ma_map_->save_explored_map(map_dir);
    if (!saved) {
        logger::error(logger::ros2, "Failed to save global map {}: {}", map_dir, saved.error());
        return;
    }
    logger::info(
        logger::ros2,
        "Saved global map {}x{} (res={:.3f}, origin=({:.2f},{:.2f}), occ={}, free={}, unk={}) -> {}",
        saved->saved_geometry.voxel_num.y(),
        saved->saved_geometry.voxel_num.x(),
        saved->saved_geometry.resolution,
        saved->saved_geometry.origin.x(),
        saved->saved_geometry.origin.y(),
        saved->occupied_count,
        saved->free_count,
        saved->unknown_count,
        saved->pgm_path
    );
}

void GlobalPlanner2d::save_clicked_regions(const std::string& yaml_path) {
    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "regions" << YAML::Value << YAML::BeginSeq;

    for (size_t i = 0; i < clicked_region_polygons_.size(); ++i) {
        out << YAML::BeginMap;
        out << YAML::Key << "name" << YAML::Value << clicked_region_names_[i];
        out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;

        for (const auto& p: clicked_region_polygons_[i]) {
            out << YAML::Flow << YAML::BeginSeq;
            out << p.x << p.y << p.z;
            out << YAML::EndSeq;
        }

        out << YAML::EndSeq;
        out << YAML::EndMap;
    }

    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream f(yaml_path);
    if (!f.is_open()) {
        logger::error(logger::ros2, "Failed to save tunnel regions: {}", yaml_path);
        return;
    }

    f << out.c_str();
    f.close();

    logger::info(logger::ros2, "Saved {} tunnel regions to {}", clicked_region_polygons_.size(), yaml_path);
}
void GlobalPlanner2d::load_clicked_regions(const std::string& yaml_path) {
    YAML::Node root;

    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        logger::warn(logger::ros2, "Failed to load tunnel regions {}: {}", yaml_path, e.what());
        return;
    }

    if (!root["regions"]) {
        logger::warn(logger::ros2, "No regions in {}", yaml_path);
        return;
    }

    {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);

        clicked_regions_.clear();
        clicked_region_names_.clear();
        clicked_region_polygons_.clear();

        int index = 0;
        for (const auto& region_node: root["regions"]) {
            std::string name =
                region_node["name"] ? region_node["name"].as<std::string>() : "region_" + std::to_string(index);

            std::vector<geometry_msgs::msg::Point> polygon;

            if (region_node["points"]) {
                for (const auto& pt_node: region_node["points"]) {
                    if (!pt_node.IsSequence() || pt_node.size() < 2) {
                        continue;
                    }

                    geometry_msgs::msg::Point p;
                    p.x = pt_node[0].as<double>();
                    p.y = pt_node[1].as<double>();
                    p.z = pt_node.size() > 2 ? pt_node[2].as<double>() : 0.0;
                    polygon.push_back(p);
                }
            }

            if (polygon.size() < 3) {
                logger::warn(logger::ros2, "Skip invalid region {} with {} points", name, polygon.size());
                continue;
            }

            std::vector<Eigen::Vector2d> eigen_polygon;
            for (const auto& p: polygon) {
                eigen_polygon.emplace_back(p.x, p.y);
            }

            ma_map_->set_semantics_polygon(eigen_polygon, grid_map::GridMap::Semantics::TUNNEL);

            visualization_msgs::msg::Marker region;
            region.header.stamp = nh->now();
            region.header.frame_id = frame_id;
            region.ns = "clicked_region";
            region.id = index;
            region.type = visualization_msgs::msg::Marker::LINE_STRIP;
            region.action = visualization_msgs::msg::Marker::ADD;
            region.pose.orientation.w = 1.0;
            region.scale.x = 0.05;
            region.color.r = 0.0;
            region.color.g = 1.0;
            region.color.b = 0.0;
            region.color.a = 1.0;

            for (const auto& p: polygon) {
                region.points.push_back(p);
            }
            region.points.push_back(polygon.front());

            clicked_region_names_.push_back(name);
            clicked_region_polygons_.push_back(polygon);
            clicked_regions_.push_back(region);

            ++index;
        }
    }

    publish_clicked_regions();

    logger::info(logger::ros2, "Loaded {} tunnel regions from {}", clicked_region_polygons_.size(), yaml_path);
}
} // namespace planner
