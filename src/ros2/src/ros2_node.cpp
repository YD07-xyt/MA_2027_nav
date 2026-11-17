#include "ros2/ros2_node.h"
#include <fstream>
#include "map/grid_map.hpp"
#include "controller/mpc.h"
#include "replan/fsm_replanner.h"
#include "utils/logger.hpp"
#include "utils/plotter.hpp"
#include "utils/type_utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
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
    mpc_(config.mpc_params),
    nav_state(replan::FsmReplan::PathState::IDLE) {
    fsm_replanner.set_param(config.planner_config);

    map_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    planner_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 2. 订阅者配置选项
    rclcpp::SubscriptionOptions opts_map;
    opts_map.callback_group = map_cb_group_;
    rclcpp::SubscriptionOptions opts_odom;
    opts_odom.callback_group = map_cb_group_; // 里程计也更新地图，放入地图组避免同时写
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

    target_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
        config.target_topic_name,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { target_callback(msg); },
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
        std::chrono::milliseconds(10),
        [this]() { pub_callback(); },
        control_cb_group_ // 可视化放控制组（轻量）
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
    fold_pub_ = nh->create_publisher<std_msgs::msg::Bool>("/ma_nav/fold", 10);
    nav_feedback_pub_ = nh->create_publisher<std_msgs::msg::Int16>("/ma_nav/nav_feedback", 10);

    // 保存/加载全局地图的 ROS2 服务
    save_map_srv_ = nh->create_service<std_srvs::srv::Trigger>(
        config.save_map_srv_topic,
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
            save_global_map(config.save_global_map_path);
            std::string msg = "Global map saved to " + config.save_global_map_path;
            res->success = true;
            res->message = msg;
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
    }
    if (!config.mapping_model) {
        load_global_map(config.global_map_path);
        load_clicked_regions(config.tunnel_regions_path);
    }
}
void GlobalPlanner2d::controller_callback() {
    if (!current_XYTheta.has_value()) return;
    if (!current_pose.has_value()) return;

    if (!ma_traj_interface_ || !ma_traj_interface_->valid()) {
        return;
    }

    Eigen::Vector2d pos(current_XYTheta->x(), current_XYTheta->y());

    // 急停：刹车距离不够，刹车段时长内直接发零速（不经过 MPC）
    if (emergency_stop_until_.has_value()) {
        if (std::chrono::steady_clock::now() < emergency_stop_until_.value()) {
            geometry_msgs::msg::Twist zero;
            cmd_vel_pub_->publish(zero);
            last_vx_world_cmd_ = 0.0;
            last_vy_world_cmd_ = 0.0;
            has_last_mpc_cmd_ = true;
            return;
        }
        // 急停结束：跳过刹车段，从样条上离机器人最近的点开始跟踪
        emergency_stop_until_.reset();
        t_track_ = ma_traj_interface_->nearest_time_from(pos, ma_traj_interface_->brake_duration());
    }

    // 刹车段内按时间推进游标（反向样条与刹车段位置重合，按位置搜索会卡在刹停点附近）；之后按位置更新
    const auto now_tp = std::chrono::steady_clock::now();
    const double dt_ctrl = last_control_time_.has_value()
        ? std::clamp(std::chrono::duration<double>(now_tp - last_control_time_.value()).count(), 0.0, 0.05)
        : 0.0;
    last_control_time_ = now_tp;
    if (t_track_ < ma_traj_interface_->brake_duration()) {
        t_track_ += dt_ctrl;
    } else {
        t_track_ = ma_traj_interface_->update_track_time(pos, t_track_);
    }

    double t_now = t_track_;

    // yaw 只用于速度坐标变换
    const double yaw = current_XYTheta->z();
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
        const double vx_world_odom = current_pose->v.x();
        const double vy_world_odom = current_pose->v.y();

        // 限幅到 MPC 速度约束内
        constexpr double kMaxVel = 3.0;
        vx_world_state = std::clamp(vx_world_odom, -kMaxVel, kMaxVel);
        vy_world_state = std::clamp(vy_world_odom, -kMaxVel, kMaxVel);
    }

    // 4 状态 MPC: [x, y, vx, vy]
    control::StateVector x0;
    x0 << current_XYTheta->x(), current_XYTheta->y(), vx_world_state, vy_world_state;

    control::InputVector u_cmd;
    std::vector<control::StateVector> predicted_states;
    std::vector<control::InputVector> predicted_inputs;

    if (!mpc_.solve(x0, t_now, u_cmd, predicted_states, predicted_inputs)) {
        // 调试时打印 x0，确认是不是速度导致 infeasible
        logger::warn(logger::ros2, "MPC solve failed, x0=[{:.3f}, {:.3f}, {:.3f}, {:.3f}]", x0(0), x0(1), x0(2), x0(3));
        return;
    }

    // 求解成功，更新上一帧 MPC 指令
    if (predicted_states.size() >= 2) {
        last_vx_world_cmd_ = predicted_states[1](2);
        last_vy_world_cmd_ = predicted_states[1](3);
        has_last_mpc_cmd_ = true;
    }

    // 用预测的下一时刻状态作为速度指令
    if (predicted_states.size() < 2) {
        return;
    }

    while (next_fold_event_idx_ < fold_events_.size() && t_track_ >= fold_events_[next_fold_event_idx_].time) {
        const bool fold = fold_events_[next_fold_event_idx_].fold;

        //publish_fold_cmd(fold);
        current_fold_state_ = fold;

        if (fold) {
            std_msgs::msg::Bool fold_msg;
            fold_msg.data = true;
            fold_pub_->publish(fold_msg);
            logger::info(logger::ros2, "进入隧道，发送折叠指令, t_track={:.3f}", t_track_);
        } else {
            std_msgs::msg::Bool fold_msg;
            fold_msg.data = false;
            fold_pub_->publish(fold_msg);
            logger::info(logger::ros2, "离开隧道，发送抬升指令, t_track={:.3f}", t_track_);
        }

        ++next_fold_event_idx_;
    }

    // 4 状态索引: 0=x, 1=y, 2=vx, 3=vy
    const double vx_world_cmd = predicted_states[1](2);
    const double vy_world_cmd = predicted_states[1](3);

    // world -> body 输出
    geometry_msgs::msg::Twist twist;
    twist.linear.x = c * vx_world_cmd + s * vy_world_cmd;
    twist.linear.y = -s * vx_world_cmd + c * vy_world_cmd;
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
    if (this->goal_pose == std::nullopt) {
        logger::debug(logger::ros2, "no goal");
        return;
    }
    if (this->current_pose == std::nullopt) {
        logger::debug(logger::ros2, "no current_pose");
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!plan_start_time_.has_value()) {
        plan_start_time_ = now;
    }
    const auto deadline = plan_start_time_.value() + std::chrono::seconds(20);
    if (now >= deadline) {
        logger::warn(logger::ros2, "navigation task exceeded 20s -> nav:failed");
        nav_state = replan::FsmReplan::PathState::FAILED;
        nav_feedback_pub_->publish(pub_nav_feedback());
        goal_pose.reset();
        plan_start_time_.reset();
        return;
    }

    const double dist_to_goal = (current_pose->p.head<2>() - goal_pose->p.head<2>()).norm();
    if (dist_to_goal < config.planner_config.replan_params.goal_reached_radius) {
        nav_state = replan::FsmReplan::PathState::SUCCESSED;

        auto msg = pub_nav_feedback(); // 打印 SUCCESSED
        nav_feedback_pub_->publish(msg);

        // 到达后收尾，避免重复触发
        goal_pose.reset();
        plan_start_time_.reset();
        return;
    }

    // 本次规划全程使用同一份地图快照（双缓冲），地图线程的更新不会改写它。
    const auto plan_map = ma_map_->get_grid_map();
    auto result = fsm_replanner.plan(goal_pose.value(), current_pose.value(), plan_map);
    const auto finished_at = std::chrono::steady_clock::now();
    if (finished_at >= deadline) {
        logger::warn(logger::ros2, "planning result arrived after 20s -> nav:failed");
        nav_state = replan::FsmReplan::PathState::FAILED;
        nav_feedback_pub_->publish(pub_nav_feedback());
        goal_pose.reset();
        plan_start_time_.reset();
        return;
    }
    if (!result) {
        nav_state = replan::FsmReplan::PathState::RUNNING;
        return;
    }
    auto path_result = result.value();

    // 规划失败保留当前任务，下一帧继续重试；20 秒期限由上方统一处理。
    if (path_result.path_state != replan::FsmReplan::PathState::SUCCESSED) {
        logger::debug(logger::ros2, "PathState not SUCCESSED, waiting...");
        nav_state = replan::FsmReplan::PathState::RUNNING;
        return;
    }

    // 交接前再次检查，过期结果不能下发给控制器。
    if (std::chrono::steady_clock::now() >= deadline) {
        logger::warn(logger::ros2, "planning result expired before handoff -> nav:failed");
        nav_state = replan::FsmReplan::PathState::FAILED;
        nav_feedback_pub_->publish(pub_nav_feedback());
        goal_pose.reset();
        plan_start_time_.reset();
        return;
    }

    visualizer.PubGlobalPath(path_result.planning_traj.raw_path, frame_id);
    visualizer.PubOptPath(path_result.ma_spline_traj.trajectory, frame_id);

    std::vector<Eigen::Vector3d> route;
    for (auto point: path_result.planning_traj.optimized_path) {
        route.emplace_back(point.x(), point.y(), 0.0);
    }

    // 新轨迹下发：把 MPC 跟踪游标定位到新轨迹上离机器人最近的点。
    if (path_result.is_new_trajectory && path_result.ma_spline_traj.success
        && path_result.ma_spline_traj.trajectory.isInitialized())
    {
        ma_traj_interface_ = std::make_shared<control::MaSplineTrajectoryInterface>(path_result.ma_spline_traj);

        mpc_.set_trajectory(ma_traj_interface_);

        fold_events_ = generate_fold_events(
            path_result.ma_spline_traj,
            *plan_map,
            config.planner_config.path_planning_params.fold_time,
            config.planner_config.path_planning_params.unfold_time,
            config.planner_config.path_planning_params.fold_margin
        );

        next_fold_event_idx_ = 0;

        // 跳过新轨迹上已经过去的折叠事件。
        bool desired_fold_state = current_fold_state_;

        while (next_fold_event_idx_ < fold_events_.size() && fold_events_[next_fold_event_idx_].time <= t_track_) {
            desired_fold_state = fold_events_[next_fold_event_idx_].fold;
            ++next_fold_event_idx_;
        }

        if (desired_fold_state != current_fold_state_) {
            //publish_fold_cmd(desired_fold_state);
            current_fold_state_ = desired_fold_state;
        }
        if (current_XYTheta.has_value()) {
            Eigen::Vector2d pos(current_XYTheta->x(), current_XYTheta->y());
            t_track_ = ma_traj_interface_->nearest_time(pos);
        } else {
            t_track_ = 0.0;
        }

        // 反向刹车距离不够：刹车段时长内急停发零速。
        if (ma_traj_interface_->brake_emergency()) {
            const auto brake_ms = static_cast<int64_t>(ma_traj_interface_->brake_duration() * 1000.0);
            emergency_stop_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(brake_ms);
            logger::warn(logger::ros2, "brake distance not enough, emergency stop {} ms", brake_ms);
        } else {
            emergency_stop_until_.reset();
        }

        // 可视化。
        std::vector<Eigen::Vector3d> fold_pts, unfold_pts;

        if (path_result.ma_spline_traj.success && path_result.ma_spline_traj.trajectory.isInitialized()) {
            for (const auto& event: fold_events_) {
                const double local_t = std::clamp(
                    event.time - path_result.ma_spline_traj.brake.duration,
                    0.0,
                    path_result.ma_spline_traj.trajectory.getDuration()
                );

                const double t = path_result.ma_spline_traj.trajectory.getStartTime() + local_t;

                const Eigen::Vector3d p = path_result.ma_spline_traj.trajectory.getTrajectory().evaluate(t, 0);

                if (event.fold) {
                    fold_pts.push_back(p);
                } else {
                    unfold_pts.push_back(p);
                }
            }
        }

        visualizer.visualizeTunnelAndFold(path_result.ma_spline_traj, route, *plan_map, fold_pts, unfold_pts, frame_id);
    }
    auto msg = pub_nav_feedback();

    nav_feedback_pub_->publish(msg);
}
auto GlobalPlanner2d::pub_nav_feedback() -> std_msgs::msg::Int16 {
    std_msgs::msg::Int16 msg;
    switch (nav_state) {
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

void GlobalPlanner2d::odom_callback(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    warn_frame_mismatch_once(odom_frame_warned_, config.odom_topic_name, msg->header.frame_id, frame_id);
    if (!current_pose.has_value()) {
        current_pose = utils::RobotState();
    }
    if (!current_XYTheta.has_value()) {
        current_XYTheta = Eigen::Vector3d::Zero();
    }
    const auto& quat = msg->pose.pose.orientation;
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    current_pose->p.x() = msg->pose.pose.position.x;
    current_pose->p.y() = msg->pose.pose.position.y;
    current_pose->p.z() = msg->pose.pose.position.z;
    const auto& twist = msg->twist.twist;
    current_pose->v.x() = twist.linear.x;
    current_pose->v.y() = twist.linear.y;
    current_pose->v.z() = twist.linear.z;
    current_pose->yaw = yaw;
    current_pose->wz = twist.angular.z;
    current_XYTheta->x() = msg->pose.pose.position.x;
    current_XYTheta->y() = msg->pose.pose.position.y;
    current_XYTheta->z() = yaw;
    rog_map::Pose pose;
    pose.first = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    pose.second = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );

    ma_map_->update_odom(pose);
};
void GlobalPlanner2d::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    warn_frame_mismatch_once(cloud_frame_warned_, config.map_topic_name, msg->header.frame_id, frame_id);
    rog_map::PointCloud pc;
    pcl::fromROSMsg(*msg, pc);
    ma_map_->update_cloud(pc);
    ma_map_->update_map();
    grid_map_ = ma_map_->get_grid_map();
    visualizer.visualize_occupied_map(*grid_map_,frame_id);
    mapInitialized = true;
}
void GlobalPlanner2d::pub_callback() {
    static auto last_pub = std::chrono::steady_clock::now();
    // Use viz_time_rate from map config as publish frequency (Hz)
    if (ma_map_) {
        double viz_rate = ma_map_->get_config().viz_time_rate;
        if (viz_rate > 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_pub).count();
            if (elapsed < 1.0 / viz_rate) {
                return;
            }
            last_pub = now;
        }
    }
    visualizer.map_viz_callback(*ma_map_,frame_id);
    // 取当前快照（与地图线程不共享同一个 shared_ptr 变量，避免并发读写指针本身）
    if (mapInitialized) {
        const auto viz_map = ma_map_->get_grid_map();
        visualizer.visualize_occupied_grid(*viz_map,frame_id);
        visualizer.visualize_esdf_grid(*viz_map,frame_id);
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
    if (mapInitialized) {
        const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 1.0);
        if (old_goal_pose_ == goal) {
            //logger::warn(logger::ros2,"old_goal_pose_==goal");
            return;
        }
        utils::RobotState temp_goal_pose;

        temp_goal_pose.p = goal;
        temp_goal_pose.yaw = atan2(msg->pose.orientation.z, msg->pose.orientation.w) * 2.0;
        goal_pose = temp_goal_pose;
        old_goal_pose_ = goal;
        nav_state = replan::FsmReplan::PathState::RUNNING;
        //logger::info(logger::ros2, "set goal success");
        plan_start_time_ = std::chrono::steady_clock::now();
    } else {
        logger::warn(logger::ros2, "map no init");
    }
    return;
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

void GlobalPlanner2d::load_global_map(const std::string& pgm_path) {
    std::ifstream f(pgm_path, std::ios::binary);
    if (!f.is_open()) {
        logger::error(logger::ros2, "Failed to open global map: {}", pgm_path);
        return;
    }
    std::string magic;
    if (!(f >> magic) || magic != "P5") {
        logger::error(logger::ros2, "Only P5 binary PGM supported: {}", pgm_path);
        return;
    }

    // PGM 头里允许出现 '#' 注释和任意空白，逐个跳过
    auto skip_pgm_header_space = [&f]() {
        while (true) {
            int c = f.peek();
            if (c == '#') {
                f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                f.get();
            } else {
                break;
            }
        }
    };

    int w = 0, h = 0, maxval = 0;
    skip_pgm_header_space();
    if (!(f >> w)) {
        logger::error(logger::ros2, "Invalid PGM width: {}", pgm_path);
        return;
    }

    skip_pgm_header_space();
    if (!(f >> h)) {
        logger::error(logger::ros2, "Invalid PGM height: {}", pgm_path);
        return;
    }

    skip_pgm_header_space();
    if (!(f >> maxval)) {
        logger::error(logger::ros2, "Invalid PGM maxval: {}", pgm_path);
        return;
    }

    if (maxval != 255 || w <= 0 || h <= 0) {
        logger::error(
            logger::ros2,
            "Only P5 binary PGM (maxval=255) supported: {} ({}x{}, maxval={})",
            pgm_path,
            w,
            h,
            maxval
        );
        return;
    }

    // 关键：PGM 规范里 maxval 后面只消耗一个空白字符。
    // 不能跳过所有空白，因为二进制像素数据本身可能以空格/换行等字节开头。
    int sep = f.get();
    if (sep == '\r' && f.peek() == '\n') {
        f.get(); // 兼容 CRLF
    }

    // 读入全部像素，之后按世界坐标做任意像素查询
    std::vector<unsigned char> pixels(static_cast<size_t>(w) * h);
    f.read(reinterpret_cast<char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (f.gcount() != static_cast<std::streamsize>(pixels.size())) {
        logger::error(logger::ros2, "PGM pixel data truncated: {}", pgm_path);
        return;
    }
    f.close();

    // 读取同目录 global_map.yaml 中的 resolution/origin（save_global_map 生成的）。
    // 缺少该信息无法把 PGM 像素映射回世界坐标，直接拒绝加载，避免错位/越界。
    std::string yaml_path = pgm_path;
    if (yaml_path.size() > 4 && yaml_path.substr(yaml_path.size() - 4) == ".pgm") {
        yaml_path = yaml_path.substr(0, yaml_path.size() - 4) + ".yaml";
    }
    double pgm_res = 0.0;
    Eigen::Vector2d pgm_origin = Eigen::Vector2d::Zero();
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root["resolution"] || !root["origin"]) {
            logger::error(
                logger::ros2,
                "Global map yaml {} missing resolution/origin, load aborted (size mismatch would misalign map)",
                yaml_path
            );
            return;
        }
        pgm_res = root["resolution"].as<double>();
        auto origin = root["origin"].as<std::vector<double>>();
        if (origin.size() < 2 || pgm_res <= 0.0) {
            logger::error(logger::ros2, "Global map yaml {} has invalid resolution/origin, load aborted", yaml_path);
            return;
        }
        pgm_origin << origin[0], origin[1];
    } catch (const std::exception& e) {
        logger::error(
            logger::ros2,
            "Failed to parse global map yaml {} ({}), load aborted (size mismatch would misalign map)",
            yaml_path,
            e.what()
        );

        return;
    }

    // 按世界坐标重投影到当前栅格图：遍历当前每个 cell，把其中心的世界坐标
    // 换算成 PGM 像素坐标，仅当像素落在 PGM 范围内时取值（否则保持 0/自由）。
    // 这样无论当前地图尺寸/分辨率与保存时是否一致，障碍物都落在正确位置。
    auto grid = ma_map_->get_grid_map();
    const Eigen::Vector2i voxel_num = grid->getVoxelNum();
    const Eigen::Vector2d grid_origin = grid->getOrigin();
    const double grid_res = grid->getResolution();
    const int pgm_rows = h, pgm_cols = w;

    grid_map::RowMatrixXi global = grid_map::RowMatrixXi::Zero(voxel_num.x(), voxel_num.y());
    int loaded = 0;
    for (int i = 0; i < voxel_num.x(); ++i) {
        for (int j = 0; j < voxel_num.y(); ++j) {
            // 当前 cell 中心的世界坐标（与 GridMap::indexToPos 一致）
            const double wx = (i + 0.5) * grid_res + grid_origin.x();
            const double wy = (j + 0.5) * grid_res + grid_origin.y();
            // PGM 像素坐标约定（与 save_global_map 一致）：
            //   占用矩阵 map_occ(row, col) 的 row=世界 x、col=世界 y；
            //   PGM 列方向 = 世界 Y（左→右 y 增大），PGM 行方向 = 世界 X（顶→底 x 减小）
            const int px = static_cast<int>(std::floor((wy - pgm_origin.y()) / pgm_res)); // PGM 列 = 世界 y
            const int py_grid =
                static_cast<int>(std::floor((wx - pgm_origin.x()) / pgm_res)); // 世界 x 格号（0 = x 最小）
            const int pgm_row = pgm_rows - 1 - py_grid; // 顶行 = x 最大
            if (px < 0 || px >= pgm_cols || pgm_row < 0 || pgm_row >= pgm_rows) {
                continue; // 当前 cell 在老地图范围之外，保持自由
            }
            if (pixels[static_cast<size_t>(pgm_row) * pgm_cols + px] < 128) {
                global(i, j) = 1;
                ++loaded;
            }
        }
    }
    ma_map_->set_global_map(global);
    logger::info(
        logger::ros2,
        "Loaded global map {}x{} (res={:.3f}, origin=({:.2f},{:.2f})) -> current grid {}x{} (res={:.3f}), {} occupied cells projected",
        pgm_cols,
        pgm_rows,
        pgm_res,
        pgm_origin.x(),
        pgm_origin.y(),
        voxel_num.x(),
        voxel_num.y(),
        grid_res,
        loaded
    );
}

void GlobalPlanner2d::save_global_map(const std::string& map_dir) {
    auto map_occ = ma_map_->get_occupancy_2d(); // 保存全球永久层
    int h = map_occ.rows(), w = map_occ.cols();
    double res = ma_map_->get_grid_map()->getResolution();
    Eigen::Vector2d origin = ma_map_->get_grid_map()->getOrigin();

    std::ofstream f(map_dir + "/global_map.pgm", std::ios::binary);
    f << "P5\n" << w << " " << h << "\n255\n";
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            // GridMap 行 0=底部=世界 y_min，PGM 行 0=顶部=世界 y_max，翻转 Y
            f.put(map_occ(h - 1 - y, x) ? (char)0 : (char)255);
    f.close();

    std::ofstream y(map_dir + "/global_map.yaml");
    y << "image: global_map.pgm\n";
    y << "resolution: " << res << "\n";
    y << "origin: [" << origin.x() << ", " << origin.y() << ", 0.0]\n";
    y << "negate: 0\n";
    y << "occupied_thresh: 0.65\n";
    y << "free_thresh: 0.196\n";
    y << "mode: trinary\n";
    y.close();

    logger::info(logger::ros2, "Saved global map {}x{} -> {}", w, h, map_dir);
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

std::vector<GlobalPlanner2d::TunnelInterval> GlobalPlanner2d::detect_tunnel_intervals(
    const ma_spline_opt::MAsplineOutput& ma_traj,
    const grid_map::GridMap& map,
    double sample_dt
) {
    std::vector<TunnelInterval> intervals;

    if (!ma_traj.success || !ma_traj.trajectory.isInitialized()) {
        return intervals;
    }

    const double duration = ma_traj.trajectory.getDuration();
    const double start_time = ma_traj.trajectory.getStartTime();
    const auto& spline = ma_traj.trajectory.getTrajectory();

    bool inside = false;
    double entry_time = 0.0;

    for (double local_t = 0.0; local_t <= duration + 1e-6; local_t += sample_dt) {
        const double t = start_time + local_t;
        const Eigen::Vector2d pos = spline.evaluate(t, 0).head<2>();

        const bool in_tunnel = map.is_tunnel(pos);

        if (!inside && in_tunnel) {
            inside = true;
            entry_time = local_t;
        }

        if (inside && !in_tunnel) {
            inside = false;
            intervals.push_back({entry_time, local_t});
        }
    }

    if (inside) {
        intervals.push_back({entry_time, duration});
    }
    // for(auto i:intervals){
    //     logger::info(
    //         logger::ros2,
    //         "tunnel interval: entry={:.3f}, exit={:.3f}, duration={:.3f}",
    //         i.entry_time,
    //         i.exit_time,
    //         i.exit_time - i.entry_time
    //     );
    // }

    return intervals;
}
std::vector<GlobalPlanner2d::FoldEvent> GlobalPlanner2d::generate_fold_events(
    const ma_spline_opt::MAsplineOutput& ma_traj,
    const grid_map::GridMap& map,
    double fold_time,
    double unfold_time,
    double margin,
    double sample_dt
) {
    std::vector<FoldEvent> events;

    // 1. 先检测隧道区间
    auto intervals = detect_tunnel_intervals(ma_traj, map, sample_dt);

    if (intervals.empty()) {
        return events;
    }

    // 2. 合并过近的隧道区间
    //    如果两个隧道之间不足以安全完成“抬升 -> 再重新折叠”，
    //    就视为同一个连续折叠段，避免中间错误抬升。
    const double merge_gap = fold_time + unfold_time + 2.0 * margin;

    std::vector<TunnelInterval> merged_intervals;
    for (const auto& interval: intervals) {
        if (!merged_intervals.empty() && interval.entry_time - merged_intervals.back().exit_time < merge_gap) {
            // 合并到前一个区间，出口取更晚的那个
            merged_intervals.back().exit_time = std::max(merged_intervals.back().exit_time, interval.exit_time);
        } else {
            merged_intervals.push_back(interval);
        }
    }

    // 3. 基于合并后的区间生成原始事件
    std::vector<FoldEvent> raw_events;

    for (const auto& interval: merged_intervals) {
        double t_fold_start = interval.entry_time - fold_time - margin;
        double t_unfold_start = interval.exit_time + margin;

        // 如果还没出发就需要折叠，就强制从 0 开始
        t_fold_start = std::max(0.0, t_fold_start);

        raw_events.push_back({true, t_fold_start});
        raw_events.push_back({false, t_unfold_start});
    }

    // 4. 排序，并按状态压缩，避免无意义的连续折叠/展开
    std::sort(raw_events.begin(), raw_events.end(), [](const FoldEvent& a, const FoldEvent& b) {
        return a.time < b.time;
    });

    bool current_folded = false;

    for (const auto& event: raw_events) {
        if (event.fold != current_folded) {
            events.push_back(event);
            current_folded = event.fold;
        }
    }

    // 事件时间按样条计算；跟踪游标 t_track_ 包含前面的刹车段，统一加上刹车段时长
    for (auto& event: events) {
        event.time += ma_traj.brake.duration;
    }

    return events;
}
} // namespace planner
