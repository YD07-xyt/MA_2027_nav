#pragma once
#include "config.hpp"

//planner

#include "map/ma_map.hpp"
//map

//controller
#include "controller/mpc.h"
//nav
#include "controller/traj_interface.hpp"
#include "replan/fsm_replanner.h"
//#include "fsm/fsm.hpp"

//3rd
#include <Eigen/Core>
//log
#include <spdlog/spdlog.h>
//ros2

#include "utils/clock.hpp"
#include "utils/plotter.hpp"
#include "utils/type_utils.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include "ros2/misc/visualizer.hpp"
//std
#include <atomic>
#include <optional>
#include <cstddef>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace planner {

class GlobalPlanner2d {
private:
    Config config;
    rclcpp::Node::SharedPtr nh;
    std::string frame_id; // 来自 planner.yaml ros2.frame_id，构造时赋值
    // 输入消息 frame_id 与配置不一致时只告警一次
    bool cloud_frame_warned_ = false;
    bool odom_frame_warned_ = false;
    bool target_frame_warned_ = false;
private:
    rclcpp::CallbackGroup::SharedPtr map_cb_group_;
    rclcpp::CallbackGroup::SharedPtr planner_cb_group_;
    rclcpp::CallbackGroup::SharedPtr control_cb_group_; // 控制器 + 可视化

    // 保护 clicked_regions_ 与语义区域写入临界区（MaMap 快照读写由内部锁保护）
    mutable std::shared_mutex map_mutex_;

    // 时间源：use_sim_time 时订阅 /clock 并注入 MaMap，否则使用 steady_clock。
    bool use_sim_time_ = false;
    utils::AtomicClockSource sim_clock_source_;
    utils::Clock clock_ = utils::Clock::steady();
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sim_clock_sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clicked_region_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fold_pub_; 
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr nav_feedback_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_tunnel_regions_srv_;

    // rviz2 点选区域：名字 + 4 个点
    std::vector<geometry_msgs::msg::Point> clicked_points_;
    std::vector<std::string> clicked_region_names_;
    std::vector<std::vector<geometry_msgs::msg::Point>> clicked_region_polygons_;
    // 已生成的历史区域（MarkerArray 整体重发，历史区域保持显示）
    std::vector<visualization_msgs::msg::Marker> clicked_regions_;

    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    rclcpp::TimerBase::SharedPtr pub_viz_timer_;
    std::optional<utils::RobotState> current_pose = std::nullopt;
    std::optional<utils::RobotState> goal_pose = std::nullopt;
    std::optional<Eigen::Vector3d> current_XYTheta = std::nullopt;
    std::atomic<bool> mapInitialized;
    // 导航模式下静态先验图是否成功加载；失败时 map_callback 不得把 mapInitialized 置 true。
    bool static_map_loaded_ = false;
    bool static_map_missing_warned_ = false;
    Visualizer visualizer;

private:
    Eigen::Vector3d old_goal_pose_=Eigen::Vector3d::Zero();
    //地图
    std::shared_ptr<ma_map::MaMap> ma_map_;
    //重规划
    replan::FsmReplan fsm_replanner;
    std::optional<double> plan_start_time_sec_;
    replan::FsmReplan::PathState nav_state;
   auto pub_nav_feedback()->std_msgs::msg::Int16;
    //FSM fsm_;
    tools::Plotter plotter_;
    struct FoldEvent {
        bool fold = true;
        double time = 0.0;
    };
    struct TunnelInterval {
        double entry_time = 0.0;
        double exit_time = 0.0;
    };
    bool is_print = true;
    const float kFoldTime = 1.0; //折叠时间
    const float kUnfoldTime = 1.0; //展开时间
    const float kMarginTime = 0.5;
    std::vector<FoldEvent> fold_events_;
    size_t next_fold_event_idx_ = 0;
    // 当前下发的云台状态：true=折叠，false=抬升
    bool current_fold_state_ = false;
    std::vector<TunnelInterval> detect_tunnel_intervals(
        const ma_spline_opt::MAsplineOutput& ma_traj,
        const grid_map::GridMap& map,
        double sample_dt = 0.02
    );
    std::vector<FoldEvent> generate_fold_events(
        const ma_spline_opt::MAsplineOutput& ma_traj,
        const grid_map::GridMap& map,
        double fold_time,
        double unfold_time,
        double margin,
        double sample_dt = 0.002
    );

private:
    //contorller
    double t_now_;
    Trajectory<5, 2> minco_trajectory_;

private:
    control::Mpc<control::MaSplineTrajectoryInterface> mpc_;
    std::shared_ptr<control::MaSplineTrajectoryInterface> ma_traj_interface_;
    double t_track_ = 0.0;
    // 保存上一帧 MPC 的 world 速度指令，作为下一帧 x0 的速度状态
    double last_vx_world_cmd_ = 0.0;
    double last_vy_world_cmd_ = 0.0;
    bool has_last_mpc_cmd_ = false;
    // 上一次控制回调时刻：刹车段内按时间推进跟踪游标（double 秒，跟随注入时钟）
    std::optional<double> last_control_time_sec_;
    // 反向刹车距离不够时的急停截止时刻：之前一直发零速，之后跳过刹车段从样条起点跟踪
    std::optional<double> emergency_stop_until_sec_;
    // 可视化发布节流时刻（double 秒，跟随注入时钟）
    double last_pub_sec_ = -1.0;

public:
    explicit GlobalPlanner2d(rclcpp::Node::SharedPtr nh_, std::string params_path);
    inline auto get_num_threads()->int{
        return config.num_threads;
    }
    void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr& msg);
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr& msg);
    void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr& msg);
    void planner_callback();
    void controller_callback();
    void plan_omni();
    void pub_callback();
    void load_global_map(const std::string& map_path);
    void save_global_map(const std::string& map_dir); // 目录路径，自动生成 .pgm + .yaml
    void save_clicked_regions(const std::string& yaml_path);
    void load_clicked_regions(const std::string& yaml_path);
    void publish_clicked_regions();
};
} // namespace planner