cmake_minimum_required(VERSION 3.16)

project(ros2)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")

ament_auto_add_library(${PROJECT_NAME} 
${sources}
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${rosgraph_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
)

target_link_libraries(ros2 PUBLIC
    rclcpp::rclcpp
    ${std_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${rosgraph_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
    tf2::tf2
    tf2_ros::tf2_ros
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    spdlog::spdlog
    utils
    yaml-cpp
    ${PCL_LIBRARIES}
    3d_occ_map
    path_planning
    controller
    traj_optimize
    replan
)