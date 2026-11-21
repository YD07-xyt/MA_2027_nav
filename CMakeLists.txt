cmake_minimum_required(VERSION 3.10)
project(ma_nav)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3  -Wall -g -DNDEBUG -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -flto=auto")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto=auto")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "-flto=auto")

#########################################################
###################     LTO     #########################
#########################################################
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)

if(ipo_supported)

    add_compile_options($<$<CONFIG:Release>:-flto=auto>)
    add_link_options($<$<CONFIG:Release>:-flto=auto>)
     message(STATUS "全局 LTO 已启用")
else()
    message(WARNING "LTO 不被支持: ${ipo_error}")
endif()
#########################################################



#rog_map need
add_definitions( -DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\" )
add_definitions(-DORIGIN_AT_CORNER)

# 查找依赖
find_package(ament_cmake_auto REQUIRED)

# ---- ROS 2 显式 find_package，创建 std_msgs::std_msgs 等target ----

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)
find_package(spdlog REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PCL REQUIRED)

# find_package(nlohmann_json REQUIRED)
# find_package(behaviortree_cpp REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system filesystem thread)

#include_directories(/usr/local/include/osqp)

add_subdirectory(src/utils)
add_subdirectory(src/map)
add_subdirectory(src/planner/path_planning)
add_subdirectory(src/planner/traj_optimize)
add_subdirectory(src/planner/controller)
add_subdirectory(src/planner/replan)
add_subdirectory(src/ros2)
add_subdirectory(src/test)

# 添加可执行文件
ament_auto_add_executable(ma_nav src/ma_nav.cpp)
set_target_properties(ma_nav PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

target_link_libraries(ma_nav PUBLIC
  ${OMPL_LIBRARIES}
  spdlog::spdlog
  osqp::osqp
  OsqpEigen::OsqpEigen
  yaml-cpp
  utils
  3d_occ_map
  path_planning
  controller
  traj_optimize
  replan
  ros2
)
# 导出包
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)