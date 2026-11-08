cmake_minimum_required(VERSION 3.16)

project(test)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# 使用系统/环境已安装的 Google Benchmark（需提前安装，见下方说明）
find_package(benchmark CONFIG REQUIRED)

add_executable(test_astar_vs_jps  src/test_astar_vs_jps.cpp)
target_include_directories(test_astar_vs_jps PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_astar_vs_jps  PUBLIC
    spdlog::spdlog
    utils
    Eigen3::Eigen
    3d_occ_map
    yaml-cpp
    ${PCL_LIBRARIES}
    path_planning
    controller
    traj_optimize
    replan
)
target_link_libraries(test_astar_vs_jps 
  PRIVATE 
    ${OpenCV_LIBS}
    benchmark::benchmark # main 由 test_astar_vs_jps.cpp 提供(先输出对比图,再跑 benchmark)
)

# 路径后处理测试：可视化（点红、连线蓝）+ 分阶段耗时 + Google Benchmark，结果写 images/output/post_processing/
add_executable(test_post_processing src/test_post_processing.cpp)
target_include_directories(test_post_processing PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_post_processing PUBLIC
    spdlog::spdlog
    utils
    Eigen3::Eigen
    3d_occ_map
    yaml-cpp
    path_planning
)
target_link_libraries(test_post_processing
  PRIVATE
    ${OpenCV_LIBS}
    benchmark::benchmark # main 由 test_post_processing.cpp 提供
)

# 轨迹优化测试：JPS -> 后处理 -> 两阶段样条优化，可视化（速度着色）+ 动力学/安全距离检查 + 耗时，结果写 images/output/traj_optimize/
add_executable(test_traj_optimize src/test_traj_optimize.cpp)
target_include_directories(test_traj_optimize PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_traj_optimize PUBLIC
    spdlog::spdlog
    utils
    Eigen3::Eigen
    3d_occ_map
    yaml-cpp
    path_planning
    traj_optimize
)
target_link_libraries(test_traj_optimize
  PRIVATE
    ${OpenCV_LIBS}
    benchmark::benchmark # main 由 test_traj_optimize.cpp 提供
)

# STL 场地 -> ROG 3D 占据 -> 2D 占据（按列）-> 2D ESDF，结果写 images/output/stl_map/
add_executable(test_stl_to_map src/test_stl_to_map.cpp)
target_include_directories(test_stl_to_map PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_stl_to_map PUBLIC
    spdlog::spdlog
    utils
    Eigen3::Eigen
    3d_occ_map
    yaml-cpp
    ${PCL_LIBRARIES}
)
target_link_libraries(test_stl_to_map PRIVATE ${OpenCV_LIBS})

# 地形分析离线回放（帧文件由 scripts/terrain_replay_extract.py 从 rosbag 导出）
add_executable(test_terrain_replay src/test_terrain_replay.cpp)
target_link_libraries(test_terrain_replay PUBLIC
    spdlog::spdlog
    utils
    Eigen3::Eigen
    3d_occ_map
    yaml-cpp
    ${PCL_LIBRARIES}
)