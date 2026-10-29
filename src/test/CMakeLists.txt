cmake_minimum_required(VERSION 3.16)

project(test)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

include(FetchContent)

FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG        v1.8.3 # 建议指定稳定版本
)

# 关键：关闭 benchmark 自身的测试，可大幅减少构建时间
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE) 
FetchContent_MakeAvailable(benchmark)

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