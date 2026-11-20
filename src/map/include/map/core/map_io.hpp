#pragma once

#include "map/core/grid_geometry.hpp"
#include "utils/expected.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ma_map {

// 三态栅格单元语义（建图持久层与持久化 I/O 共享）：
// - UNKNOWN  : 未观测区域（建图在线规划可通行；保存为 PGM 205，导航加载按 unknown_as_occupied 处理）
// - FREE     : 已观测可通行区域（保存为 PGM 254）
// - OCCUPIED : 已观测障碍区域（保存为 PGM 0）
enum class CellState : uint8_t {
    UNKNOWN = 0,
    FREE = 1,
    OCCUPIED = 2,
};

struct LoadedMap {
    grid_map::GridGeometry geometry;
    std::vector<CellState> cells; // 行主序 x * ny + y
    std::vector<uint8_t> occupancy; // 0/1，按 LoadOptions::unknown_as_occupied 推导
    int occupied_count = 0;
    int free_count = 0;
    int unknown_count = 0;
};

struct LoadOptions {
    bool unknown_as_occupied = true;
};

struct SaveOptions {
    bool crop_to_explored = true; // 仅保存已探索区域，不写入未探索空白外框
    int margin_cells = 2; // 已探索包围盒外扩的 UNKNOWN 边距格数（夹紧到原图内）
    bool cleanup_free_pinholes = true; // 填补完全被 FREE 包围的内部孤立小 UNKNOWN 洞
    int max_pinhole_cells = 4; // 孤立 UNKNOWN 洞最大清理面积（4 连通格数）
};

struct SaveReport {
    std::string pgm_path;
    std::string yaml_path;
    grid_map::GridGeometry saved_geometry;
    int occupied_count = 0;
    int free_count = 0;
    int unknown_count = 0;
    int cleaned_pinholes = 0;
};

// 纯 C++ 静态地图持久化（P5 PGM + ROS map_server YAML）。
// 轴约定与既有 global_map 兼容：
//   pgm_cols = voxel_num.y()（世界 Y 递增），pgm_rows = voxel_num.x()（顶行对应世界 X 最大，即 pgm_row = nx - 1 - ix）。
class MapIO {
public:
    // 从 .pgm 或 .yaml 文件加载地图，严格继承文件内的 resolution、origin 与尺寸。
    // 失败返回错误描述字符串，无副作用。
    static auto
    load(std::string_view map_path, const LoadOptions& options = {}) -> tl::expected<LoadedMap, std::string>;

    // 保存三态栅格为 P5 PGM（0=OCCUPIED, 254=FREE, 205=UNKNOWN）与同名 YAML。
    // output_path 可为目录（写入 global_map.pgm / global_map.yaml）或 .pgm / .yaml 文件路径。
    // 若 crop_to_explored 为真且没有任何已探索格，返回错误并拒绝覆盖目标文件。
    static auto save(
        std::string_view output_path,
        const grid_map::GridGeometry& geometry,
        std::span<const CellState> cells,
        const SaveOptions& options = {}
    ) -> tl::expected<SaveReport, std::string>;
};

} // namespace ma_map
