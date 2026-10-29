#include "map/terrain_analysis.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Terrain {

auto TerrainAnalyzer::analyze(const utils::RobotState &robot_pose,
                              const Map &occ_map) -> Map {
  Map free_unused;
  return analyze(robot_pose, occ_map, free_unused);
}

auto TerrainAnalyzer::analyze(const utils::RobotState &robot_pose,
                              const Map &occ_map, Map &free_out) -> Map {
  return analyze(robot_pose, occ_map, free_out, VoxelQuery{});
}

auto TerrainAnalyzer::analyze(const utils::RobotState &robot_pose,
                              const Map &occ_map, Map &free_out,
                              const VoxelQuery &voxel_query) -> Map {
  free_out.clear();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const bool use_evidence = static_cast<bool>(voxel_query);
  // 体素边长：terrain 网格与 ROG 体素同分辨率，occ_map 里的点都是体素中心
  const float res = config_.resolution;

  // 1. 确定分析窗口
  GridInfo grid = setupGrid(robot_pose.p);

  // 2. 将点云分配到网格，记录每个网格的最低点和最高点
  std::vector<std::vector<float>> height_lists(grid.width * grid.height);
  // 每格一个真实体素中心的 xy，用于向 ROG 查询该列体素状态
  std::vector<utils::Vec3f> column_xy(grid.width * grid.height,
                                      utils::Vec3f::Zero());
  for (const auto &pt : occ_map) {
    auto idx_opt = worldToGrid(grid, pt);
    if (!idx_opt) continue;
    auto [gx, gy] = *idx_opt;
    auto &list = height_lists[gy * grid.width + gx];
    if (list.empty()) column_xy[gy * grid.width + gx] = pt;
    list.push_back(pt.z());
  }

  // 从体素中心高度 z_occ 正下方开始，向下连续 KNOWN_FREE 的体素数。
  // 允许先跳过最多 overhang_unknown_skip 个紧邻的 UNKNOWN（杆底混合体素射线清不掉）。
  auto free_run_below = [&](int idx, float z_occ) -> int {
    const utils::Vec3f &xy = column_xy[idx];
    auto state_at = [&](float z) {
      return voxel_query(utils::Vec3f(xy.x(), xy.y(), z));
    };
    auto run_from = [&](float z_top) {
      int run = 0;
      // 上限：最多向下看 2 m，防止退化成长循环
      const int max_steps = static_cast<int>(2.0f / res);
      for (int d = 0; d < max_steps; ++d) {
        if (state_at(z_top - d * res) != utils::KNOWN_FREE) break;
        ++run;
      }
      return run;
    };
    int best = run_from(z_occ - res);
    for (int s = 1; s <= config_.overhang_unknown_skip; ++s) {
      if (state_at(z_occ - s * res) != utils::UNKNOWN) break;
      best = std::max(best, run_from(z_occ - (s + 1) * res));
    }
    return best;
  };
  // 已知空闲段的底（偏保守：按跳过了 overhang_unknown_skip 个未知体素估计）
  auto free_bottom = [&](float z_occ, int run) {
    return z_occ - 0.5f * res - (run + config_.overhang_unknown_skip) * res;
  };
  // 空闲段底是否接到了地面：地面是占据体素中心，容差半格
  auto reaches_ground = [&](float bottom, float ground_z) {
    return bottom <= ground_z + 0.5f * res + 1e-4f;
  };

  // 3. 提取网格内的 最低点（作为地面） 和 最高点（作为墙体）
  Eigen::ArrayXXf ground = Eigen::ArrayXXf::Constant(
      grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());
  Eigen::ArrayXXf max_height = Eigen::ArrayXXf::Constant(
      grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());

  for (int gy = 0; gy < grid.height; ++gy) {
    for (int gx = 0; gx < grid.width; ++gx) {
      const auto &z_vals = height_lists[gy * grid.width + gx];
      if (z_vals.empty()) continue;

      float min_z = std::numeric_limits<float>::max();
      float max_z = -std::numeric_limits<float>::max();
      for (float z : z_vals) {
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
      }
      ground(gx, gy) = min_z;
      max_height(gx, gy) = max_z;
    }
  }

  // 4. 保留一份未平滑的格内最低点：台阶边沿判据要用原始值，平滑会把边沿抹掉
  Eigen::ArrayXXf ground_raw = ground;

  // ================= 悬空列（需要 3D 空闲证据）=================
  // 列的最低占据体素正下方被射线扫空，且空闲段一直通到邻域地面 → 这是悬空结构（杆）的下沿，
  // 不是地面。台面/台阶顶面下方是实心，射线进不去只会是 UNKNOWN，不会被判成悬空；
  // 台沿外侧被掠射射线刻出的一小段空气，底下接不到地面，也过不了"通到地面"这一条。
  // 悬空列不提供地面高度（否则杆下沿会被当地面，杆旁的格高差误判成墙）。
  Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> floating =
      Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>::Constant(
          grid.width, grid.height, false);
  if (use_evidence) {
    Eigen::ArrayXXi run_lo = Eigen::ArrayXXi::Zero(grid.width, grid.height);
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> cand = floating;
    for (int gy = 0; gy < grid.height; ++gy) {
      for (int gx = 0; gx < grid.width; ++gx) {
        if (std::isnan(ground_raw(gx, gy))) continue;
        run_lo(gx, gy) = free_run_below(gy * grid.width + gx, ground_raw(gx, gy));
        cand(gx, gy) = run_lo(gx, gy) >= config_.overhang_free_voxels;
      }
    }
    // 邻域地面：非候选列的最低占据。杆宽 0.25–0.6 m，杆中间列的近邻也全是候选，半径要放大
    const int R = config_.overhang_ground_radius;
    for (int gy = 0; gy < grid.height; ++gy) {
      for (int gx = 0; gx < grid.width; ++gx) {
        if (!cand(gx, gy)) continue;
        float g_nb = nan;
        for (int dy = -R; dy <= R; ++dy) {
          for (int dx = -R; dx <= R; ++dx) {
            const int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height) continue;
            if (cand(nx, ny)) continue;
            const float v = ground_raw(nx, ny);
            if (std::isnan(v)) continue;
            if (std::isnan(g_nb) || v < g_nb) g_nb = v;
          }
        }
        if (std::isnan(g_nb)) continue;  // 邻域没有地面：不下结论
        const float z_lo = ground_raw(gx, gy);
        floating(gx, gy) =
            (z_lo - g_nb >= config_.min_overhang_clearance) &&
            reaches_ground(free_bottom(z_lo, run_lo(gx, gy)), g_nb);
      }
    }
    for (int gy = 0; gy < grid.height; ++gy)
      for (int gx = 0; gx < grid.width; ++gx)
        if (floating(gx, gy)) ground_raw(gx, gy) = nan;
    ground = ground_raw;
  }
  // =========================================================

  // 5. 中值滤波平滑地面高度（沿坡面连续，坡面靠它保持可通行）
  ground = medianFilter(ground, config_.kernel_size);

  // ================= 参考地面 z_ref =================
  // 只用格内最低点当地面会把台阶踏面、悬空物下方当成"地面"：
  //   - 台阶踏面格：格内只有台面点，最低点就是台面 → 自认平坦地面 → 台阶漏检；
  //   - 限高杆靠近高台：邻域地面被坡/台面抬高 → 高差判成墙 → 限高杆被误杀。
  // 这里引入"邻域最低地面"：如果本格地面明显高于邻域最低地面，说明本格地面被抬高了，
  // 改用邻域最低地面作为参考。高台内部远离边沿时邻域全是台面，参考地面仍是台面，
  // 因此只有台阶边沿一圈会被判成障碍。

  // 高出参考地面多少以内算"贴地"（平地、平缓坡）
  const float kObstacleBand = config_.obstacle_band;
  // 本格地面比邻域最低地面高出多少，才认为本格地面被抬高了
  const float kGroundJumpThresh = config_.ground_jump_thresh;
  // 邻域最低地面的搜索半径（格），2 对应 5×5、按 0.05 m 分辨率约 0.25 m
  const int kGroundMinRadius = config_.ground_min_radius;

  Eigen::ArrayXXf z_ref = Eigen::ArrayXXf::Constant(
      grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());

  for (int gy = 0; gy < grid.height; ++gy) {
    for (int gx = 0; gx < grid.width; ++gx) {
      const float g_med = ground(gx, gy);
      if (std::isnan(g_med)) continue;
      if (floating(gx, gy)) continue;  // 悬空列不参与参考地面与梯度

      float g_min = g_med;
      for (int dy = -kGroundMinRadius; dy <= kGroundMinRadius; ++dy) {
        for (int dx = -kGroundMinRadius; dx <= kGroundMinRadius; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const int nx = gx + dx;
          const int ny = gy + dy;
          if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height) continue;
          const float neighbor = ground_raw(nx, ny);
          if (std::isnan(neighbor)) continue;
          if (neighbor < g_min) g_min = neighbor;
        }
      }
      // 邻域存在明显更低的接地格 → 本格地面被抬高，以低的为准；否则沿用平滑后的地面
      z_ref(gx, gy) = (g_med - g_min > kGroundJumpThresh) ? g_min : g_med;
    }
  }
  // =========================================================

  // =================计算参考地面的邻域高度梯度 =================
  // 只在 z_ref 上算梯度：地面本身不连续（台阶/陡坡）才会触发，
  // 限高杆靠近高台时 z_ref 是连续的低处地面，不会被误触发。

  Eigen::ArrayXXf gradient = Eigen::ArrayXXf::Zero(grid.width, grid.height);

  for (int gy = 0; gy < grid.height; ++gy) {
    for (int gx = 0; gx < grid.width; ++gx) {
      if (std::isnan(z_ref(gx, gy))) continue;
      float max_diff = 0.0f;
      // 遍历8邻域
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          int nx = gx + dx;
          int ny = gy + dy;
          if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height)
          continue; float neighbor = z_ref(nx, ny); if
          (std::isnan(neighbor)) continue; float diff = std::abs(z_ref(gx,
          gy) - neighbor); if (diff > max_diff) max_diff = diff;
        }
      }
      gradient(gx, gy) = max_diff;
    }
  }
  // =========================================================

  // 综合判断并输出障碍物
  // 判据：相对参考地面 z_ref
  //   1) 没有高出 z_ref + kObstacleBand 的点           → 贴地（平地、平缓坡），可通行
  //   2) 最低悬空点 - z_ref >= robot_height            → 可从下方通过（限高杆/隧道/桌下），可通行
  //   3) 其余                                          → 障碍（台阶、墙、立柱、陡坡）
  Map output;
  output.reserve(grid.width * grid.height);
  // 本帧判为障碍的格，供后面"空列空闲证据"做邻接保护
  Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> is_obstacle =
      Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>::Constant(
          grid.width, grid.height, false);

  for (int gy = 0; gy < grid.height; ++gy) {
    for (int gx = 0; gx < grid.width; ++gx) {
      if (floating(gx, gy)) {
        // 悬空列：下沿正下方净空已被射线证实一直通到地面，可从下方通过
        free_out.push_back(gridToWorld(grid, gx, gy));
        continue;
      }
      const float ground_ref = z_ref(gx, gy);
      if (std::isnan(ground_ref)) continue;

      const float obstacle_max_z = max_height(gx, gy);
      if (std::isnan(obstacle_max_z)) continue;  // 本格无观测，不下结论

      const auto &z_vals = height_lists[gy * grid.width + gx];

      // 高出参考地面的点里最低的那个，即挡住通行的那一层的下沿
      float z_low = std::numeric_limits<float>::quiet_NaN();
      for (float z : z_vals) {
        if (z <= ground_ref + kObstacleBand) continue;
        if (std::isnan(z_low) || z < z_low) z_low = z;
      }

      if (std::isnan(z_low)) {
        // 全部点都贴着参考地面：地面本身
        // 地面在邻域内有明显跳变（台阶顶面、陡坡）时仍判障碍
        if (gradient(gx, gy) > config_.steep_threshold) {
          auto world_pt = gridToWorld(grid, gx, gy);
          world_pt.z() = obstacle_max_z;
          output.push_back(world_pt);
          is_obstacle(gx, gy) = true;
        } else {
          free_out.push_back(gridToWorld(grid, gx, gy));  // 平坦地面/平缓坡：空闲证据
        }
        continue;
      }

      const float vertical_gap = z_low - ground_ref;
      bool pass = vertical_gap >= config_.robot_height;
      if (!pass && use_evidence &&
          vertical_gap >= config_.min_overhang_clearance) {
        // 体素量化/杆底拖点使间隙偏小：下沿到地面之间被射线扫空才放行
        const int run = free_run_below(gy * grid.width + gx, z_low);
        pass = run >= config_.overhang_free_voxels &&
               reaches_ground(free_bottom(z_low, run), ground_ref);
      }
      if (pass) {
        // 下方间隙足够，可从下方通过（限高杆、隧道、桌下）
        free_out.push_back(gridToWorld(grid, gx, gy));
      } else {
        // 间隙不足：台阶立面、墙、立柱、陡坡
        auto world_pt = gridToWorld(grid, gx, gy);
        world_pt.z() = obstacle_max_z;
        output.push_back(world_pt);
        is_obstacle(gx, gy) = true;
      }
    }
  }

  // ================= 空列空闲证据（需要 3D 证据）=================
  // 本格没有任何占据体素时，上面的判据不下结论，2D 里以前写进去的障碍就永远清不掉
  // （例：低地看到台面下方的假地面 → 台面判矮障碍；上高地后台面体素被掠射擦掉，只剩空列）。
  // 这里补一条：本格列为空，且邻域地面往上到机身高度这段已被射线证实为空闲 → 空闲证据。
  // 邻接保护：8 邻域本帧有障碍格时不给（台阶边沿保留的那 1 格靠邻格梯度判出，本身常是空列）。
  if (use_evidence) {
    const int R = config_.overhang_ground_radius;
    const int band = static_cast<int>(std::lround(config_.robot_height / res));
    std::vector<float> nb;
    nb.reserve((2 * R + 1) * (2 * R + 1));
    for (int gy = 0; gy < grid.height; ++gy) {
      for (int gx = 0; gx < grid.width; ++gx) {
        const int idx = gy * grid.width + gx;
        if (!height_lists[idx].empty()) continue;  // 有占据：已由上面判据处理
        bool near_obstacle = false;
        for (int dy = -1; dy <= 1 && !near_obstacle; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height) continue;
            if (is_obstacle(nx, ny)) { near_obstacle = true; break; }
          }
        }
        if (near_obstacle) continue;
        // 参考地面：邻域（非悬空）列最低占据的中位数
        nb.clear();
        for (int dy = -R; dy <= R; ++dy) {
          for (int dx = -R; dx <= R; ++dx) {
            const int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height) continue;
            const float v = ground_raw(nx, ny);
            if (!std::isnan(v)) nb.push_back(v);
          }
        }
        if (nb.size() < 3) continue;  // 邻域地面不足：不下结论
        auto mid = nb.begin() + (nb.size() - 1) / 2;
        std::nth_element(nb.begin(), mid, nb.end());
        const float g_ref = *mid;
        // 本格中心（与体素中心对齐），从参考地面上一格到机身高度逐格查
        const utils::Vec3f c = gridToWorld(grid, gx, gy);
        int unknown = 0;
        bool ok = true;
        for (int k = 1; k <= band; ++k) {
          const auto t = voxel_query(utils::Vec3f(c.x(), c.y(), g_ref + k * res));
          if (t == utils::KNOWN_FREE) continue;
          if (t == utils::UNKNOWN && ++unknown <= config_.overhang_unknown_skip) continue;
          ok = false;  // 占据 / 出图 / 未知过多
          break;
        }
        if (ok) free_out.push_back(c);
      }
    }
  }
  // =========================================================
  return output;
}
// auto TerrainAnalyzer::analyze(const utils::RobotState &robot_pose,
//                               const Map &occ_map) -> Map {
//   GridInfo grid = setupGrid(robot_pose.p);

//   // 1. 存储每个网格内的完整点（Vec3f）
//   std::vector<std::vector<utils::Vec3f>> points_in_grid(grid.width *
//                                                           grid.height);
//   for (const auto &pt : occ_map) {
//     auto idx_opt = worldToGrid(grid, pt);
//     if (!idx_opt)
//       continue;
//     auto [gx, gy] = *idx_opt;
//     points_in_grid[gy * grid.width + gx].push_back(pt);
//   }

//   // 2. 提取最低点（地面）和最高点
//   Eigen::ArrayXXf ground = Eigen::ArrayXXf::Constant(
//       grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());
//   Eigen::ArrayXXf max_height = Eigen::ArrayXXf::Constant(
//       grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());

//   for (int gy = 0; gy < grid.height; ++gy) {
//     for (int gx = 0; gx < grid.width; ++gx) {
//       const auto &pts = points_in_grid[gy * grid.width + gx];
//       if (pts.empty())
//         continue;

//       float min_z = std::numeric_limits<float>::max();
//       float max_z = -std::numeric_limits<float>::max();
//       for (const auto &pt : pts) {
//         float z = pt.z();
//         if (z < min_z)
//           min_z = z;
//         if (z > max_z)
//           max_z = z;
//       }
//       ground(gx, gy) = min_z;
//       max_height(gx, gy) = max_z;
//     }
//   }

//   // 3. 中值滤波
//   ground = medianFilter(ground, config_.kernel_size);

//   // 4. 计算梯度（保持不变）
//   Eigen::ArrayXXf gradient = Eigen::ArrayXXf::Zero(grid.width, grid.height);
//   for (int gy = 0; gy < grid.height; ++gy) {
//     for (int gx = 0; gx < grid.width; ++gx) {
//       if (std::isnan(ground(gx, gy)))
//         continue;
//       float max_diff = 0.0f;
//       for (int dy = -1; dy <= 1; ++dy) {
//         for (int dx = -1; dx <= 1; ++dx) {
//           if (dx == 0 && dy == 0)
//             continue;
//           int nx = gx + dx;
//           int ny = gy + dy;
//           if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height)
//             continue;
//           float neighbor = ground(nx, ny);
//           if (std::isnan(neighbor))
//             continue;
//           float diff = std::abs(ground(gx, gy) - neighbor);
//           if (diff > max_diff)
//             max_diff = diff;
//         }
//       }
//       gradient(gx, gy) = max_diff;
//     }
//   }

//   // 5. 综合判断并输出障碍物（输出原始点）
//   Map output;
//   output.reserve(grid.width * grid.height * 10); // 预留空间

//   for (int gy = 0; gy < grid.height; ++gy) {
//     for (int gx = 0; gx < grid.width; ++gx) {
//       float current_ground = ground(gx, gy);
//       if (std::isnan(current_ground))
//         continue;

//       float obstacle_max_z = max_height(gx, gy);
//       if (std::isnan(obstacle_max_z))
//         continue;

//       float high_diff = obstacle_max_z - current_ground;
//       bool is_steep_edge = (gradient(gx, gy) > config_.steep_threshold);

//       // 【情况A】陡峭边缘 → 输出所有高于地面的点
//       if (is_steep_edge) {
//         for (const auto &pt : points_in_grid[gy * grid.width + gx]) {
//           if (pt.z() > current_ground + 0.05f) {
//             output.push_back(pt);
//           }
//         }
//         continue;
//       }

//       // 【情况B】非陡峭，但内部高差过大 → 进一步做间隙分析
//       if (high_diff <= config_.max_step_height)
//         continue;

//       // 间隙分析（原逻辑，需使用存储的点进行高度排序）
//       const auto &pts = points_in_grid[gy * grid.width + gx];
//       if (pts.empty())
//         continue;

//       std::vector<float> heights;
//       heights.reserve(pts.size());
//       for (const auto &pt : pts) {
//         float z = pt.z();
//         if (z >= current_ground - 0.05f) { // 过滤过低噪声
//           heights.push_back(z);
//         }
//       }
//       if (heights.empty())
//         continue;

//       std::sort(heights.begin(), heights.end());
//       float last_z = current_ground;
//       float max_gap = 0.0f;
//       for (float z : heights) {
//         if (z <= last_z)
//           continue;
//         float gap = z - last_z;
//         if (gap > max_gap)
//           max_gap = gap;
//         last_z = z;
//       }

//       bool blocked = (max_gap < config_.robot_height);
//       if (blocked && obstacle_max_z > 0.15f) {
//         // 输出该网格内所有高于地面的点
//         for (const auto &pt : pts) {
//           if (pt.z() > current_ground + 0.05f) {
//             output.push_back(pt);
//           }
//         }
//       }
//     }
//   }
//   return output;
// }
// ========== 工具函数实现 ==========

auto TerrainAnalyzer::setupGrid(const utils::Vec3f &robot_pos) const
    -> GridInfo {
  GridInfo grid;
  // 将分析窗口原点对齐到 resolution 网格上，保证网格中心在世界坐标系下
  // 保持固定，避免机器人连续运动时地形点发生亚网格级抖动。
  const float res = config_.resolution;
  const float ox =
      std::round((robot_pos.x() - config_.map_size_x * 0.5f) / res) * res;
  const float oy =
      std::round((robot_pos.y() - config_.map_size_y * 0.5f) / res) * res;
  grid.origin = utils::Vec3f(ox, oy, 0.0f);
  grid.width =
      static_cast<int>(std::ceil(config_.map_size_x / config_.resolution));
  grid.height =
      static_cast<int>(std::ceil(config_.map_size_y / config_.resolution));
  return grid;
}

auto TerrainAnalyzer::worldToGrid(const GridInfo &grid,
                                  const utils::Vec3f &world) const
    -> std::optional<std::pair<int, int>> {
  int gx = static_cast<int>(
      std::floor((world.x() - grid.origin.x()) / config_.resolution));
  int gy = static_cast<int>(
      std::floor((world.y() - grid.origin.y()) / config_.resolution));
  if (gx >= 0 && gx < grid.width && gy >= 0 && gy < grid.height) {
    return std::make_pair(gx, gy);
  }
  return std::nullopt;
}

auto TerrainAnalyzer::gridToWorld(const GridInfo &grid, int gx, int gy) const
    -> utils::Vec3f {
  float wx =
      grid.origin.x() + (static_cast<float>(gx) + 0.5f) * config_.resolution;
  float wy =
      grid.origin.y() + (static_cast<float>(gy) + 0.5f) * config_.resolution;
  return utils::Vec3f(wx, wy, 0.0f);
}

auto TerrainAnalyzer::medianFilter(const Eigen::ArrayXXf &input,
                                   int kernel_size) -> Eigen::ArrayXXf {
  int rows = input.rows();
  int cols = input.cols();
  Eigen::ArrayXXf output = Eigen::ArrayXXf::Constant(
      rows, cols, std::numeric_limits<float>::quiet_NaN());
  int k = kernel_size / 2;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      std::vector<float> neighbors;
      neighbors.reserve(kernel_size * kernel_size);

      for (int m = -k; m <= k; ++m) {
        for (int n = -k; n <= k; ++n) {
          int ni = i + m;
          int nj = j + n;
          if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
            float val = input(ni, nj);
            if (!std::isnan(val)) {
              neighbors.push_back(val);
            }
          }
        }
      }

      if (!neighbors.empty()) {
        std::sort(neighbors.begin(), neighbors.end());
        size_t mid = neighbors.size() / 2;
        if (neighbors.size() % 2 == 0) {
          output(i, j) = (neighbors[mid - 1] + neighbors[mid]) * 0.5f;
        } else {
          output(i, j) = neighbors[mid];
        }
      }
    }
  }
  return output;
}

} // namespace Terrain