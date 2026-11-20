#include "map/core/map_io.hpp"

#include "utils/logger.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>

namespace ma_map {

namespace {

    namespace fs = std::filesystem;

    constexpr uint8_t kPgmOccupied = 0;
    constexpr uint8_t kPgmUnknown = 205;
    constexpr uint8_t kPgmFree = 254;

    auto hasSuffix(std::string_view s, std::string_view suffix) -> bool {
        return s.size() >= suffix.size() && s.substr(s.size() - suffix.size()) == suffix;
    }

    auto skipPgmHeaderSpace(std::ifstream& f) -> void {
        while (true) {
            const int c = f.peek();
            if (c == '#') {
                f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                f.get();
            } else {
                break;
            }
        }
    }

    auto writeFileAtomically(const fs::path& dst, const std::string& content, bool binary)
        -> tl::expected<void, std::string> {
        const fs::path tmp = fs::path(dst.string() + ".tmp");
        const std::ios::openmode mode = binary ? (std::ios::binary | std::ios::trunc) : std::ios::trunc;
        {
            std::ofstream out(tmp, mode);
            if (!out.is_open()) {
                return tl::make_unexpected("Failed to open temp file for writing: " + tmp.string());
            }
            out.write(content.data(), static_cast<std::streamsize>(content.size()));
            out.flush();
            if (!out.good()) {
                std::error_code ec;
                fs::remove(tmp, ec);
                return tl::make_unexpected("Failed while writing file: " + tmp.string());
            }
        }
        std::error_code ec;
        fs::rename(tmp, dst, ec);
        if (ec) {
            std::error_code rm_ec;
            fs::remove(tmp, rm_ec);
            return tl::make_unexpected("Failed to move temp file to " + dst.string() + ": " + ec.message());
        }
        return {};
    }

    // 清理完全被 FREE 包围的内部孤立小 UNKNOWN 洞（4 连通，面积 <= max_pinhole_cells，
    // 不接触裁剪图边界，且不与任何 OCCUPIED 格相邻），避免单帧射线漏扫在重载后变成孤立障碍点。
    auto cleanupFreePinholes(std::vector<CellState>& grid, int nx, int ny, int max_pinhole_cells) -> int {
        if (nx <= 2 || ny <= 2 || max_pinhole_cells <= 0) {
            return 0;
        }

        std::vector<uint8_t> visited(static_cast<size_t>(nx) * static_cast<size_t>(ny), 0);
        std::vector<int> component;
        component.reserve(static_cast<size_t>(max_pinhole_cells) + 4);
        std::queue<int> q;

        constexpr int kDx[4] = {-1, 1, 0, 0};
        constexpr int kDy[4] = {0, 0, -1, 1};

        int cleaned_cells = 0;
        for (int x = 0; x < nx; ++x) {
            for (int y = 0; y < ny; ++y) {
                const int start = x * ny + y;
                if (grid[static_cast<size_t>(start)] != CellState::UNKNOWN || visited[static_cast<size_t>(start)] != 0)
                {
                    continue;
                }

                component.clear();
                bool valid_pinhole = true;
                visited[static_cast<size_t>(start)] = 1;
                q.push(start);

                while (!q.empty()) {
                    const int cur = q.front();
                    q.pop();
                    component.push_back(cur);

                    const int cx = cur / ny;
                    const int cy = cur % ny;
                    if (cx == 0 || cy == 0 || cx == nx - 1 || cy == ny - 1) {
                        valid_pinhole = false;
                    }
                    if (static_cast<int>(component.size()) > max_pinhole_cells) {
                        valid_pinhole = false;
                    }

                    for (int k = 0; k < 4; ++k) {
                        const int nx_i = cx + kDx[k];
                        const int ny_i = cy + kDy[k];
                        if (nx_i < 0 || ny_i < 0 || nx_i >= nx || ny_i >= ny) {
                            continue;
                        }
                        const int nidx = nx_i * ny + ny_i;
                        const CellState st = grid[static_cast<size_t>(nidx)];
                        if (st == CellState::OCCUPIED) {
                            valid_pinhole = false;
                        } else if (st == CellState::UNKNOWN && visited[static_cast<size_t>(nidx)] == 0) {
                            visited[static_cast<size_t>(nidx)] = 1;
                            q.push(nidx);
                        }
                    }
                }

                if (valid_pinhole) {
                    for (const int idx: component) {
                        grid[static_cast<size_t>(idx)] = CellState::FREE;
                    }
                    cleaned_cells += static_cast<int>(component.size());
                }
            }
        }
        return cleaned_cells;
    }

} // namespace

auto MapIO::load(std::string_view map_path, const LoadOptions& options) -> tl::expected<LoadedMap, std::string> {
    if (map_path.empty()) {
        return tl::make_unexpected("Empty map path");
    }

    fs::path pgm_path;
    fs::path yaml_path;
    if (hasSuffix(map_path, ".yaml") || hasSuffix(map_path, ".yml")) {
        yaml_path = fs::path(map_path);
    } else if (hasSuffix(map_path, ".pgm")) {
        pgm_path = fs::path(map_path);
        yaml_path = pgm_path;
        yaml_path.replace_extension(".yaml");
    } else {
        return tl::make_unexpected("Unsupported map path (expected .pgm or .yaml): " + std::string(map_path));
    }

    // 1. 解析 YAML 元数据（分辨率、原点、阈值）
    double resolution = 0.0;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    int negate = 0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;

    try {
        const YAML::Node root = YAML::LoadFile(yaml_path.string());
        if (!root["resolution"] || !root["origin"]) {
            return tl::make_unexpected("Map yaml missing resolution or origin: " + yaml_path.string());
        }
        resolution = root["resolution"].as<double>();
        const auto origin_vec = root["origin"].as<std::vector<double>>();
        if (origin_vec.size() < 2 || !std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(origin_vec[0])
            || !std::isfinite(origin_vec[1]))
        {
            return tl::make_unexpected("Map yaml has invalid resolution/origin: " + yaml_path.string());
        }
        origin = Eigen::Vector2d(origin_vec[0], origin_vec[1]);

        if (root["negate"]) {
            negate = root["negate"].as<int>();
        }
        if (root["occupied_thresh"]) {
            occupied_thresh = root["occupied_thresh"].as<double>();
        }
        if (root["free_thresh"]) {
            free_thresh = root["free_thresh"].as<double>();
        }
        if (!std::isfinite(occupied_thresh) || !std::isfinite(free_thresh) || occupied_thresh < free_thresh) {
            return tl::make_unexpected("Map yaml has invalid occupied_thresh/free_thresh: " + yaml_path.string());
        }

        if (pgm_path.empty()) {
            if (root["image"]) {
                const fs::path img(root["image"].as<std::string>());
                pgm_path = img.is_absolute() ? img : (yaml_path.parent_path() / img);
            } else {
                pgm_path = yaml_path;
                pgm_path.replace_extension(".pgm");
            }
        }
    } catch (const std::exception& e) {
        return tl::make_unexpected("Failed to parse map yaml " + yaml_path.string() + ": " + e.what());
    }

    // 2. 读取 P5 二进制 PGM
    std::ifstream f(pgm_path, std::ios::binary);
    if (!f.is_open()) {
        return tl::make_unexpected("Failed to open PGM file: " + pgm_path.string());
    }
    std::string magic;
    if (!(f >> magic) || magic != "P5") {
        return tl::make_unexpected("Only P5 binary PGM is supported: " + pgm_path.string());
    }

    int w = 0;
    int h = 0;
    int maxval = 0;
    skipPgmHeaderSpace(f);
    if (!(f >> w)) {
        return tl::make_unexpected("Invalid PGM width: " + pgm_path.string());
    }
    skipPgmHeaderSpace(f);
    if (!(f >> h)) {
        return tl::make_unexpected("Invalid PGM height: " + pgm_path.string());
    }
    skipPgmHeaderSpace(f);
    if (!(f >> maxval)) {
        return tl::make_unexpected("Invalid PGM maxval: " + pgm_path.string());
    }
    if (maxval != 255 || w <= 0 || h <= 0) {
        return tl::make_unexpected("Only P5 binary PGM (maxval=255, positive size) supported: " + pgm_path.string());
    }

    // PGM 规范：maxval 后仅消耗一个空白字符（兼容 CRLF），避免吞掉二进制像素首字节。
    const int sep = f.get();
    if (sep == '\r' && f.peek() == '\n') {
        f.get();
    }

    const size_t pixel_count = static_cast<size_t>(w) * static_cast<size_t>(h);
    std::vector<uint8_t> pixels(pixel_count);
    f.read(reinterpret_cast<char*>(pixels.data()), static_cast<std::streamsize>(pixel_count));
    if (f.gcount() != static_cast<std::streamsize>(pixel_count)) {
        return tl::make_unexpected("PGM pixel data truncated: " + pgm_path.string());
    }

    // 3. 按既有轴约定组装严格匹配尺寸的 LoadedMap：
    //    nx = h（世界 X），ny = w（世界 Y），pgm_row = h - 1 - ix，pgm_col = iy。
    LoadedMap out;
    out.geometry = grid_map::GridGeometry::fromOriginAndSize(origin, Eigen::Vector2i(h, w), resolution);
    if (!out.geometry.isValid()) {
        return tl::make_unexpected("Loaded map produced invalid GridGeometry: " + pgm_path.string());
    }

    out.cells.resize(pixel_count, CellState::UNKNOWN);
    out.occupancy.resize(pixel_count, 0);

    for (int ix = 0; ix < h; ++ix) {
        const int pgm_row = h - 1 - ix;
        const size_t pgm_row_off = static_cast<size_t>(pgm_row) * static_cast<size_t>(w);
        const size_t grid_row_off = static_cast<size_t>(ix) * static_cast<size_t>(w);
        for (int iy = 0; iy < w; ++iy) {
            const uint8_t px = pixels[pgm_row_off + static_cast<size_t>(iy)];
            const double occ_prob =
                (negate != 0) ? (static_cast<double>(px) / 255.0) : (static_cast<double>(255 - px) / 255.0);
            const size_t dst = grid_row_off + static_cast<size_t>(iy);
            if (occ_prob > occupied_thresh) {
                out.cells[dst] = CellState::OCCUPIED;
                out.occupancy[dst] = 1;
                ++out.occupied_count;
            } else if (occ_prob < free_thresh) {
                out.cells[dst] = CellState::FREE;
                out.occupancy[dst] = 0;
                ++out.free_count;
            } else {
                out.cells[dst] = CellState::UNKNOWN;
                out.occupancy[dst] = options.unknown_as_occupied ? 1 : 0;
                ++out.unknown_count;
            }
        }
    }

    // 4. 自动查找并加载伴随 2.5D 静态地形先验（<stem>.terrain.bin）；缺失或失配时优雅降级，不阻塞加载。
    std::string terrain_candidate = StaticTerrainPrior::companionPath(pgm_path.string());
    std::error_code terrain_ec;
    if (!fs::exists(terrain_candidate, terrain_ec) || terrain_ec) {
        const std::string yaml_candidate = StaticTerrainPrior::companionPath(yaml_path.string());
        terrain_ec.clear();
        if (yaml_candidate != terrain_candidate && fs::exists(yaml_candidate, terrain_ec) && !terrain_ec) {
            terrain_candidate = yaml_candidate;
        } else {
            terrain_candidate.clear();
        }
    }
    if (!terrain_candidate.empty()) {
        auto prior_res = StaticTerrainPrior::load(terrain_candidate, out.geometry);
        if (prior_res) {
            out.terrain_prior = std::move(*prior_res);
            out.terrain_prior_path = terrain_candidate;
        } else {
            logger::MaMap->warn(
                "Companion terrain prior ignored ({}): {}; falling back to 2D static map without 2.5D prior",
                terrain_candidate,
                prior_res.error()
            );
        }
    }

    return out;
}

auto MapIO::save(
    std::string_view output_path,
    const grid_map::GridGeometry& geometry,
    std::span<const CellState> cells,
    const SaveOptions& options,
    const StaticTerrainPrior* terrain_prior
) -> tl::expected<SaveReport, std::string> {
    if (output_path.empty()) {
        return tl::make_unexpected("Empty output path");
    }
    if (!geometry.isValid()) {
        return tl::make_unexpected("Cannot save map: invalid GridGeometry");
    }
    if (cells.size() != geometry.cellCount()) {
        return tl::make_unexpected("Cannot save map: cell buffer size does not match GridGeometry");
    }
    const bool write_prior = (terrain_prior != nullptr) && options.save_terrain_prior;
    if (write_prior && (!terrain_prior->isValid() || terrain_prior->geometry != geometry)) {
        return tl::make_unexpected("Cannot save map: StaticTerrainPrior geometry does not match GridGeometry");
    }

    fs::path pgm_path;
    fs::path yaml_path;
    if (hasSuffix(output_path, ".pgm")) {
        pgm_path = fs::path(output_path);
        yaml_path = pgm_path;
        yaml_path.replace_extension(".yaml");
    } else if (hasSuffix(output_path, ".yaml") || hasSuffix(output_path, ".yml")) {
        yaml_path = fs::path(output_path);
        pgm_path = yaml_path;
        pgm_path.replace_extension(".pgm");
    } else {
        const fs::path dir(output_path);
        pgm_path = dir / "global_map.pgm";
        yaml_path = dir / "global_map.yaml";
    }

    const fs::path parent = pgm_path.parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        fs::create_directories(parent, ec);
        if (ec) {
            return tl::make_unexpected("Failed to create output directory " + parent.string() + ": " + ec.message());
        }
    }

    const int nx = geometry.voxel_num.x();
    const int ny = geometry.voxel_num.y();

    int x_min = 0;
    int x_max = nx - 1;
    int y_min = 0;
    int y_max = ny - 1;

    if (options.crop_to_explored) {
        x_min = nx;
        x_max = -1;
        y_min = ny;
        y_max = -1;
        for (int x = 0; x < nx; ++x) {
            const size_t row_off = static_cast<size_t>(x) * static_cast<size_t>(ny);
            for (int y = 0; y < ny; ++y) {
                if (cells[row_off + static_cast<size_t>(y)] != CellState::UNKNOWN) {
                    x_min = std::min(x_min, x);
                    x_max = std::max(x_max, x);
                    y_min = std::min(y_min, y);
                    y_max = std::max(y_max, y);
                }
            }
        }
        if (x_max < x_min || y_max < y_min) {
            return tl::make_unexpected("Cannot save map: no explored cells");
        }
        const int margin = std::max(0, options.margin_cells);
        x_min = std::max(0, x_min - margin);
        x_max = std::min(nx - 1, x_max + margin);
        y_min = std::max(0, y_min - margin);
        y_max = std::min(ny - 1, y_max + margin);
    }

    const int crop_nx = x_max - x_min + 1;
    const int crop_ny = y_max - y_min + 1;
    std::vector<CellState> cropped(static_cast<size_t>(crop_nx) * static_cast<size_t>(crop_ny), CellState::UNKNOWN);
    for (int ix = 0; ix < crop_nx; ++ix) {
        const size_t src_off = static_cast<size_t>(x_min + ix) * static_cast<size_t>(ny) + static_cast<size_t>(y_min);
        const size_t dst_off = static_cast<size_t>(ix) * static_cast<size_t>(crop_ny);
        std::copy_n(cells.data() + src_off, static_cast<size_t>(crop_ny), cropped.data() + dst_off);
    }

    std::vector<CellState> pre_cleanup_cropped;
    if (write_prior && options.cleanup_free_pinholes && options.max_pinhole_cells > 0) {
        pre_cleanup_cropped = cropped;
    }

    int cleaned_pinholes = 0;
    if (options.cleanup_free_pinholes && options.max_pinhole_cells > 0) {
        cleaned_pinholes = cleanupFreePinholes(cropped, crop_nx, crop_ny, options.max_pinhole_cells);
    }

    const Eigen::Vector2d saved_origin(
        geometry.origin.x() + static_cast<double>(x_min) * geometry.resolution,
        geometry.origin.y() + static_cast<double>(y_min) * geometry.resolution
    );
    const grid_map::GridGeometry saved_geom =
        grid_map::GridGeometry::fromOriginAndSize(saved_origin, Eigen::Vector2i(crop_nx, crop_ny), geometry.resolution);

    // 编码 P5 PGM：pgm_w = crop_ny, pgm_h = crop_nx, pgm_row = crop_nx - 1 - ix, pgm_col = iy
    const int pgm_w = crop_ny;
    const int pgm_h = crop_nx;
    std::ostringstream header;
    header << "P5\n" << pgm_w << " " << pgm_h << "\n255\n";
    const std::string header_str = header.str();

    std::string pgm_bytes;
    pgm_bytes.resize(header_str.size() + static_cast<size_t>(pgm_w) * static_cast<size_t>(pgm_h));
    std::copy(header_str.begin(), header_str.end(), pgm_bytes.begin());

    int occ_cnt = 0;
    int free_cnt = 0;
    int unk_cnt = 0;
    char* pixel_dst = pgm_bytes.data() + header_str.size();
    for (int pgm_row = 0; pgm_row < pgm_h; ++pgm_row) {
        const int ix = crop_nx - 1 - pgm_row;
        const size_t src_row = static_cast<size_t>(ix) * static_cast<size_t>(crop_ny);
        const size_t dst_row = static_cast<size_t>(pgm_row) * static_cast<size_t>(pgm_w);
        for (int iy = 0; iy < pgm_w; ++iy) {
            const CellState st = cropped[src_row + static_cast<size_t>(iy)];
            uint8_t px = kPgmUnknown;
            if (st == CellState::OCCUPIED) {
                px = kPgmOccupied;
                ++occ_cnt;
            } else if (st == CellState::FREE) {
                px = kPgmFree;
                ++free_cnt;
            } else {
                px = kPgmUnknown;
                ++unk_cnt;
            }
            pixel_dst[dst_row + static_cast<size_t>(iy)] = static_cast<char>(px);
        }
    }

    auto pgm_res = writeFileAtomically(pgm_path, pgm_bytes, true);
    if (!pgm_res) {
        return tl::make_unexpected(pgm_res.error());
    }

    std::ostringstream yaml_out;
    yaml_out << std::setprecision(12);
    yaml_out << "image: " << pgm_path.filename().string() << "\n";
    yaml_out << "resolution: " << saved_geom.resolution << "\n";
    yaml_out << "origin: [" << saved_geom.origin.x() << ", " << saved_geom.origin.y() << ", 0.0]\n";
    yaml_out << "negate: 0\n";
    yaml_out << "occupied_thresh: 0.65\n";
    yaml_out << "free_thresh: 0.196\n";
    yaml_out << "mode: trinary\n";

    auto yaml_res = writeFileAtomically(yaml_path, yaml_out.str(), false);
    if (!yaml_res) {
        return tl::make_unexpected(yaml_res.error());
    }

    SaveReport report;
    report.pgm_path = pgm_path.string();
    report.yaml_path = yaml_path.string();
    report.saved_geometry = saved_geom;
    report.occupied_count = occ_cnt;
    report.free_count = free_cnt;
    report.unknown_count = unk_cnt;
    report.cleaned_pinholes = cleaned_pinholes;

    if (write_prior) {
        StaticTerrainPrior cropped_prior = terrain_prior->crop(x_min, x_max, y_min, y_max);
        if (!pre_cleanup_cropped.empty() && cleaned_pinholes > 0) {
            std::vector<uint8_t> filled_mask(cropped.size(), 0);
            for (size_t i = 0; i < cropped.size(); ++i) {
                if (pre_cleanup_cropped[i] == CellState::UNKNOWN && cropped[i] == CellState::FREE) {
                    filled_mask[i] = 1;
                }
            }
            cropped_prior.interpolateMissingGround(filled_mask, 2);
        }
        const std::string prior_path = StaticTerrainPrior::companionPath(pgm_path.string());
        auto prior_res = StaticTerrainPrior::save(prior_path, cropped_prior);
        if (!prior_res) {
            return tl::make_unexpected(prior_res.error());
        }
        report.terrain_prior_path = prior_path;
        report.saved_terrain_prior = true;
    }

    return report;
}

} // namespace ma_map
