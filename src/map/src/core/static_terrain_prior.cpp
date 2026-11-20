#include "map/core/static_terrain_prior.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <vector>

namespace ma_map {

namespace {

    namespace fs = std::filesystem;

    struct TerrainBinHeader {
        char magic[4];
        uint32_t version;
        int32_t nx;
        int32_t ny;
        double resolution;
        double origin_x;
        double origin_y;
        uint64_t reserved;
    };

    static_assert(sizeof(TerrainBinHeader) == 48, "TerrainBinHeader must be 48 bytes");

    auto hasSuffix(std::string_view s, std::string_view suffix) -> bool {
        return s.size() >= suffix.size() && s.substr(s.size() - suffix.size()) == suffix;
    }

    auto medianOf(std::vector<float>& vals) -> float {
        const size_t n = vals.size();
        const size_t mid = n / 2;
        std::nth_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(mid), vals.end());
        const float hi = vals[mid];
        if ((n & 1U) != 0U) {
            return hi;
        }
        const float lo = *std::max_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(mid));
        return 0.5f * (lo + hi);
    }

} // namespace

auto StaticTerrainPrior::createEmpty(const grid_map::GridGeometry& geom) -> StaticTerrainPrior {
    StaticTerrainPrior prior;
    if (!geom.isValid()) {
        return prior;
    }
    prior.geometry = geom;
    const size_t n = geom.cellCount();
    prior.ground_z.assign(n, std::numeric_limits<float>::quiet_NaN());
    prior.overhang_min_z.assign(n, std::numeric_limits<float>::infinity());
    return prior;
}

auto StaticTerrainPrior::sampleGroundZ(const Eigen::Vector2d& world_xy, int fallback_radius_cells) const -> float {
    if (!isValid() || !world_xy.allFinite() || !geometry.isInside(world_xy)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const Eigen::Vector2i id = geometry.posToIndex(world_xy);
    const int nx = geometry.voxel_num.x();
    const int ny = geometry.voxel_num.y();
    const float exact = ground_z[static_cast<size_t>(geometry.toAddress(id.x(), id.y()))];
    if (std::isfinite(exact)) {
        return exact;
    }
    if (fallback_radius_cells <= 0) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    std::vector<float> ring_vals;
    ring_vals.reserve(static_cast<size_t>(8 * fallback_radius_cells));
    for (int r = 1; r <= fallback_radius_cells; ++r) {
        ring_vals.clear();
        const int x_lo = std::max(0, id.x() - r);
        const int x_hi = std::min(nx - 1, id.x() + r);
        const int y_lo = std::max(0, id.y() - r);
        const int y_hi = std::min(ny - 1, id.y() + r);
        for (int ix = x_lo; ix <= x_hi; ++ix) {
            const size_t row_off = static_cast<size_t>(ix) * static_cast<size_t>(ny);
            for (int iy = y_lo; iy <= y_hi; ++iy) {
                if (std::max(std::abs(ix - id.x()), std::abs(iy - id.y())) != r) {
                    continue;
                }
                const float gz = ground_z[row_off + static_cast<size_t>(iy)];
                if (std::isfinite(gz)) {
                    ring_vals.push_back(gz);
                }
            }
        }
        if (!ring_vals.empty()) {
            return medianOf(ring_vals);
        }
    }
    return std::numeric_limits<float>::quiet_NaN();
}

auto StaticTerrainPrior::sampleOverhangMinZ(const Eigen::Vector2d& world_xy, int radius_cells) const -> float {
    if (!isValid() || !world_xy.allFinite() || !geometry.isInside(world_xy)) {
        return std::numeric_limits<float>::infinity();
    }
    const Eigen::Vector2i id = geometry.posToIndex(world_xy);
    const int nx = geometry.voxel_num.x();
    const int ny = geometry.voxel_num.y();
    const int r = std::max(0, radius_cells);
    const int x_lo = std::max(0, id.x() - r);
    const int x_hi = std::min(nx - 1, id.x() + r);
    const int y_lo = std::max(0, id.y() - r);
    const int y_hi = std::min(ny - 1, id.y() + r);

    float min_z = std::numeric_limits<float>::infinity();
    for (int ix = x_lo; ix <= x_hi; ++ix) {
        const size_t row_off = static_cast<size_t>(ix) * static_cast<size_t>(ny);
        for (int iy = y_lo; iy <= y_hi; ++iy) {
            const float oz = overhang_min_z[row_off + static_cast<size_t>(iy)];
            if (std::isfinite(oz) && oz < min_z) {
                min_z = oz;
            }
        }
    }
    return min_z;
}

auto StaticTerrainPrior::crop(int x_min, int x_max, int y_min, int y_max) const -> StaticTerrainPrior {
    if (!isValid()) {
        return {};
    }
    const int nx = geometry.voxel_num.x();
    const int ny = geometry.voxel_num.y();
    if (x_min < 0 || y_min < 0 || x_max >= nx || y_max >= ny || x_min > x_max || y_min > y_max) {
        return {};
    }

    const int crop_nx = x_max - x_min + 1;
    const int crop_ny = y_max - y_min + 1;
    const Eigen::Vector2d new_origin(
        geometry.origin.x() + static_cast<double>(x_min) * geometry.resolution,
        geometry.origin.y() + static_cast<double>(y_min) * geometry.resolution
    );
    const grid_map::GridGeometry new_geom =
        grid_map::GridGeometry::fromOriginAndSize(new_origin, Eigen::Vector2i(crop_nx, crop_ny), geometry.resolution);

    StaticTerrainPrior out = createEmpty(new_geom);
    for (int ix = 0; ix < crop_nx; ++ix) {
        const size_t src_off = static_cast<size_t>(x_min + ix) * static_cast<size_t>(ny) + static_cast<size_t>(y_min);
        const size_t dst_off = static_cast<size_t>(ix) * static_cast<size_t>(crop_ny);
        std::copy_n(ground_z.data() + src_off, static_cast<size_t>(crop_ny), out.ground_z.data() + dst_off);
        std::copy_n(
            overhang_min_z.data() + src_off,
            static_cast<size_t>(crop_ny),
            out.overhang_min_z.data() + dst_off
        );
    }
    return out;
}

auto StaticTerrainPrior::interpolateMissingGround(std::span<const uint8_t> target_mask, int search_radius) -> int {
    if (!isValid() || target_mask.size() != geometry.cellCount() || search_radius <= 0) {
        return 0;
    }
    const int nx = geometry.voxel_num.x();
    const int ny = geometry.voxel_num.y();
    const std::vector<float> src_ground = ground_z;
    std::vector<float> ring_vals;
    ring_vals.reserve(static_cast<size_t>(8 * search_radius));

    int filled = 0;
    for (int ix = 0; ix < nx; ++ix) {
        const size_t row_off = static_cast<size_t>(ix) * static_cast<size_t>(ny);
        for (int iy = 0; iy < ny; ++iy) {
            const size_t idx = row_off + static_cast<size_t>(iy);
            if (target_mask[idx] == 0 || std::isfinite(src_ground[idx])) {
                continue;
            }
            for (int r = 1; r <= search_radius; ++r) {
                ring_vals.clear();
                const int x_lo = std::max(0, ix - r);
                const int x_hi = std::min(nx - 1, ix + r);
                const int y_lo = std::max(0, iy - r);
                const int y_hi = std::min(ny - 1, iy + r);
                for (int jx = x_lo; jx <= x_hi; ++jx) {
                    const size_t jrow = static_cast<size_t>(jx) * static_cast<size_t>(ny);
                    for (int jy = y_lo; jy <= y_hi; ++jy) {
                        if (std::max(std::abs(jx - ix), std::abs(jy - iy)) != r) {
                            continue;
                        }
                        const float gz = src_ground[jrow + static_cast<size_t>(jy)];
                        if (std::isfinite(gz)) {
                            ring_vals.push_back(gz);
                        }
                    }
                }
                if (!ring_vals.empty()) {
                    ground_z[idx] = medianOf(ring_vals);
                    ++filled;
                    break;
                }
            }
        }
    }
    return filled;
}

auto StaticTerrainPrior::companionPath(std::string_view map_path) -> std::string {
    if (map_path.empty()) {
        return {};
    }
    if (hasSuffix(map_path, ".terrain.bin")) {
        return std::string(map_path);
    }
    if (hasSuffix(map_path, ".pgm")) {
        return std::string(map_path.substr(0, map_path.size() - 4)) + ".terrain.bin";
    }
    if (hasSuffix(map_path, ".yaml")) {
        return std::string(map_path.substr(0, map_path.size() - 5)) + ".terrain.bin";
    }
    if (hasSuffix(map_path, ".yml")) {
        return std::string(map_path.substr(0, map_path.size() - 4)) + ".terrain.bin";
    }
    fs::path p(map_path);
    p.replace_extension(".terrain.bin");
    return p.string();
}

auto StaticTerrainPrior::save(std::string_view path, const StaticTerrainPrior& prior)
    -> tl::expected<void, std::string> {
    if (path.empty()) {
        return tl::make_unexpected("Empty terrain prior output path");
    }
    if (!prior.isValid()) {
        return tl::make_unexpected("Cannot save invalid StaticTerrainPrior");
    }

    const fs::path dst(path);
    const fs::path parent = dst.parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        fs::create_directories(parent, ec);
        if (ec) {
            return tl::make_unexpected(
                "Failed to create terrain prior directory " + parent.string() + ": " + ec.message()
            );
        }
    }

    TerrainBinHeader hdr {};
    std::memcpy(hdr.magic, kMagic, sizeof(kMagic));
    hdr.version = kFormatVersion;
    hdr.nx = prior.geometry.voxel_num.x();
    hdr.ny = prior.geometry.voxel_num.y();
    hdr.resolution = prior.geometry.resolution;
    hdr.origin_x = prior.geometry.origin.x();
    hdr.origin_y = prior.geometry.origin.y();
    hdr.reserved = 0;

    const fs::path tmp(dst.string() + ".tmp");
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out.is_open()) {
            return tl::make_unexpected("Failed to open temp terrain prior file: " + tmp.string());
        }
        const auto array_bytes = static_cast<std::streamsize>(prior.ground_z.size() * sizeof(float));
        out.write(reinterpret_cast<const char*>(&hdr), static_cast<std::streamsize>(sizeof(hdr)));
        out.write(reinterpret_cast<const char*>(prior.ground_z.data()), array_bytes);
        out.write(reinterpret_cast<const char*>(prior.overhang_min_z.data()), array_bytes);
        out.flush();
        if (!out.good()) {
            std::error_code ec;
            fs::remove(tmp, ec);
            return tl::make_unexpected("Failed writing temp terrain prior file: " + tmp.string());
        }
    }

    std::error_code ec;
    fs::rename(tmp, dst, ec);
    if (ec) {
        std::error_code rm_ec;
        fs::remove(tmp, rm_ec);
        return tl::make_unexpected("Failed replacing terrain prior file " + dst.string() + ": " + ec.message());
    }
    return {};
}

auto StaticTerrainPrior::load(std::string_view path) -> tl::expected<StaticTerrainPrior, std::string> {
    if (path.empty()) {
        return tl::make_unexpected("Empty terrain prior path");
    }
    const fs::path src(path);
    std::error_code ec;
    if (!fs::exists(src, ec) || ec) {
        return tl::make_unexpected("Terrain prior file not found: " + src.string());
    }
    const uintmax_t file_size = fs::file_size(src, ec);
    if (ec || file_size < sizeof(TerrainBinHeader)) {
        return tl::make_unexpected("Terrain prior file too small or unreadable: " + src.string());
    }

    std::ifstream in(src, std::ios::binary);
    if (!in.is_open()) {
        return tl::make_unexpected("Failed to open terrain prior file: " + src.string());
    }

    TerrainBinHeader hdr {};
    in.read(reinterpret_cast<char*>(&hdr), static_cast<std::streamsize>(sizeof(hdr)));
    if (!in.good()) {
        return tl::make_unexpected("Failed to read terrain prior header: " + src.string());
    }

    if (std::memcmp(hdr.magic, kMagic, sizeof(kMagic)) != 0) {
        return tl::make_unexpected("Invalid terrain prior magic header: " + src.string());
    }
    if (hdr.version != kFormatVersion) {
        return tl::make_unexpected(
            "Unsupported terrain prior version " + std::to_string(hdr.version) + " in " + src.string()
        );
    }

    const grid_map::GridGeometry geom = grid_map::GridGeometry::fromOriginAndSize(
        Eigen::Vector2d(hdr.origin_x, hdr.origin_y),
        Eigen::Vector2i(hdr.nx, hdr.ny),
        hdr.resolution
    );
    if (!geom.isValid()) {
        return tl::make_unexpected("Invalid geometry in terrain prior file: " + src.string());
    }

    const uint64_t expected_size =
        static_cast<uint64_t>(sizeof(TerrainBinHeader)) + 2ULL * static_cast<uint64_t>(geom.cellCount()) * sizeof(float);
    if (file_size != expected_size) {
        return tl::make_unexpected(
            "Terrain prior payload size mismatch in " + src.string() + ": expected " + std::to_string(expected_size)
            + " bytes, got " + std::to_string(file_size)
        );
    }

    StaticTerrainPrior out = createEmpty(geom);
    const auto array_bytes = static_cast<std::streamsize>(geom.cellCount() * sizeof(float));
    in.read(reinterpret_cast<char*>(out.ground_z.data()), array_bytes);
    in.read(reinterpret_cast<char*>(out.overhang_min_z.data()), array_bytes);
    if (!in.good()) {
        return tl::make_unexpected("Failed reading terrain prior payload: " + src.string());
    }
    return out;
}

auto StaticTerrainPrior::load(std::string_view path, const grid_map::GridGeometry& expected_geom)
    -> tl::expected<StaticTerrainPrior, std::string> {
    if (!expected_geom.isValid()) {
        return tl::make_unexpected("Cannot validate terrain prior against invalid expected geometry");
    }
    auto loaded = load(path);
    if (!loaded) {
        return loaded;
    }
    const auto& g = loaded->geometry;
    if (g.voxel_num != expected_geom.voxel_num || std::abs(g.resolution - expected_geom.resolution) > 1e-6
        || (g.origin - expected_geom.origin).cwiseAbs().maxCoeff() > 1e-4)
    {
        return tl::make_unexpected(
            "Terrain prior geometry mismatch with static PGM/YAML: prior=(" + std::to_string(g.voxel_num.x()) + "x"
            + std::to_string(g.voxel_num.y()) + ", res=" + std::to_string(g.resolution) + ") vs map=("
            + std::to_string(expected_geom.voxel_num.x()) + "x" + std::to_string(expected_geom.voxel_num.y())
            + ", res=" + std::to_string(expected_geom.resolution) + ")"
        );
    }
    // 消除微米级浮点舍入差异，与静态图几何保持逐位一致
    loaded->geometry = expected_geom;
    return loaded;
}

} // namespace ma_map
