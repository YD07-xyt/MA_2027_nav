#include "map/core/grid_map.hpp"

#include <atomic>
#include <cstring>

namespace grid_map {

auto GridMap::nextRevision() -> uint64_t {
    static std::atomic<uint64_t> counter {1};
    return counter.fetch_add(1, std::memory_order_relaxed);
}

auto GridMap::nextEsdfRevision() -> uint64_t {
    static std::atomic<uint64_t> counter {1};
    return counter.fetch_add(1, std::memory_order_relaxed);
}

auto GridMap::init(const GridGeometry& geom, OutOfBoundsPolicy oob_policy) -> void {
    geometry_ = geom;
    oob_policy_ = oob_policy;
    buffer_size_ = geometry_.isValid() ? static_cast<int>(geometry_.cellCount()) : 0;

    occ_buffer_.assign(static_cast<size_t>(buffer_size_), 0);
    esdf_buffer_.assign(static_cast<size_t>(buffer_size_), 0.0);
    semantics_buffer_.assign(static_cast<size_t>(buffer_size_), static_cast<uint8_t>(Semantics::NONE));
    esdf_revision_ = nextEsdfRevision();
    revision_ = nextRevision();
}

auto GridMap::direct_set_map(const RowMatrixXi& map) -> void {
    setMap(map);
}

auto GridMap::inflate(double radius) -> void {
    const Eigen::Vector2i voxel_num = getVoxelNum();
    const double res = getResolution();
    if (!geometry_.isValid()) {
        return;
    }

    RowMatrixXi occ(voxel_num.x(), voxel_num.y());
    for (int x = 0; x < voxel_num.x(); ++x) {
        for (int y = 0; y < voxel_num.y(); ++y) {
            occ(x, y) = isOccupied({x, y}) ? 1 : 0;
        }
    }

    const RowMatrixXi inflated = inflateOccupancy(occ, radius, res);
    setMap(inflated);
}

auto GridMap::setMap(const RowMatrixXi& map) -> bool {
    if (!checkMapSize(map)) return false;
    bool changed = false;
    const int* src = map.data();
    for (int i = 0; i < buffer_size_; ++i) {
        const char new_val = src[i] != 0 ? static_cast<char>(1) : static_cast<char>(0);
        if (occ_buffer_[i] != new_val) {
            occ_buffer_[i] = new_val;
            changed = true;
        }
    }
    if (changed) {
        updateESDF();
        esdf_revision_ = nextEsdfRevision();
        revision_ = nextRevision();
    }
    return changed;
}

auto GridMap::overwriteMapAndUpdateEsdf(const RowMatrixXi& map, EsdfWorkspace* workspace) -> bool {
    if (!checkMapSize(map)) return false;
    const int* src = map.data();
    for (int i = 0; i < buffer_size_; ++i) {
        occ_buffer_[i] = src[i] != 0 ? static_cast<char>(1) : static_cast<char>(0);
    }
    updateESDF(workspace);
    esdf_revision_ = nextEsdfRevision();
    revision_ = nextRevision();
    return true;
}

auto GridMap::overwriteOccAndUpdateEsdf(std::span<const uint8_t> occ, EsdfWorkspace* workspace) -> bool {
    if (occ.size() != static_cast<size_t>(buffer_size_)) {
        logger::error(
            logger::GirdMap2D,
            "overwriteOccAndUpdateEsdf: size mismatch: got {}, expected {}",
            occ.size(),
            buffer_size_
        );
        return false;
    }
    for (int i = 0; i < buffer_size_; ++i) {
        occ_buffer_[i] = occ[static_cast<size_t>(i)] != 0 ? static_cast<char>(1) : static_cast<char>(0);
    }
    updateESDF(workspace);
    esdf_revision_ = nextEsdfRevision();
    revision_ = nextRevision();
    return true;
}

auto GridMap::copyOccAndEsdfFrom(const GridMap& src) -> bool {
    if (geometry_ != src.geometry_ || buffer_size_ != src.buffer_size_) {
        logger::error(logger::GirdMap2D, "copyOccAndEsdfFrom: map geometry mismatch");
        return false;
    }
    occ_buffer_ = src.occ_buffer_;
    esdf_buffer_ = src.esdf_buffer_;
    oob_policy_ = src.oob_policy_;
    esdf_revision_ = src.esdf_revision_;
    revision_ = nextRevision();
    return true;
}

auto GridMap::copySemanticsFrom(std::span<const uint8_t> semantics) -> bool {
    if (semantics.size() != semantics_buffer_.size()) {
        logger::error(
            logger::GirdMap2D,
            "copySemanticsFrom: size mismatch: got {}, expected {}",
            semantics.size(),
            semantics_buffer_.size()
        );
        return false;
    }
    std::copy(semantics.begin(), semantics.end(), semantics_buffer_.begin());
    revision_ = nextRevision();
    return true;
}

auto GridMap::equalsOccupancy(const RowMatrixXi& map) const -> bool {
    if (map.rows() != geometry_.voxel_num.x() || map.cols() != geometry_.voxel_num.y()) {
        return false;
    }
    const int* src = map.data();
    const char* dst = occ_buffer_.data();
    int i = 0;
    for (; i + 64 <= buffer_size_; i += 64) {
        uint8_t diff = 0;
        for (int k = 0; k < 64; ++k) {
            diff |= static_cast<uint8_t>((src[i + k] != 0) != (dst[i + k] != 0));
        }
        if (diff != 0) {
            return false;
        }
    }
    for (; i < buffer_size_; ++i) {
        if ((src[i] != 0) != (dst[i] != 0)) {
            return false;
        }
    }
    return true;
}

auto GridMap::equalsOccupancy(std::span<const uint8_t> occ) const -> bool {
    if (occ.size() != occ_buffer_.size()) {
        return false;
    }
    if (occ.empty()) {
        return true;
    }
    return std::memcmp(occ_buffer_.data(), occ.data(), occ.size()) == 0;
}

auto GridMap::computeBoundaryDistanceAndNormal(const Eigen::Vector2d& pos, double& dist, Eigen::Vector2d* inward_normal)
    const -> void {
    const double d_left = pos.x() - geometry_.min_boundary.x();
    const double d_right = geometry_.max_boundary.x() - pos.x();
    const double d_bot = pos.y() - geometry_.min_boundary.y();
    const double d_top = geometry_.max_boundary.y() - pos.y();
    dist = std::min({d_left, d_right, d_bot, d_top});
    if (!inward_normal) {
        return;
    }

    constexpr double kTieEps = 1e-9;
    Eigen::Vector2d n = Eigen::Vector2d::Zero();
    if (std::abs(d_left - dist) <= kTieEps) n.x() += 1.0;
    if (std::abs(d_right - dist) <= kTieEps) n.x() -= 1.0;
    if (std::abs(d_bot - dist) <= kTieEps) n.y() += 1.0;
    if (std::abs(d_top - dist) <= kTieEps) n.y() -= 1.0;
    const double norm = n.norm();
    if (norm > 1e-12) {
        *inward_normal = n / norm;
    } else {
        inward_normal->setZero();
    }
}

template<bool ComputeGradient>
auto GridMap::sampleEsdf(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d* gradient) const -> bool {
    if (!geometry_.isInside(pos)) return false;

    const double lx = (pos.x() - geometry_.origin.x()) * geometry_.resolution_inv;
    const double ly = (pos.y() - geometry_.origin.y()) * geometry_.resolution_inv;
    const int cx = static_cast<int>(std::lround(lx - 0.5));
    const int cy = static_cast<int>(std::lround(ly - 0.5));
    const int nx = geometry_.voxel_num.x();
    const int ny = geometry_.voxel_num.y();

    // 二次拟合以最近格心为 0；贴边时 3x3 邻域不完整，退回双线性（西南角 floor 索引，分数坐标落在 [0,1]）。
    if (cx < 1 || cy < 1 || cx + 1 >= nx || cy + 1 >= ny) {
        const Eigen::Vector2d pos_m = pos - 0.5 * geometry_.resolution * Eigen::Vector2d::Ones();
        Eigen::Vector2i idx;
        posToIndex(pos_m, idx);
        Eigen::Vector2d idx_pos;
        indexToPos(idx, idx_pos);
        const double fx = (pos.x() - idx_pos.x()) * geometry_.resolution_inv;
        const double fy = (pos.y() - idx_pos.y()) * geometry_.resolution_inv;

        double values[2][2];
        for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex(current_idx);
                values[x][y] = esdf_buffer_[toAddress(current_idx)];
            }
        }
        const double v0 = values[0][0] * (1.0 - fx) + values[1][0] * fx;
        const double v1 = values[0][1] * (1.0 - fx) + values[1][1] * fx;
        distance = v0 * (1.0 - fy) + v1 * fy;
        if constexpr (ComputeGradient) {
            (*gradient)(1) = (v1 - v0) * geometry_.resolution_inv;
            (*gradient)(0) = ((1.0 - fy) * (values[1][0] - values[0][0]) + fy * (values[1][1] - values[0][1]))
                * geometry_.resolution_inv;
        }
        return true;
    }

    const double tx = lx - 0.5 - static_cast<double>(cx);
    const double ty = ly - 0.5 - static_cast<double>(cy);

    const double wx0 = 0.5 * tx * (tx - 1.0);
    const double wx1 = 1.0 - tx * tx;
    const double wx2 = 0.5 * tx * (tx + 1.0);
    const double wy0 = 0.5 * ty * (ty - 1.0);
    const double wy1 = 1.0 - ty * ty;
    const double wy2 = 0.5 * ty * (ty + 1.0);

    const double* const r0 = esdf_buffer_.data() + (cx - 1) * ny + (cy - 1);
    const double* const r1 = r0 + ny;
    const double* const r2 = r1 + ny;

    const double s0 = wy0 * r0[0] + wy1 * r0[1] + wy2 * r0[2];
    const double s1 = wy0 * r1[0] + wy1 * r1[1] + wy2 * r1[2];
    const double s2 = wy0 * r2[0] + wy1 * r2[1] + wy2 * r2[2];
    distance = wx0 * s0 + wx1 * s1 + wx2 * s2;

    if constexpr (ComputeGradient) {
        const double dwx0 = tx - 0.5;
        const double dwx1 = -2.0 * tx;
        const double dwx2 = tx + 0.5;
        const double dwy0 = ty - 0.5;
        const double dwy1 = -2.0 * ty;
        const double dwy2 = ty + 0.5;

        const double ddx = dwx0 * s0 + dwx1 * s1 + dwx2 * s2;
        const double ds0 = dwy0 * r0[0] + dwy1 * r0[1] + dwy2 * r0[2];
        const double ds1 = dwy0 * r1[0] + dwy1 * r1[1] + dwy2 * r1[2];
        const double ds2 = dwy0 * r2[0] + dwy1 * r2[1] + dwy2 * r2[2];
        const double ddy = wx0 * ds0 + wx1 * ds1 + wx2 * ds2;
        gradient->x() = ddx * geometry_.resolution_inv;
        gradient->y() = ddy * geometry_.resolution_inv;
    }
    return true;
}

auto GridMap::getDistance(const Eigen::Vector2d& pos) const -> double {
    double distance = 0.0;
    if (sampleEsdf<false>(pos, distance, nullptr)) {
        if (oob_policy_ == OutOfBoundsPolicy::OCCUPIED) {
            double d_bound = 0.0;
            computeBoundaryDistanceAndNormal(pos, d_bound, nullptr);
            return std::min(distance, d_bound);
        }
        return distance;
    }

    if (!geometry_.isValid() || !pos.allFinite()) {
        return oob_policy_ == OutOfBoundsPolicy::OCCUPIED ? -std::numeric_limits<double>::infinity()
                                                          : std::numeric_limits<double>::max();
    }

    if (oob_policy_ == OutOfBoundsPolicy::FREE) {
        return std::numeric_limits<double>::max();
    }

    // OCCUPIED 越界：返回点到地图边界盒的负欧氏距离（上边界线上为 0.0）
    const Eigen::Vector2d clamped(
        std::clamp(pos.x(), geometry_.min_boundary.x(), geometry_.max_boundary.x()),
        std::clamp(pos.y(), geometry_.min_boundary.y(), geometry_.max_boundary.y())
    );
    return -(clamped - pos).norm();
}

auto GridMap::getDistanceAndGradient(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& gradient) const
    -> bool {
    if (sampleEsdf<true>(pos, distance, &gradient)) {
        if (oob_policy_ == OutOfBoundsPolicy::OCCUPIED) {
            double d_bound = 0.0;
            Eigen::Vector2d n_bound = Eigen::Vector2d::Zero();
            computeBoundaryDistanceAndNormal(pos, d_bound, &n_bound);
            if (d_bound < distance) {
                distance = d_bound;
                gradient = n_bound;
            }
        }
        return true;
    }

    if (!geometry_.isValid() || !pos.allFinite()) {
        distance = oob_policy_ == OutOfBoundsPolicy::OCCUPIED ? -std::numeric_limits<double>::infinity()
                                                              : std::numeric_limits<double>::max();
        gradient.setZero();
        return false;
    }

    if (oob_policy_ == OutOfBoundsPolicy::FREE) {
        distance = std::numeric_limits<double>::max();
        gradient.setZero();
        return false;
    }

    // OCCUPIED 越界：解析负距离 + 指向地图内部的单位梯度
    const Eigen::Vector2d clamped(
        std::clamp(pos.x(), geometry_.min_boundary.x(), geometry_.max_boundary.x()),
        std::clamp(pos.y(), geometry_.min_boundary.y(), geometry_.max_boundary.y())
    );
    const Eigen::Vector2d to_box = clamped - pos;
    const double dist_out = to_box.norm();
    if (dist_out > 1e-12) {
        distance = -dist_out;
        gradient = to_box / dist_out;
    } else {
        // 恰好落在左闭右开的上边界（x == max_x 或 y == max_y）：距离为 0，给定向内单位法向
        distance = 0.0;
        constexpr double kEdgeEps = 1e-9;
        Eigen::Vector2d n = Eigen::Vector2d::Zero();
        if (pos.x() >= geometry_.max_boundary.x() - kEdgeEps) n.x() -= 1.0;
        if (pos.x() <= geometry_.min_boundary.x() + kEdgeEps) n.x() += 1.0;
        if (pos.y() >= geometry_.max_boundary.y() - kEdgeEps) n.y() -= 1.0;
        if (pos.y() <= geometry_.min_boundary.y() + kEdgeEps) n.y() += 1.0;
        const double n_norm = n.norm();
        if (n_norm > 1e-12) {
            gradient = n / n_norm;
        } else {
            gradient.setZero();
        }
    }
    return true;
}

auto GridMap::isCollision(const Eigen::Vector2d& pos, double safe_threshold) const -> bool {
    double distance = 0.0;
    if (sampleEsdf<false>(pos, distance, nullptr)) {
        if (oob_policy_ == OutOfBoundsPolicy::OCCUPIED) {
            double d_bound = 0.0;
            computeBoundaryDistanceAndNormal(pos, d_bound, nullptr);
            distance = std::min(distance, d_bound);
        }
        // 占据/边界格 ESDF 恰为 0.0：safe_threshold <= 0 时 distance < threshold 会漏判，
        // 因此显式把非正 ESDF（占据或贴边）视为碰撞；孤立单格障碍的离中心点双线性 ESDF 略大于 0
        // （理论上界 <= 0.854 * resolution），在 distance < resolution 时补查格内原始占据位，
        // 既杜绝离中心点穿透单格障碍，又对远处自由格零额外开销。
        if (distance <= 0.0 || distance < safe_threshold) {
            return true;
        }
        if (distance < geometry_.resolution) {
            Eigen::Vector2i idx;
            posToIndex(pos, idx);
            if (isInMap(idx) && occ_buffer_[toAddress(idx)] != 0) {
                return true;
            }
        }
        return false;
    }
    if (!pos.allFinite()) {
        return true;
    }
    return oob_policy_ == OutOfBoundsPolicy::OCCUPIED;
}

auto GridMap::isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const -> bool {
    if (!geometry_.isValid()) {
        return oob_policy_ == OutOfBoundsPolicy::OCCUPIED;
    }
    if (!p1.allFinite() || !p2.allFinite()) {
        return true;
    }
    const Eigen::Vector2d diff = p2 - p1;
    const double max_dist = diff.norm();
    if (max_dist < 1e-6) return isCollision(p1);

    // 采样步长 <= 0.25 * resolution（远小于 0.5 * resolution），保证对角线不会跨过角点障碍。
    const double step = std::max(1e-6, geometry_.resolution * 0.25);
    const int n_steps = std::max(1, static_cast<int>(std::ceil(max_dist / step)));
    const Eigen::Vector2d dir = diff / max_dist;

    // 始终显式检查两个精确端点，避免采样点因浮点累加错过 p2。
    if (isCollision(p1)) return true;
    if (isCollision(p2)) return true;
    for (int i = 1; i < n_steps; ++i) {
        const Eigen::Vector2d pt = p1 + dir * (static_cast<double>(i) * step);
        if (isCollision(pt)) return true;
    }
    return false;
}

auto GridMap::getMap() const -> RowMatrixXd {
    RowMatrixXd map(geometry_.voxel_num.x(), geometry_.voxel_num.y());
    for (int x = 0; x < geometry_.voxel_num.x(); ++x) {
        for (int y = 0; y < geometry_.voxel_num.y(); ++y) {
            map(x, y) = esdf_buffer_[toAddress(x, y)];
        }
    }
    return map;
}

auto GridMap::inflateOccupancy(const RowMatrixXi& occ, double inflate_radius, double resolution) -> RowMatrixXi {
    const int rows = static_cast<int>(occ.rows());
    const int cols = static_cast<int>(occ.cols());

    // 膨胀半径对应的格子数
    const int r = static_cast<int>(std::ceil(inflate_radius / resolution));

    // 拷贝一份，避免影响原始 occupancy
    grid_map::RowMatrixXi inflated = occ;

    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            if (occ(x, y) != 1) continue;

            // 圆形膨胀
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    if (dx * dx + dy * dy > r * r) continue;

                    const int nx = x + dx;
                    const int ny = y + dy;

                    if (nx < 0 || nx >= rows || ny < 0 || ny >= cols) {
                        continue;
                    }

                    inflated(nx, ny) = 1;
                }
            }
        }
    }

    return inflated;
}

auto GridMap::updateESDF(EsdfWorkspace* workspace) -> void {
    const int rows = geometry_.voxel_num.x();
    const int cols = geometry_.voxel_num.y();
    if (rows <= 0 || cols <= 0) {
        return;
    }

    // 退化情形（全空闲或全占据）直接填充 ±地图对角线，避免 fillESDF 产生 ~6.7e152 极值。
    bool has_occ = false;
    bool has_free = false;
    for (int i = 0; i < buffer_size_; ++i) {
        if (occ_buffer_[i] != 0) {
            has_occ = true;
        } else {
            has_free = true;
        }
        if (has_occ && has_free) {
            break;
        }
    }
    if (!has_occ || !has_free) {
        const double fill_val = has_occ ? -geometry_.diagonal() : geometry_.diagonal();
        std::fill(esdf_buffer_.begin(), esdf_buffer_.end(), fill_val);
        return;
    }

    EsdfWorkspace local_ws;
    EsdfWorkspace& ws = workspace != nullptr ? *workspace : local_ws;

    if (ws.tmp_buffer.rows() != rows || ws.tmp_buffer.cols() != cols) {
        ws.tmp_buffer.resize(rows, cols);
        ws.neg_buffer.resize(rows, cols);
        ws.dist_buffer.resize(rows, cols);
    }
    const int max_dim = std::max(rows, cols);
    if (static_cast<int>(ws.v.size()) < max_dim) {
        ws.v.resize(static_cast<size_t>(max_dim));
    }
    if (static_cast<int>(ws.z.size()) < max_dim + 1) {
        ws.z.resize(static_cast<size_t>(max_dim + 1));
    }

    RowMatrixXd& tmp_buffer = ws.tmp_buffer;
    RowMatrixXd& neg_buffer = ws.neg_buffer;
    RowMatrixXd& dist_buffer = ws.dist_buffer;

    for (int x = 0; x < rows; ++x) {
        fillESDF(
            [&](int y) { return occ_buffer_[toAddress(x, y)] ? 0.0 : std::numeric_limits<double>::infinity(); },
            [&](int y, double val) { tmp_buffer(x, y) = val; },
            0,
            cols - 1,
            ws.v,
            ws.z
        );
    }

    for (int y = 0; y < cols; ++y) {
        fillESDF(
            [&](int x) { return tmp_buffer(x, y); },
            [&](int x, double val) { dist_buffer(x, y) = geometry_.resolution * std::sqrt(val); },
            0,
            rows - 1,
            ws.v,
            ws.z
        );
    }

    for (int x = 0; x < rows; ++x) {
        fillESDF(
            [&](int y) { return occ_buffer_[toAddress(x, y)] == 0 ? 0.0 : std::numeric_limits<double>::infinity(); },
            [&](int y, double val) { tmp_buffer(x, y) = val; },
            0,
            cols - 1,
            ws.v,
            ws.z
        );
    }

    for (int y = 0; y < cols; ++y) {
        fillESDF(
            [&](int x) { return tmp_buffer(x, y); },
            [&](int x, double val) { neg_buffer(x, y) = geometry_.resolution * std::sqrt(val); },
            0,
            rows - 1,
            ws.v,
            ws.z
        );
    }

    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            const double pos_dist = dist_buffer(x, y);
            const double neg_dist = std::abs(neg_buffer(x, y));

            if (neg_dist > 0.0) {
                esdf_buffer_[toAddress(x, y)] = -neg_dist + geometry_.resolution;
            } else {
                esdf_buffer_[toAddress(x, y)] = pos_dist;
            }
        }
    }
}

} // namespace grid_map
