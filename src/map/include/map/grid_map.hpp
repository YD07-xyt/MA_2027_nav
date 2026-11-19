/*
    MIT License

    Copyright (c) 2025 Senming Tan (senmingtan5@gmail.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#pragma once

#include "utils/logger.hpp"
#include <algorithm>
#include <cstdint>
#include <vector>
#include <Eigen/Eigen>
#include <memory>
#include <limits>
#include <iostream>
#include <span>

namespace grid_map {

using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class GridMap {
public:
    typedef std::shared_ptr<GridMap> Ptr;
    using ConstPtr = std::shared_ptr<const GridMap>;

    // ESDF 复用工作区：由单写者持有并在多次 updateESDF 间复用，避免每帧重复分配临时矩阵。
    struct EsdfWorkspace {
        RowMatrixXd tmp_buffer;
        RowMatrixXd neg_buffer;
        RowMatrixXd dist_buffer;
        std::vector<int> v;
        std::vector<double> z;
    };

    double getResolution() const {
        return resolution_;
    }
    Eigen::Vector2d getMapSize() const {
        return map_size_;
    }
    Eigen::Vector2d getOrigin() const {
        return map_origin_;
    }

    bool isInsideMap(const Eigen::Vector2d& pos) const {
        return (pos.array() >= min_boundary_.array()).all() && (pos.array() <= max_boundary_.array()).all();
    }

    // 按原点向四个方向的延伸量初始化地图边界：
    //   x ∈ [-x_left, +x_right]，y ∈ [-y_left, +y_right]
    auto init(double x_left, double x_right, double y_left, double y_right, double resolution) -> void;
    //给定总尺寸，地图以原点为中心（等价于左=右=size/2）
    void init(double map_size_x, double map_size_y, double resolution) {
        init(map_size_x / 2.0, map_size_x / 2.0, map_size_y / 2.0, map_size_y / 2.0, resolution);
    }
    auto direct_set_map(const RowMatrixXi& map) -> void;
    void set_semantics();
    auto inflate(double radius) -> void;

    // 按差异写占据并更新 ESDF；仅当尺寸合法且至少一格变化时重算 ESDF 并返回 true。
    auto setMap(const RowMatrixXi& map) -> bool;
    // 无条件覆写占据并重算 ESDF（供对象池复用槽位使用，槽位可能持有更早代际的数据）。
    // workspace 为可选的单写者复用缓存，为空时退回局部临时分配。
    auto overwriteMapAndUpdateEsdf(const RowMatrixXi& map, EsdfWorkspace* workspace = nullptr) -> bool;
    // 从几何一致的快照复用占据与 ESDF（保留源 esdf_revision_，不触发 ESDF 重算，仅递增 revision_）。
    auto copyOccAndEsdfFrom(const GridMap& src) -> bool;
    // 覆写语义层（不修改占据与 ESDF，不改变 esdf_revision_，仅递增 revision_）。
    auto copySemanticsFrom(std::span<const uint8_t> semantics) -> bool;
    // 快速比较外部占据矩阵是否与当前快照逐格一致（首个差异即提前退出）。
    auto equalsOccupancy(const RowMatrixXi& map) const -> bool;

    // 单调递增版本号：revision() 覆盖占据/ESDF/语义任意变化；
    // esdf_revision() 仅在 ESDF（或 init）变化时递增，供规划器占用缓存失效判定使用。
    auto revision() const -> uint64_t {
        return revision_;
    }

    auto esdf_revision() const -> uint64_t {
        return esdf_revision_;
    }

    // 只读连续数据视图：布局为 (x, y) 行主序，索引 = x * voxel_num_.y() + y。
    // 借用生命周期不得超过当前 GridMap 快照（或下一次 init() 重新分配）。
    auto occData() const -> std::span<const char> {
        return occ_buffer_;
    }

    auto esdfData() const -> std::span<const double> {
        return esdf_buffer_;
    }

    auto semanticsData() const -> std::span<const uint8_t> {
        return semantics_buffer_;
    }

    auto getDistance(const Eigen::Vector2d& pos) const -> double;
    auto getDistanceAndGradient(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& gradient) const -> bool;
    auto isCollision(const Eigen::Vector2d& pos, double safe_threshold = 0.0) const -> bool;
    // bool isCollision(const Eigen::Vector2d& pos, double safe_threshold = 0.0) const {
    //     return !isInMap(pos) || getDistance(pos) < safe_threshold;
    // }

    auto isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const -> bool;

    auto getMap() const -> RowMatrixXd;

    Eigen::Vector2i getVoxelNum() const {
        return voxel_num_;
    }

    bool isOccupied(const Eigen::Vector2i& id) const {
        return isInMap(id) ? occ_buffer_[toAddress(id)] == 1 : true;
    }

    void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id) const {
        id << static_cast<int>((pos.x() - map_origin_.x()) * resolution_inv_),
            static_cast<int>((pos.y() - map_origin_.y()) * resolution_inv_);
    }

    void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos) const {
        pos << (id.x() + 0.5) * resolution_ + map_origin_.x(), (id.y() + 0.5) * resolution_ + map_origin_.y();
    }

public:
    enum Semantics : uint8_t {
        NONE = 0,
        TUNNEL = 1,
    };
    // 设置某个栅格的标记
    auto set_semantics(const Eigen::Vector2i& idx, Semantics semantics) -> void {
        if (!isInMap(idx)) return;
        const int addr = toAddress(idx);
        const auto target = static_cast<uint8_t>(semantics);
        if (semantics_buffer_[addr] != target) {
            semantics_buffer_[addr] = target;
            revision_ = nextRevision();
        }
    }

    // 获取某个栅格的标记
    auto get_semantics(const Eigen::Vector2i& idx) const -> Semantics {
        if (!isInMap(idx)) return Semantics::NONE;
        return static_cast<Semantics>(semantics_buffer_[toAddress(idx)]);
    }

    //获取标记矩阵（用于可视化）
    auto get_semantics_map() const -> RowMatrixXi {
        RowMatrixXi semantics_map(voxel_num_.x(), voxel_num_.y());
        for (int x = 0; x < voxel_num_.x(); ++x)
            for (int y = 0; y < voxel_num_.y(); ++y)
                semantics_map(x, y) = semantics_buffer_[toAddress(x, y)];
        return semantics_map;
    }
    auto semantics_polygon_region(const std::vector<Eigen::Vector2d>& polygon, Semantics semantics) -> bool {
        // 遍历所有栅格，将多边形内的栅格标记设为 semantics；仅在至少一格变化时递增 revision_
        const auto target = static_cast<uint8_t>(semantics);
        bool changed = false;
        for (int x = 0; x < voxel_num_.x(); ++x) {
            for (int y = 0; y < voxel_num_.y(); ++y) {
                Eigen::Vector2d pos;
                indexToPos(Eigen::Vector2i(x, y), pos);
                if (point_in_polygon(pos, polygon)) {
                    const int addr = toAddress(x, y);
                    if (semantics_buffer_[addr] != target) {
                        semantics_buffer_[addr] = target;
                        changed = true;
                    }
                }
            }
        }
        if (changed) {
            revision_ = nextRevision();
        }
        return changed;
    }

    // 可选：清除所有标记
    auto clear_all_markers() -> void {
        const auto none = static_cast<uint8_t>(Semantics::NONE);
        bool changed = false;
        for (auto& cell: semantics_buffer_) {
            if (cell != none) {
                cell = none;
                changed = true;
            }
        }
        if (changed) {
            revision_ = nextRevision();
        }
    }

    auto is_tunnel(const Eigen::Vector2d& pos) const -> bool {
        if (!isInMap(pos)) return false;
        Eigen::Vector2i idx;
        posToIndex(pos, idx);
        return get_semantics(idx) == Semantics::TUNNEL;
    }

protected:
    inline int toAddress(int x, int y) const {
        return x * voxel_num_.y() + y;
    }

    // 保护 setMap/direct_set_map 不越界：输入矩阵行列必须与当前栅格一致，
    // 否则直接拒绝（尺寸不一致时按像素索引拷贝会导致错位或读越界内存）
    auto checkMapSize(const RowMatrixXi& map) const -> bool {
        if (map.rows() != voxel_num_.x() || map.cols() != voxel_num_.y()) {
            logger::error(
                logger::GirdMap2D,
                "size mismatch: got {}x{}, expected {}x{}, map update rejected",
                map.rows(),
                map.cols(),
                voxel_num_.x(),
                voxel_num_.y()
            );
            return false;
        }
        return true;
    }
    auto inflateOccupancy(const RowMatrixXi& occ, double inflate_radius, double resolution) -> RowMatrixXi;
    auto updateESDF(EsdfWorkspace* workspace = nullptr) -> void;
    auto sampleEsdf(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d* gradient) const -> bool;

private:
    static auto nextRevision() -> uint64_t;
    static auto nextEsdfRevision() -> uint64_t;

    template<typename F_get_val, typename F_set_val>
    static void fillESDF(
        F_get_val f_get_val,
        F_set_val f_set_val,
        int start,
        int end,
        std::vector<int>& v,
        std::vector<double>& z
    ) {
        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;
        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q)
                k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    inline int toAddress(const Eigen::Vector2i& id) const {
        return id.x() * voxel_num_.y() + id.y();
    }

    inline void boundIndex(Eigen::Vector2i& id) const {
        id(0) = std::max(std::min(id(0), voxel_num_.x() - 1), 0);
        id(1) = std::max(std::min(id(1), voxel_num_.y() - 1), 0);
    }

    inline bool isInMap(const Eigen::Vector2d& pos) const {
        return (pos.array() >= min_boundary_.array()).all() && (pos.array() <= max_boundary_.array()).all();
    }

    inline bool isInMap(const Eigen::Vector2i& idx) const {
        return idx.x() >= 0 && idx.y() >= 0 && idx.x() < voxel_num_.x() && idx.y() < voxel_num_.y();
    }

private:
    // 判断点是否在多边形内
    auto point_in_polygon(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& polygon) const -> bool {
        bool inside = false;
        int n = polygon.size();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            const Eigen::Vector2d& pi = polygon[i];
            const Eigen::Vector2d& pj = polygon[j];
            bool intersect = ((pi.y() > p.y()) != (pj.y() > p.y()))
                && (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x());
            if (intersect) inside = !inside;
        }
        return inside;
    }

private:
    double resolution_ = 0.0;
    double resolution_inv_ = 0.0;
    Eigen::Vector2d map_size_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_origin_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d min_boundary_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d max_boundary_ = Eigen::Vector2d::Zero();
    Eigen::Vector2i voxel_num_ = Eigen::Vector2i::Zero();
    int buffer_size_ = 0;
    uint64_t revision_ = 0;
    uint64_t esdf_revision_ = 0;
    std::vector<char> occ_buffer_;
    std::vector<double> esdf_buffer_;
    std::vector<uint8_t> semantics_buffer_; // 每个元素存储 Semantics 的整数值
};

} // namespace grid_map