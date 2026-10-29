/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <queue>
#include "inf_map.h"
#include "free_cnt_map.h"
#include "esdf_map.h"
#include "raycaster.h"

namespace rog_map {
using utils::Pose;

class ProbMap: public SlidingMap {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ProbMap> Ptr;

    ProbMap() = default;

    ~ProbMap() override = default;

    void initProbMap();

    void setConfig(const rog_map::Config& cfg) {
        cfg_ = cfg;
    }

    bool isOccupied(const utils::Vec3f& pos) const;

    bool isUnknown(const utils::Vec3f& pos) const;

    bool isKnownFree(const utils::Vec3f& pos) const;

    bool isOccupiedInflate(const utils::Vec3f& pos) const;

    bool isUnknownInflate(const utils::Vec3f& pos) const;

    bool isKnownFreeInflate(const utils::Vec3f& pos) const;

    bool isFrontier(const utils::Vec3f& pos) const;

    bool isFrontier(const utils::Vec3i& id_g) const;

    // Query result
    GridType getGridType(utils::Vec3i& id_g) const;

    GridType getGridType(const utils::Vec3f& pos) const;

    GridType getInfGridType(const utils::Vec3f& pos) const;

    double getMapValue(const utils::Vec3f& pos) const;

    void boxSearch(
        const utils::Vec3f& _box_min,
        const utils::Vec3f& _box_max,
        const GridType& gt,
        utils::vec_E<utils::Vec3f>& out_points
    ) const;

    void boxSearchInflate(
        const utils::Vec3f& box_min,
        const utils::Vec3f& box_max,
        const GridType& gt,
        utils::vec_E<utils::Vec3f>& out_points
    ) const;

    void boundBoxByLocalMap(utils::Vec3f& box_min, utils::Vec3f& box_max) const;

    utils::Vec3f getLocalMapOrigin() const;

    utils::Vec3f getLocalMapSize() const;

    double getResolution() const {
        return sc_.resolution;
    }

    double getInfResolution() const {
        return inf_map_->getResolution();
    }

    void updateOccPointCloud(const PointCloud& input_cloud);

    void writeTimeConsumingToLog(std::ofstream& log_file);

    void writeMapInfoToLog(std::ofstream& log_file);

    void updateProbMap(const PointCloud& cloud, const Pose& pose);

    // 运行时切换遗忘阈值（秒，≤0 禁用）。导航模式开、建图模式关，由 MaMap 按模式设置。
    void setForgetThreshold(double threshold) {
        forget_threshold_ = threshold;
    }

public:
    auto get_esdf_map() -> std::shared_ptr<ESDFMap> {
        return esdf_map_;
    }

protected:
    rog_map::Config cfg_;
    InfMap::Ptr inf_map_;
    FreeCntMap::Ptr fcnt_map_;
    ESDFMap::Ptr esdf_map_;
    /// Spherical neighborhood lookup table
    std::vector<float> occupancy_buffer_;
    std::vector<double> occ_timestamp_buffer_; // 记录每个体素变为占据的时间戳
    double forget_threshold_; // 遗忘时间阈值（秒）

    bool map_empty_ {true};
    struct RaycastData {
        raycaster::RayCaster raycaster;
        std::queue<utils::Vec3i> update_cache_id_g;
        std::vector<uint16_t> operation_cnt;
        std::vector<uint16_t> hit_cnt;
        utils::Vec3f cache_box_max, cache_box_min, local_update_box_max, local_update_box_min;
        int batch_update_counter {0};
        std::mutex raycast_range_mtx;
    } raycast_data_;

public:
    auto get_raycast_data() -> RaycastData& {
        return raycast_data_;
    }

protected:
    std::vector<double> time_consuming_;
    std::vector<std::string> time_consuming_name_ {
        "Total",
        "Raycast",
        "Update_cache",
        "Inflation",
        "PointCloudNumber",
        "CacheNumber",
        "InflationNumber"};

    // standardization query
    // Known free < l_free
    // occupied >= l_occ
    bool isKnownFree(const double& prob) const {
        return prob < cfg_.l_free;
    }

    bool isOccupied(const double& prob) const {
        return prob >= cfg_.l_occ;
    }

    bool isUnknown(const double& prob) const {
        return prob >= cfg_.l_free && prob < cfg_.l_occ;
    }

    void slideAllMap(const utils::Vec3f& pos);

    // warning using this function will cause memory leak if the id_g is not in the map
    bool isOccupied(const utils::Vec3i& id_g) const;

    bool isUnknown(const utils::Vec3i& id_g) const;

    bool isKnownFree(const utils::Vec3i& id_g) const;

    //====================================================================
    void resetCell(const int& hash_id) override;

    void probabilisticMapFromCache();

    void hitPointUpdate(const utils::Vec3f& pos, const int& hash_id, const int& hit_num);

    void missPointUpdate(const utils::Vec3f& pos, const int& hash_id, const int& hit_num);

    void raycastProcess(const PointCloud& input_cloud, const utils::Vec3f& cur_odom);

    void insertUpdateCandidate(const utils::Vec3i& id_g, bool is_hit);

    void updateLocalBox(const utils::Vec3f& cur_odom);

    void resetLocalMap() override;

    void applyForgetting(const utils::Vec3f& robot_pos);
};
}