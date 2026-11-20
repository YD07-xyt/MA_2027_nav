#pragma once

#include "map/core/grid_geometry.hpp"
#include "map/core/grid_map.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace ma_map {

// 池化不可变 GridMap 快照对象池（按 GridGeometry 实例化）：
// - 默认预热 2 槽，上限 4 槽；读线程持有旧快照期间不阻塞写线程发布。
// - 当地图几何变更（动态扩图或加载新静态图）时，由 LayeredMap 整体替换为新池；
//   旧快照经 weak_ptr 回收器释放时检测到旧池已失效即自动析构旧 GridMap，零悬挂风险。
class SnapshotPool: public std::enable_shared_from_this<SnapshotPool> {
public:
    static constexpr size_t kPrewarmSlots = 2;
    static constexpr size_t kMaxSlots = 4;

    struct Slot {
        std::unique_ptr<grid_map::GridMap> map;
        uint64_t semantics_version = 0;
    };

    SnapshotPool(const grid_map::GridGeometry& geometry, grid_map::OutOfBoundsPolicy oob_policy):
        geometry_(geometry),
        oob_policy_(oob_policy) {
        free_slots_.reserve(kMaxSlots);
        for (size_t i = 0; i < kPrewarmSlots; ++i) {
            free_slots_.push_back(Slot {createGridMap(oob_policy), 0});
        }
    }

    auto geometry() const -> const grid_map::GridGeometry& {
        return geometry_;
    }

    auto setDefaultOutOfBoundsPolicy(grid_map::OutOfBoundsPolicy policy) -> void {
        std::lock_guard<std::mutex> lock(mtx_);
        oob_policy_ = policy;
    }

    auto acquire() -> Slot {
        grid_map::OutOfBoundsPolicy policy = grid_map::OutOfBoundsPolicy::FREE;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!free_slots_.empty()) {
                Slot slot = std::move(free_slots_.back());
                free_slots_.pop_back();
                return slot;
            }
            policy = oob_policy_;
        }
        return Slot {createGridMap(policy), 0};
    }

    auto release(Slot slot) -> void {
        if (!slot.map) {
            return;
        }
        std::unique_ptr<grid_map::GridMap> overflow;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (free_slots_.size() < kMaxSlots) {
                free_slots_.push_back(std::move(slot));
                return;
            }
            overflow = std::move(slot.map);
        }
    }

    auto makeSnapshot(Slot slot) -> grid_map::GridMap::ConstPtr {
        grid_map::GridMap* raw = slot.map.release();
        return grid_map::GridMap::ConstPtr(raw, RecycleDeleter {weak_from_this(), slot.semantics_version});
    }

private:
    struct RecycleDeleter {
        std::weak_ptr<SnapshotPool> pool;
        uint64_t semantics_version = 0;

        void operator()(grid_map::GridMap* raw) const {
            std::unique_ptr<grid_map::GridMap> ptr(raw);
            if (!ptr) {
                return;
            }
            if (auto p = pool.lock()) {
                p->release(Slot {std::move(ptr), semantics_version});
            }
        }
    };

    auto createGridMap(grid_map::OutOfBoundsPolicy policy) const -> std::unique_ptr<grid_map::GridMap> {
        auto map = std::make_unique<grid_map::GridMap>();
        map->init(geometry_, policy);
        return map;
    }

    grid_map::GridGeometry geometry_;
    grid_map::OutOfBoundsPolicy oob_policy_ = grid_map::OutOfBoundsPolicy::FREE;
    std::mutex mtx_;
    std::vector<Slot> free_slots_;
};

} // namespace ma_map
