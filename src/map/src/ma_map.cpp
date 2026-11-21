#include "map/ma_map.hpp"

#include "perception/local_perception_pipeline.hpp"

#include <utility>

namespace ma_map {

struct MaMap::Impl {
    explicit Impl(const std::string& cfg_path): cfg(cfg_path), perception(cfg), layered_map(makeLayeredConfig(cfg)) {}

    static auto makeLayeredConfig(const rog_map::Config& c) -> LayeredMap::Config {
        LayeredMap::Config lc;
        lc.x_left = c.grid_map_x_left;
        lc.x_right = c.grid_map_x_right;
        lc.y_left = c.grid_map_y_left;
        lc.y_right = c.grid_map_y_right;
        lc.resolution = c.grid_map_resolution;
        lc.expand_chunk_m = c.grid_map_expand_chunk_m;
        lc.max_map_size_x = c.grid_map_max_size_x;
        lc.max_map_size_y = c.grid_map_max_size_y;
        lc.save_margin_cells = c.grid_map_save_margin_cells;
        lc.unknown_as_occupied = c.grid_map_unknown_as_occupied;
        return lc;
    }

    rog_map::Config cfg;
    LocalPerceptionPipeline perception;
    LayeredMap layered_map;
};

MaMap::MaMap(const std::string& cfg_path): impl_(std::make_unique<Impl>(cfg_path)) {}

MaMap::~MaMap() = default;
MaMap::MaMap(MaMap&&) noexcept = default;
MaMap& MaMap::operator=(MaMap&&) noexcept = default;

auto MaMap::set_mode(MapMode mode) -> void {
    impl_->perception.setMode(mode);
    impl_->layered_map.setMode(mode);
}

auto MaMap::mode() const -> MapMode {
    return impl_->layered_map.mode();
}

auto MaMap::set_mapping_model(bool mapping_model) -> void {
    set_mode(mapping_model ? MapMode::MAPPING : MapMode::NAVIGATION);
}

auto MaMap::set_clock(utils::Clock clock) -> void {
    impl_->perception.setClock(clock);
}

auto MaMap::update_odom(const rog_map::Pose& pose, double stamp_sec) -> void {
    impl_->perception.updateOdom(pose, stamp_sec);
}

auto MaMap::update_cloud(const rog_map::PointCloud& cloud, double stamp_sec) -> void {
    impl_->perception.updateCloud(cloud, stamp_sec);
}

auto MaMap::update_map() -> void {
    const StaticNavContext static_ctx =
        (impl_->layered_map.mode() == MapMode::NAVIGATION && impl_->perception.usesStvlPrior())
        ? impl_->layered_map.getStaticNavContext()
        : StaticNavContext {};
    auto frame = impl_->perception.processPendingFrame(static_ctx);
    if (!frame.has_value()) {
        return;
    }
    impl_->layered_map.updateFromPerception(*frame);
}

auto MaMap::ensure_covers(const Eigen::Vector2d& world_pt) -> bool {
    return impl_->layered_map.ensureCovers(world_pt);
}

auto MaMap::load_static_map(std::string_view map_path) -> tl::expected<LoadReport, std::string> {
    auto res = impl_->layered_map.loadStaticMap(map_path);
    if (res.has_value()) {
        impl_->perception.resetDynamic();
    }
    return res;
}

auto MaMap::save_explored_map(std::string_view output_path, const std::optional<SaveOptions>& options) const
    -> tl::expected<SaveReport, std::string> {
    if (impl_->layered_map.mode() == MapMode::MAPPING && impl_->perception.usesStvlPrior()
        && impl_->perception.hasOnlineMappingData())
    {
        auto built = impl_->perception.buildGlobalStaticMap();
        if (built.has_value() && built->geometry.isValid()) {
            const SaveOptions effective_opts = options.value_or(impl_->layered_map.defaultSaveOptions());
            return MapIO::save(output_path, built->geometry, built->cells, effective_opts, &built->terrain_prior);
        }
    }
    return impl_->layered_map.saveExploredMap(output_path, options);
}

auto MaMap::set_global_map(grid_map::RowMatrixXi global_map) -> void {
    impl_->layered_map.setGlobalOccupancy(global_map);
}

auto MaMap::set_semantics_polygon(const std::vector<Eigen::Vector2d>& polygon, grid_map::GridMap::Semantics semantics)
    -> void {
    impl_->layered_map.setSemanticsPolygon(polygon, semantics);
}

auto MaMap::get_grid_map() const -> grid_map::GridMap::ConstPtr {
    return impl_->layered_map.getSnapshot();
}

auto MaMap::get_occupancy_2d() const -> grid_map::RowMatrixXi {
    return impl_->layered_map.getOccupancyMatrix();
}

auto MaMap::get_global_occ() const -> grid_map::RowMatrixXi {
    return impl_->layered_map.getStaticOccupancyMatrix();
}

auto MaMap::get_terrain_map() const -> utils::vec_E<utils::Vec3f> {
    return impl_->perception.getLastTerrainMap();
}

auto MaMap::get_ground_layer() const -> const Terrain::GroundLayer& {
    return impl_->perception.getGroundLayer();
}

auto MaMap::get_rog_map() const -> std::shared_ptr<rog_map::ROGMap> {
    return impl_->perception.getRogMap();
}

auto MaMap::get_config() const -> rog_map::Config {
    return impl_->perception.getConfig();
}

} // namespace ma_map
