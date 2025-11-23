#ifndef GLOBEART_SRC_GLOBE_VORONOI_CORE_VORONOI_SPHERE_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_CORE_VORONOI_SPHERE_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/spherical_polygon.hpp"
#include "circulator_iterator.hpp"
#include <cstddef>
#include <ranges>

namespace globe {

class VoronoiSphere {
 public:
    explicit VoronoiSphere();

    void insert(Point3 point);
    std::size_t size() const;
    auto dual_arcs() const;
    auto dual_cells() const;

    Point3 site(size_t index) const;
    void update_site(size_t index, Point3 new_position);

 private:
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using SphericalKernel = CGAL::Exact_spherical_kernel_3;
    using Triangulation = CGAL::Delaunay_triangulation_on_sphere_2<
        CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel, SphericalKernel>
    >;

    using VertexHandle = Triangulation::Vertex_handle;
    using EdgeCirculator = Triangulation::Edge_circulator;
    using Edge = Triangulation::Edge;
    using EdgeCirculatorIterator = CirculatorIterator<EdgeCirculator, Edge>;

    Triangulation _triangulation;
    std::vector<VertexHandle> _handles;

    auto all_edges() const;
    auto static edge_circulator_range(EdgeCirculator edge_circulator);
    auto incident_edges_range(VertexHandle vertex_handle) const;
    std::vector<Arc> dual_cell_arcs(size_t index) const;
};

inline VoronoiSphere::VoronoiSphere() {
}

inline void VoronoiSphere::insert(Point3 point) {
    VertexHandle handle = _triangulation.insert(point);
    _handles.push_back(handle);
}

inline auto VoronoiSphere::edge_circulator_range(EdgeCirculator edge_circulator) {
    auto begin = EdgeCirculatorIterator(edge_circulator);
    auto end = EdgeCirculatorIterator(edge_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto VoronoiSphere::incident_edges_range(VertexHandle vertex_handle) const {
    return edge_circulator_range(_triangulation.incident_edges(vertex_handle));
}

inline std::size_t VoronoiSphere::size() const {
    return _triangulation.number_of_vertices();
}

inline auto VoronoiSphere::all_edges() const {
    return std::ranges::subrange(
        _triangulation.all_edges_begin(),
        _triangulation.all_edges_end()
    );
}

inline auto VoronoiSphere::dual_arcs() const {
    return all_edges() | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );
}

inline Point3 VoronoiSphere::site(size_t index) const {
    return _triangulation.point(_handles[index]);
}

inline void VoronoiSphere::update_site(size_t index, Point3 new_position) {
    _triangulation.remove(_handles[index]);
    _handles[index] = _triangulation.insert(new_position);
}

inline std::vector<Arc> VoronoiSphere::dual_cell_arcs(size_t index) const {
    VertexHandle vertex_handle = _handles[index];
    auto range = incident_edges_range(vertex_handle) | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );
    return {range.begin(), range.end()};
}

inline auto VoronoiSphere::dual_cells() const {
    return std::views::iota(size_t(0), size()) | std::views::transform(
        [this](size_t index) {
            return SphericalPolygon(dual_cell_arcs(index));
        }
    );
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_VORONOI_CORE_VORONOI_SPHERE_HPP_

