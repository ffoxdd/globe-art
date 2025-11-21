#ifndef GLOBEART_SRC_GLOBE_VORONOI_CORE_VORONOI_SPHERE_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_CORE_VORONOI_SPHERE_HPP_

#include "../../types.hpp"
#include "../../spherical/spherical_polygon.hpp"
#include "types.hpp"
#include "handle_iterator.hpp"
#include "handle_circulator_iterator.hpp"
#include "circulator_iterator.hpp"
#include <cstddef>
#include <iterator>
#include <ranges>

namespace globe {

// TODO: consider what should go here and what should go in ./types.hpp
using VertexHandleValue = typename std::iterator_traits<FiniteVerticesIterator>::value_type::Vertex_handle;
using VertexHandleIterator = HandleIterator<FiniteVerticesIterator, VertexHandleValue>;
using FaceHandleValue = typename std::iterator_traits<FiniteFacesIterator>::value_type::Face_handle;
using FaceHandleIterator = HandleIterator<FiniteFacesIterator, FaceHandleValue>;
using VertexCirculatorIterator = CirculatorIterator<VertexCirculator, VertexHandle>;
using EdgeCirculatorIterator = CirculatorIterator<EdgeCirculator, Edge>;
using FaceHandleCirculatorValue = typename std::iterator_traits<FaceCirculator>::value_type::Face_handle;
using FaceHandleCirculatorIterator = HandleCirculatorIterator<FaceCirculator, FaceHandleCirculatorValue>;

class VoronoiSphere {
 public:
    explicit VoronoiSphere();

    void insert(Point3 point);
    template<Point3Range PR>
    void reset(PR new_points);
    VertexHandle move_vertex(VertexHandle vertex_handle, Point3 new_point);

    auto vertices() const;
    std::size_t size() const;
    auto points() const;
    auto dual_arcs() const;
    auto faces() const;
    auto dual_face_neighborhood(VertexHandle vertex_handle);
    std::vector<Arc> dual_cell_arcs(VertexHandle vertex_handle);
    auto dual_cells() const;

    Point3 site(size_t index) const;
    std::vector<Arc> dual_cell_arcs(size_t index) const;
    void update_site(size_t index, Point3 new_position);

 private:
    Triangulation _triangulation;
    std::vector<Point3> _sites;
    std::vector<VertexHandle> _handles;

    auto all_edges() const;
    auto static vertex_circulator_range(VertexCirculator vertex_circulator);
    auto static edge_circulator_range(EdgeCirculator edge_circulator);
    auto static face_circulator_range(FaceCirculator face_circulator);
    auto incident_vertices_range(VertexHandleValue vertex_handle) const;
    auto incident_edges_range(VertexHandleValue vertex_handle) const;
    void clear();
};

inline VoronoiSphere::VoronoiSphere() {
}

inline void VoronoiSphere::insert(Point3 point) {
    VertexHandle handle = _triangulation.insert(point);
    _sites.push_back(point);
    _handles.push_back(handle);
}

template<Point3Range PR>
inline void VoronoiSphere::reset(PR new_points) {
    clear();

    for (auto point : new_points) {
        VertexHandle handle = _triangulation.insert(point);
        _sites.push_back(point);
        _handles.push_back(handle);
    }
}

inline VertexHandle VoronoiSphere::move_vertex(VertexHandle vertex_handle, Point3 new_point) {
    _triangulation.remove(vertex_handle);
    return _triangulation.insert(new_point);
}

inline auto VoronoiSphere::vertex_circulator_range(VertexCirculator vertex_circulator) {
    auto begin = VertexCirculatorIterator(vertex_circulator);
    auto end = VertexCirculatorIterator(vertex_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto VoronoiSphere::edge_circulator_range(EdgeCirculator edge_circulator) {
    auto begin = EdgeCirculatorIterator(edge_circulator);
    auto end = EdgeCirculatorIterator(edge_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto VoronoiSphere::face_circulator_range(FaceCirculator face_circulator) {
    auto begin = FaceHandleCirculatorIterator(face_circulator);
    auto end = FaceHandleCirculatorIterator(face_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto VoronoiSphere::incident_vertices_range(VertexHandle vertex_handle) const {
    return vertex_circulator_range(_triangulation.incident_vertices(vertex_handle));
}

inline auto VoronoiSphere::incident_edges_range(VertexHandle vertex_handle) const {
    return edge_circulator_range(_triangulation.incident_edges(vertex_handle));
}

inline auto VoronoiSphere::vertices() const {
    return std::ranges::subrange(
        VertexHandleIterator(_triangulation.finite_vertices_begin()),
        VertexHandleIterator(_triangulation.finite_vertices_end())
    );
}

inline std::size_t VoronoiSphere::size() const {
    return _triangulation.number_of_vertices();
}

inline auto VoronoiSphere::points() const {
    return vertices() | std::views::transform(
        [](VertexHandleValue vertex_handle) {
            return vertex_handle->point();
        }
    );
}

inline auto VoronoiSphere::all_edges() const {
    return std::ranges::subrange(
        _triangulation.all_edges_begin(),
        _triangulation.all_edges_end()
    );
}

inline auto VoronoiSphere::faces() const {
    return std::ranges::subrange(
        FaceHandleIterator(_triangulation.finite_faces_begin()),
        FaceHandleIterator(_triangulation.finite_faces_end())
    );
}

inline auto VoronoiSphere::dual_face_neighborhood(VertexHandle vertex_handle) {
    return incident_vertices_range(vertex_handle) | std::views::transform(
        [this](VertexHandleValue vertex_handle) {
            return dual_cell_arcs(vertex_handle);
        }
    );
}

inline void VoronoiSphere::clear() {
    _triangulation.clear();
    _sites.clear();
    _handles.clear();
}

inline auto VoronoiSphere::dual_arcs() const {
    return all_edges() | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );
}

inline std::vector<Arc> VoronoiSphere::dual_cell_arcs(VertexHandle vertex_handle) {
    auto range = incident_edges_range(vertex_handle) | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );

    return {range.begin(), range.end()};
}

inline Point3 VoronoiSphere::site(size_t index) const {
    return _sites[index];
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

inline void VoronoiSphere::update_site(size_t index, Point3 new_position) {
    _triangulation.remove(_handles[index]);
    _handles[index] = _triangulation.insert(new_position);
    _sites[index] = new_position;
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

