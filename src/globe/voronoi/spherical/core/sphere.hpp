#ifndef GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_SPHERE_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_SPHERE_HPP_

#include "../../../cgal/types.hpp"
#include "../../../geometry/spherical/arc.hpp"
#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include "../../../geometry/spherical/polygon/polygon.hpp"
#include "circulator_iterator.hpp"
#include <cstddef>
#include <memory>
#include <ranges>
#include <unordered_map>

namespace globe::voronoi::spherical {

using geometry::spherical::Arc;
using geometry::spherical::polygon::Polygon;

struct CellEdgeInfo {
    size_t neighbor_index;
    Arc arc;
};

class Sphere {
 public:
    explicit Sphere();

    Sphere(const Sphere&) = delete;
    Sphere& operator=(const Sphere&) = delete;
    Sphere(Sphere&&) = default;
    Sphere& operator=(Sphere&&) = default;

    void insert(cgal::Point3 point);
    std::size_t size() const;

    Polygon cell(size_t index) const;
    auto cells() const;
    auto arcs() const;

    cgal::Point3 site(size_t index) const;
    void update_site(size_t index, cgal::Point3 new_position);

    std::vector<CellEdgeInfo> cell_edges(size_t index) const;

 private:
    using Kernel = ::CGAL::Exact_predicates_inexact_constructions_kernel;
    using SphericalKernel = ::CGAL::Exact_spherical_kernel_3;
    using Triangulation = ::CGAL::Delaunay_triangulation_on_sphere_2<
        ::CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel, SphericalKernel>
    >;
    using CGALArc = Triangulation::Arc_on_sphere_2;

    using VertexHandle = Triangulation::Vertex_handle;
    using EdgeCirculator = Triangulation::Edge_circulator;
    using Edge = Triangulation::Edge;
    using EdgeCirculatorIterator = CirculatorIterator<EdgeCirculator, Edge>;

    std::unique_ptr<Triangulation> _triangulation;
    std::vector<VertexHandle> _handles;
    std::unordered_map<VertexHandle, size_t> _handle_to_index;

    auto static edge_circulator_range(EdgeCirculator edge_circulator);
    auto incident_edges_range(VertexHandle vertex_handle) const;
    std::vector<Arc> cell_arcs(size_t index) const;
    size_t vertex_index(VertexHandle handle) const;

    static Arc to_spherical_arc(const CGALArc& cgal_arc);
    static VectorS2 arc_normal(const CGALArc& arc);

    template<typename P>
    static cgal::Point3 to_point(const P& p);
};

inline Sphere::Sphere() :
    _triangulation(std::make_unique<Triangulation>()) {
}

inline void Sphere::insert(cgal::Point3 point) {
    VertexHandle handle = _triangulation->insert(point);
    size_t index = _handles.size();
    _handles.push_back(handle);
    _handle_to_index[handle] = index;
}

inline auto Sphere::edge_circulator_range(EdgeCirculator edge_circulator) {
    auto begin = EdgeCirculatorIterator(edge_circulator);
    auto end = EdgeCirculatorIterator(edge_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto Sphere::incident_edges_range(VertexHandle vertex_handle) const {
    return edge_circulator_range(_triangulation->incident_edges(vertex_handle));
}

inline std::size_t Sphere::size() const {
    return _triangulation->number_of_vertices();
}

inline cgal::Point3 Sphere::site(size_t index) const {
    return _triangulation->point(_handles[index]);
}

inline void Sphere::update_site(size_t index, cgal::Point3 new_position) {
    _handle_to_index.erase(_handles[index]);
    _triangulation->remove(_handles[index]);
    _handles[index] = _triangulation->insert(new_position);
    _handle_to_index[_handles[index]] = index;
}

inline std::vector<Arc> Sphere::cell_arcs(size_t index) const {
    if (_triangulation->dimension() < 2) {
        return {};
    }

    VertexHandle vertex_handle = _handles[index];
    std::vector<Arc> arcs;

    for (const auto& edge : incident_edges_range(vertex_handle)) {
        CGALArc cgal_arc = _triangulation->dual_on_sphere(edge);
        arcs.push_back(to_spherical_arc(cgal_arc));
    }

    return arcs;
}

inline Polygon Sphere::cell(size_t index) const {
    return Polygon(cell_arcs(index));
}

inline auto Sphere::cells() const {
    size_t count = (_triangulation->dimension() >= 2) ? size() : 0;
    return std::views::iota(size_t(0), count) | std::views::transform(
        [this](size_t index) {
            return cell(index);
        }
    );
}

inline auto Sphere::arcs() const {
    return cells() | std::views::transform(
        [](const Polygon &cell) {
            return cell.arcs();
        }
    ) | std::views::join;
}

inline size_t Sphere::vertex_index(VertexHandle handle) const {
    auto it = _handle_to_index.find(handle);
    if (it != _handle_to_index.end()) {
        return it->second;
    }
    return size();
}

inline std::vector<CellEdgeInfo> Sphere::cell_edges(size_t index) const {
    std::vector<CellEdgeInfo> result;
    VertexHandle vertex_handle = _handles[index];

    for (const auto& edge : incident_edges_range(vertex_handle)) {
        auto face = edge.first;
        int edge_index = edge.second;

        VertexHandle v1 = face->vertex((edge_index + 1) % 3);
        VertexHandle v2 = face->vertex((edge_index + 2) % 3);

        VertexHandle neighbor_handle = (v1 == vertex_handle) ? v2 : v1;
        size_t neighbor_idx = vertex_index(neighbor_handle);

        if (neighbor_idx < size()) {
            CGALArc cgal_arc = _triangulation->dual_on_sphere(edge);
            result.push_back({neighbor_idx, to_spherical_arc(cgal_arc)});
        }
    }

    return result;
}

inline Arc Sphere::to_spherical_arc(const CGALArc& cgal_arc) {
    VectorS2 source = to_vector_s2(to_point(cgal_arc.source()));
    VectorS2 target = to_vector_s2(to_point(cgal_arc.target()));
    VectorS2 normal = arc_normal(cgal_arc);
    return Arc(source, target, normal);
}

inline VectorS2 Sphere::arc_normal(const CGALArc& arc) {
    auto circle = arc.supporting_circle();
    auto plane = circle.supporting_plane();
    auto normal = plane.orthogonal_vector();

    double x = ::CGAL::to_double(normal.x());
    double y = ::CGAL::to_double(normal.y());
    double z = ::CGAL::to_double(normal.z());

    double len = std::sqrt(x * x + y * y + z * z);
    if (len < 1e-15) {
        return VectorS2(0, 0, 1);
    }

    return VectorS2(x / len, y / len, z / len);
}

template<typename P>
inline cgal::Point3 Sphere::to_point(const P& p) {
    return cgal::Point3(
        ::CGAL::to_double(p.x()),
        ::CGAL::to_double(p.y()),
        ::CGAL::to_double(p.z())
    );
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_SPHERE_HPP_
