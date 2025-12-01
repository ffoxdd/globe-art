#ifndef GLOBEART_SRC_GLOBE_VORONOI_CORE_VORONOI_SPHERE_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_CORE_VORONOI_SPHERE_HPP_

#include "../../cgal_types.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "circulator_iterator.hpp"
#include <cstddef>
#include <memory>
#include <ranges>
#include <unordered_map>

namespace globe {

struct CellEdgeInfo {
    size_t neighbor_index;
    SphericalArc arc;
};

class VoronoiSphere {
 public:
    explicit VoronoiSphere();

    VoronoiSphere(const VoronoiSphere&) = delete;
    VoronoiSphere& operator=(const VoronoiSphere&) = delete;
    VoronoiSphere(VoronoiSphere&&) = default;
    VoronoiSphere& operator=(VoronoiSphere&&) = default;

    void insert(Point3 point);
    std::size_t size() const;

    SphericalPolygon cell(size_t index) const;
    auto cells() const;

    Point3 site(size_t index) const;
    void update_site(size_t index, Point3 new_position);

    std::vector<CellEdgeInfo> cell_edges(size_t index) const;

 private:
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using SphericalKernel = CGAL::Exact_spherical_kernel_3;
    using Triangulation = CGAL::Delaunay_triangulation_on_sphere_2<
        CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel, SphericalKernel>
    >;
    using Arc = Triangulation::Arc_on_sphere_2;

    using VertexHandle = Triangulation::Vertex_handle;
    using EdgeCirculator = Triangulation::Edge_circulator;
    using Edge = Triangulation::Edge;
    using EdgeCirculatorIterator = CirculatorIterator<EdgeCirculator, Edge>;

    std::unique_ptr<Triangulation> _triangulation;
    std::vector<VertexHandle> _handles;
    std::unordered_map<VertexHandle, size_t> _handle_to_index;

    auto static edge_circulator_range(EdgeCirculator edge_circulator);
    auto incident_edges_range(VertexHandle vertex_handle) const;
    std::vector<SphericalArc> cell_arcs(size_t index) const;
    size_t vertex_index(VertexHandle handle) const;

    static SphericalArc to_spherical_arc(const Arc& cgal_arc);
    static Vector3 arc_normal(const Arc& arc);

    template<typename P>
    static Point3 to_point(const P& p);
};

inline VoronoiSphere::VoronoiSphere() :
    _triangulation(std::make_unique<Triangulation>()) {
}

inline void VoronoiSphere::insert(Point3 point) {
    VertexHandle handle = _triangulation->insert(point);
    size_t index = _handles.size();
    _handles.push_back(handle);
    _handle_to_index[handle] = index;
}

inline auto VoronoiSphere::edge_circulator_range(EdgeCirculator edge_circulator) {
    auto begin = EdgeCirculatorIterator(edge_circulator);
    auto end = EdgeCirculatorIterator(edge_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto VoronoiSphere::incident_edges_range(VertexHandle vertex_handle) const {
    return edge_circulator_range(_triangulation->incident_edges(vertex_handle));
}

inline std::size_t VoronoiSphere::size() const {
    return _triangulation->number_of_vertices();
}

inline Point3 VoronoiSphere::site(size_t index) const {
    return _triangulation->point(_handles[index]);
}

inline void VoronoiSphere::update_site(size_t index, Point3 new_position) {
    _handle_to_index.erase(_handles[index]);
    _triangulation->remove(_handles[index]);
    _handles[index] = _triangulation->insert(new_position);
    _handle_to_index[_handles[index]] = index;
}

inline std::vector<SphericalArc> VoronoiSphere::cell_arcs(size_t index) const {
    if (_triangulation->dimension() < 2) {
        return {};
    }

    VertexHandle vertex_handle = _handles[index];
    std::vector<SphericalArc> arcs;

    for (const auto& edge : incident_edges_range(vertex_handle)) {
        Arc cgal_arc = _triangulation->dual_on_sphere(edge);
        arcs.push_back(to_spherical_arc(cgal_arc));
    }

    return arcs;
}

inline SphericalPolygon VoronoiSphere::cell(size_t index) const {
    return SphericalPolygon(cell_arcs(index));
}

inline auto VoronoiSphere::cells() const {
    size_t count = (_triangulation->dimension() >= 2) ? size() : 0;
    return std::views::iota(size_t(0), count) | std::views::transform(
        [this](size_t index) {
            return cell(index);
        }
    );
}

inline size_t VoronoiSphere::vertex_index(VertexHandle handle) const {
    auto it = _handle_to_index.find(handle);
    if (it != _handle_to_index.end()) {
        return it->second;
    }
    return size();
}

inline std::vector<CellEdgeInfo> VoronoiSphere::cell_edges(size_t index) const {
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
            Arc cgal_arc = _triangulation->dual_on_sphere(edge);
            result.push_back({neighbor_idx, to_spherical_arc(cgal_arc)});
        }
    }

    return result;
}

inline SphericalArc VoronoiSphere::to_spherical_arc(const Arc& cgal_arc) {
    VectorS2 source = to_vector_s2(to_point(cgal_arc.source()));
    VectorS2 target = to_vector_s2(to_point(cgal_arc.target()));
    VectorS2 normal = to_vector_s2(arc_normal(cgal_arc));
    return SphericalArc(source, target, normal);
}

inline Vector3 VoronoiSphere::arc_normal(const Arc& arc) {
    auto circle = arc.supporting_circle();
    auto plane = circle.supporting_plane();
    auto normal = plane.orthogonal_vector();

    double x = CGAL::to_double(normal.x());
    double y = CGAL::to_double(normal.y());
    double z = CGAL::to_double(normal.z());

    double len = std::sqrt(x * x + y * y + z * z);
    if (len < 1e-15) {
        return Vector3(0, 0, 1);
    }

    return Vector3(x / len, y / len, z / len);
}

template<typename P>
inline Point3 VoronoiSphere::to_point(const P& p) {
    return Point3(
        CGAL::to_double(p.x()),
        CGAL::to_double(p.y()),
        CGAL::to_double(p.z())
    );
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_CORE_VORONOI_SPHERE_HPP_
