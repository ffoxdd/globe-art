#ifndef GLOBEART_SRC_GLOBE_IO_PLY_MESH_BUILDER_HPP_
#define GLOBEART_SRC_GLOBE_IO_PLY_MESH_BUILDER_HPP_

#include "./types.hpp"
#include "../cgal_helpers.hpp"
#include "../../cgal_types.hpp"
#include <map>

namespace globe {

class MeshBuilder {
public:
    MeshBuilder(int samples_per_arc = 20, double arc_thickness = 0.001);

    void build_arc(
        SurfaceMesh &mesh,
        const Point3 &source,
        const Point3 &target
    );

private:
    void sample_arc_and_add_segments(
        SurfaceMesh &mesh,
        const Point3 &source,
        const Point3 &target,
        const Vector3 &arc_normal,
        SurfaceMesh::Vertex_index &prev_vertex,
        SurfaceMesh::Vertex_index &prev_offset_vertex
    );

    void add_segment_at_parameter(
        SurfaceMesh &mesh,
        const Point3 &source,
        const Point3 &target,
        const Vector3 &arc_normal,
        double t,
        SurfaceMesh::Vertex_index &prev_vertex,
        SurfaceMesh::Vertex_index &prev_offset_vertex
    );

    void add_ribbon_segment(
        SurfaceMesh &mesh,
        SurfaceMesh::Vertex_index prev_vertex,
        SurfaceMesh::Vertex_index curr_vertex,
        SurfaceMesh::Vertex_index prev_offset_vertex,
        SurfaceMesh::Vertex_index curr_offset_vertex
    ) const;

    SurfaceMesh::Vertex_index get_or_create_vertex(
        SurfaceMesh &mesh,
        const Point3 &point
    );

    Point3 create_offset_point(const Point3 &point, const Vector3 &arc_normal) const;

    int _samples_per_arc;
    double _arc_thickness;
    std::map<Point3, SurfaceMesh::Vertex_index> _vertices_by_point;
};

inline MeshBuilder::MeshBuilder(int samples_per_arc, double arc_thickness) :
    _samples_per_arc(samples_per_arc),
    _arc_thickness(arc_thickness) {
}

inline void MeshBuilder::build_arc(
    SurfaceMesh &mesh,
    const Point3 &source,
    const Point3 &target
) {
    if (mesh.number_of_vertices() == 0) {
        _vertices_by_point.clear();
    }

    SurfaceMesh::Vertex_index prev_vertex = get_or_create_vertex(mesh, source);
    Vector3 arc_normal = io::normal(source, target);
    Point3 prev_offset = create_offset_point(source, arc_normal);
    SurfaceMesh::Vertex_index prev_offset_vertex = get_or_create_vertex(mesh, prev_offset);

    sample_arc_and_add_segments(mesh, source, target, arc_normal, prev_vertex, prev_offset_vertex);
}

inline void MeshBuilder::sample_arc_and_add_segments(
    SurfaceMesh &mesh,
    const Point3 &source,
    const Point3 &target,
    const Vector3 &arc_normal,
    SurfaceMesh::Vertex_index &prev_vertex,
    SurfaceMesh::Vertex_index &prev_offset_vertex
) {
    for (int i = 1; i <= _samples_per_arc; ++i) {
        double t = static_cast<double>(i) / _samples_per_arc;
        add_segment_at_parameter(mesh, source, target, arc_normal, t, prev_vertex, prev_offset_vertex);
    }
}

inline void MeshBuilder::add_segment_at_parameter(
    SurfaceMesh &mesh,
    const Point3 &source,
    const Point3 &target,
    const Vector3 &arc_normal,
    double t,
    SurfaceMesh::Vertex_index &prev_vertex,
    SurfaceMesh::Vertex_index &prev_offset_vertex
) {
    Point3 sampled_point = io::interpolate(source, target, t);
    SurfaceMesh::Vertex_index curr_vertex = get_or_create_vertex(mesh, sampled_point);
    Point3 offset_point = create_offset_point(sampled_point, arc_normal);
    SurfaceMesh::Vertex_index curr_offset_vertex = get_or_create_vertex(mesh, offset_point);

    add_ribbon_segment(mesh, prev_vertex, curr_vertex, prev_offset_vertex, curr_offset_vertex);

    prev_vertex = curr_vertex;
    prev_offset_vertex = curr_offset_vertex;
}

inline void MeshBuilder::add_ribbon_segment(
    SurfaceMesh &mesh,
    SurfaceMesh::Vertex_index prev_vertex,
    SurfaceMesh::Vertex_index curr_vertex,
    SurfaceMesh::Vertex_index prev_offset_vertex,
    SurfaceMesh::Vertex_index curr_offset_vertex
) const {
    mesh.add_face(prev_vertex, curr_vertex, curr_offset_vertex);
    mesh.add_face(prev_vertex, curr_offset_vertex, prev_offset_vertex);
}

inline SurfaceMesh::Vertex_index MeshBuilder::get_or_create_vertex(
    SurfaceMesh &mesh,
    const Point3 &point
) {
    auto it = _vertices_by_point.find(point);

    if (it != _vertices_by_point.end()) {
        return it->second;
    }

    SurfaceMesh::Vertex_index vertex = mesh.add_vertex(point);
    _vertices_by_point[point] = vertex;

    return vertex;
}

inline Point3 MeshBuilder::create_offset_point(const Point3 &point, const Vector3 &arc_normal) const {
    Vector3 radius_vec = to_cgal_vector(point);
    Vector3 perpendicular = io::normalize(CGAL::cross_product(arc_normal, radius_vec));
    Vector3 offset_vec = radius_vec + (perpendicular * _arc_thickness);
    Vector3 normalized = io::normalize(offset_vec);

    return Point3(normalized.x(), normalized.y(), normalized.z());
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_IO_PLY_MESH_BUILDER_HPP_

