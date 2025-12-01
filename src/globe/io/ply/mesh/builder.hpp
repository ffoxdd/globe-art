#ifndef GLOBEART_SRC_GLOBE_IO_PLY_MESH_BUILDER_HPP_
#define GLOBEART_SRC_GLOBE_IO_PLY_MESH_BUILDER_HPP_

#include "types.hpp"
#include "../../../cgal_types.hpp"
#include "../../../types.hpp"
#include "../../../geometry/spherical/arc.hpp"
#include <Eigen/Geometry>
#include <map>

namespace globe::io::ply::mesh {

class Builder {
public:
    Builder(int samples_per_arc = 20, double arc_thickness = 0.001);

    void add_arc(const Arc &arc);
    SurfaceMesh build();

private:

    void sample_arc_and_add_segments(
        const Arc &arc,
        VertexIndex &prev_vertex,
        VertexIndex &prev_offset_vertex
    );

    void add_segment_at_parameter(
        const Arc &arc,
        double t,
        VertexIndex &prev_vertex,
        VertexIndex &prev_offset_vertex
    );

    void add_ribbon_segment(
        VertexIndex prev_vertex,
        VertexIndex curr_vertex,
        VertexIndex prev_offset_vertex,
        VertexIndex curr_offset_vertex
    );

    VertexIndex get_or_create_vertex(const VectorS2 &point);

    VectorS2 create_offset_point(const VectorS2 &point, const VectorS2 &arc_normal) const;

    int _samples_per_arc;
    double _arc_thickness;
    SurfaceMesh _mesh;
    std::map<cgal::Point3, VertexIndex> _vertices_by_point;
};

inline Builder::Builder(int samples_per_arc, double arc_thickness) :
    _samples_per_arc(samples_per_arc),
    _arc_thickness(arc_thickness) {
}

inline void Builder::add_arc(const Arc &arc) {
    VectorS2 prev_offset = create_offset_point(arc.source(), arc.normal());

    VertexIndex prev_vertex = get_or_create_vertex(arc.source());
    VertexIndex prev_offset_vertex = get_or_create_vertex(prev_offset);

    sample_arc_and_add_segments(arc, prev_vertex, prev_offset_vertex);
}

inline SurfaceMesh Builder::build() {
    SurfaceMesh result = std::move(_mesh);
    _mesh = SurfaceMesh();
    _vertices_by_point.clear();
    return result;
}

inline void Builder::sample_arc_and_add_segments(
    const Arc &arc,
    VertexIndex &prev_vertex,
    VertexIndex &prev_offset_vertex
) {
    for (int i = 1; i <= _samples_per_arc; ++i) {
        double t = static_cast<double>(i) / _samples_per_arc;
        add_segment_at_parameter(arc, t, prev_vertex, prev_offset_vertex);
    }
}

inline void Builder::add_segment_at_parameter(
    const Arc &arc,
    double t,
    VertexIndex &prev_vertex,
    VertexIndex &prev_offset_vertex
) {
    VectorS2 sampled_point = arc.interpolate(t);
    VectorS2 offset_point = create_offset_point(sampled_point, arc.normal());

    VertexIndex curr_vertex = get_or_create_vertex(sampled_point);
    VertexIndex curr_offset_vertex = get_or_create_vertex(offset_point);

    add_ribbon_segment(prev_vertex, curr_vertex, prev_offset_vertex, curr_offset_vertex);

    prev_vertex = curr_vertex;
    prev_offset_vertex = curr_offset_vertex;
}

inline void Builder::add_ribbon_segment(
    VertexIndex prev_vertex,
    VertexIndex curr_vertex,
    VertexIndex prev_offset_vertex,
    VertexIndex curr_offset_vertex
) {
    _mesh.add_face(prev_vertex, curr_vertex, curr_offset_vertex);
    _mesh.add_face(prev_vertex, curr_offset_vertex, prev_offset_vertex);
}

inline VertexIndex Builder::get_or_create_vertex(const VectorS2 &point) {
    cgal::Point3 cgal_point = cgal::to_point(point);
    auto [it, inserted] = _vertices_by_point.try_emplace(cgal_point);
    auto& [_, vertex_index] = *it;

    if (inserted) {
        vertex_index = _mesh.add_vertex(cgal_point);
    }

    return vertex_index;
}

inline VectorS2 Builder::create_offset_point(const VectorS2 &point, const VectorS2 &arc_normal) const {
    VectorS2 perpendicular = arc_normal.cross(point).normalized();
    VectorS2 offset = point + (perpendicular * _arc_thickness);

    return offset.normalized();
}

} // namespace globe::io::ply::mesh

#endif //GLOBEART_SRC_GLOBE_IO_PLY_MESH_BUILDER_HPP_
