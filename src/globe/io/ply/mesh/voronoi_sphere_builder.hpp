#ifndef GLOBEART_SRC_GLOBE_IO_PLY_MESH_VORONOI_SPHERE_BUILDER_HPP_
#define GLOBEART_SRC_GLOBE_IO_PLY_MESH_VORONOI_SPHERE_BUILDER_HPP_

#include "types.hpp"
#include "builder.hpp"
#include "../../../voronoi/spherical/core/sphere.hpp"

namespace globe::io::ply::mesh {

using voronoi::spherical::Sphere;

class VoronoiSphereBuilder {
public:
    VoronoiSphereBuilder(int samples_per_arc = 20, double arc_thickness = 0.001);
    SurfaceMesh build(const Sphere &sphere);

private:
    int _samples_per_arc;
    double _arc_thickness;
};

inline VoronoiSphereBuilder::VoronoiSphereBuilder(
    int samples_per_arc,
    double arc_thickness
) :
    _samples_per_arc(samples_per_arc),
    _arc_thickness(arc_thickness) {
}

inline SurfaceMesh VoronoiSphereBuilder::build(const Sphere &sphere) {
    Builder builder(_samples_per_arc, _arc_thickness);

    for (const auto &arc : sphere.arcs()) {
        builder.add_arc(arc);
    }

    return builder.build();
}

} // namespace globe::io::ply::mesh

#endif //GLOBEART_SRC_GLOBE_IO_PLY_MESH_VORONOI_SPHERE_BUILDER_HPP_
