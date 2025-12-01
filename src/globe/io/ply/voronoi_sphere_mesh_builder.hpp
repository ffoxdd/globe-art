#ifndef GLOBEART_SRC_GLOBE_IO_PLY_VORONOI_SPHERE_MESH_BUILDER_HPP_
#define GLOBEART_SRC_GLOBE_IO_PLY_VORONOI_SPHERE_MESH_BUILDER_HPP_

#include "./types.hpp"
#include "../../voronoi/core/voronoi_sphere.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "mesh_builder.hpp"

namespace globe {

class VoronoiSphereMeshBuilder {
public:
    VoronoiSphereMeshBuilder(int samples_per_arc = 20, double arc_thickness = 0.001);
    SurfaceMesh build(const VoronoiSphere &voronoi_sphere);

private:
    MeshBuilder _mesh_builder;
};

inline VoronoiSphereMeshBuilder::VoronoiSphereMeshBuilder(
    int samples_per_arc,
    double arc_thickness
) :
    _mesh_builder(samples_per_arc, arc_thickness) {
}

inline SurfaceMesh VoronoiSphereMeshBuilder::build(const VoronoiSphere &voronoi_sphere) {
    SurfaceMesh mesh;

    for (const auto &cell : voronoi_sphere.cells()) {
        for (const auto &arc : cell.arcs()) {
            _mesh_builder.build_arc(
                mesh,
                cgal::to_point(arc.source()),
                cgal::to_point(arc.target())
            );
        }
    }

    return std::move(mesh);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_IO_PLY_VORONOI_SPHERE_MESH_BUILDER_HPP_

