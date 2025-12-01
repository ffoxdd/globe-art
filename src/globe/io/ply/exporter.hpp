#ifndef GLOBEART_SRC_GLOBE_IO_PLY_EXPORTER_HPP_
#define GLOBEART_SRC_GLOBE_IO_PLY_EXPORTER_HPP_

#include "mesh/voronoi_sphere_builder.hpp"
#include "../../voronoi/core/voronoi_sphere.hpp"
#include <CGAL/IO/PLY.h>
#include <fstream>
#include <stdexcept>
#include <format>
#include <string>

namespace globe::io::ply {

class Exporter {
public:
    static void save_ply(
        const VoronoiSphere &voronoi_sphere,
        const std::string &filename,
        int samples_per_arc = 20,
        double arc_thickness = 0.001
    );

private:
    using SurfaceMesh = mesh::SurfaceMesh;
    using VoronoiSphereBuilder = mesh::VoronoiSphereBuilder;
};

inline void Exporter::save_ply(
    const VoronoiSphere &voronoi_sphere,
    const std::string &filename,
    int samples_per_arc,
    double arc_thickness
) {
    VoronoiSphereBuilder builder(samples_per_arc, arc_thickness);
    SurfaceMesh mesh = builder.build(voronoi_sphere);

    std::ofstream stream(filename);

    if (!stream) {
        throw std::runtime_error(std::format("Cannot open file for writing: {}", filename));
    }

    bool success = CGAL::IO::write_PLY(stream, mesh);

    if (!success) {
        throw std::runtime_error(std::format("Cannot write file: {}", filename));
    }
}

} // namespace globe::io::ply

#endif //GLOBEART_SRC_GLOBE_IO_PLY_EXPORTER_HPP_

