#include "globe_generator.h"
#include <iostream>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/point_generators_3.h>

namespace globe {

GlobeGenerator &GlobeGenerator::generate() {
    _mesh = generate_globe_sphere();

    for(int i = 0; i < 100; i++) {
        add_random_point();
    }

    return *this;
}

void GlobeGenerator::save_ply(const std::string &filename) const {
    std::ofstream stream(filename);

    if (!stream) {
        throw std::runtime_error(std::format("Cannot open file for writing: {}", filename));
    }

    bool success = CGAL::IO::write_PLY(stream, _mesh);

    if (!success) {
        throw std::runtime_error(std::format("Cannot write file: {}", filename));
    }
}

SurfaceMesh GlobeGenerator::generate_globe_sphere() const {
    return _sphere_mesh_generator->generate(_radius, 5, Point3(0, 0, 0));
}

void GlobeGenerator::add_random_point() {
    add_point(_random_point_generator->generate());
}

void GlobeGenerator::add_point(Point3 location) {
    SurfaceMesh point_mesh = _sphere_mesh_generator->generate(_radius / 50, 1, location);
    add_mesh(point_mesh);
}

void GlobeGenerator::add_mesh(SurfaceMesh &mesh) {
    SurfaceMesh result;
    CGAL::Polygon_mesh_processing::corefine_and_compute_union(_mesh, mesh, result);
    _mesh = std::move(result);
}

} // namespace globe
