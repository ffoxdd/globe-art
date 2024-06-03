#include "globe_generator.h"
#include "sphere_generator/sphere_mesh.h"
#include <iostream>
#include <CGAL/Polygon_mesh_processing/corefinement.h>

namespace globe {

GlobeGenerator &GlobeGenerator::generate() {
    _mesh = generate_globe_sphere();

    add_point(Point3(_radius, 0, 0));
    add_point(Point3(-_radius, 0, 0));
    add_point(Point3(0, _radius, 0));
    add_point(Point3(0, -_radius, 0));
    add_point(Point3(0, 0, _radius));
    add_point(Point3(0, 0, -_radius));

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
    return generate_sphere(_radius, 5, Point3(0, 0, 0));
}

SurfaceMesh GlobeGenerator::generate_sphere(double radius, int iterations, Point3 center) {
    return SphereMesh(radius, iterations, center).generate().mesh();
}

void GlobeGenerator::add_point(Point3 location) {
    SurfaceMesh point_mesh = generate_sphere(0.025, 1, location);
    add_mesh(point_mesh);
}

void GlobeGenerator::add_mesh(SurfaceMesh &mesh) {
    SurfaceMesh result;
    CGAL::Polygon_mesh_processing::corefine_and_compute_union(_mesh, mesh, result);
    _mesh = std::move(result);
}

} // namespace globe
