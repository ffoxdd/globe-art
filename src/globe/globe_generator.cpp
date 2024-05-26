#include "globe_generator.h"
#include "sphere_generator.h"
#include <iostream>

namespace globe {

void GlobeGenerator::generate() {
    _mesh = generate_globe_sphere();
}

void GlobeGenerator::save_ply(const std::string &filename) const {
    std::ofstream stream(filename);

    if (!stream) {
        throw std::runtime_error(std::format("Cannot open file for writing: {}", filename));
    }

    bool success = CGAL::IO::write_PLY(stream, *_mesh);

    if (!success) {
        throw std::runtime_error(std::format("Cannot write file: {}", filename));
    }
}

std::unique_ptr<SurfaceMesh> GlobeGenerator::generate_globe_sphere() {
    SphereGenerator sphere_generator(1.0, 7, Point3(0, 0, 0));
    sphere_generator.generate();
    return sphere_generator.mesh();
}

} // namespace globe
