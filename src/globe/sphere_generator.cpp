#include "sphere_generator.h"
#include "types.h"
#include <CGAL/subdivision_method_3.h>

namespace globe {

void SphereGenerator::generate() {
    create_icosahedron();
    subdivide();
    project_to_sphere();
}

std::unique_ptr<SurfaceMesh> SphereGenerator::mesh() {
    if (!_mesh) {
        throw std::runtime_error("Mesh has already been moved");
    }

    return std::move(_mesh);
}

void SphereGenerator::create_icosahedron() {
    CGAL::make_icosahedron(*_mesh, _center, _radius);
}

void SphereGenerator::subdivide() {
    CGAL::Subdivision_method_3::CatmullClark_subdivision(
        *_mesh,
        CGAL::parameters::number_of_iterations(_iterations)
    );
}

void SphereGenerator::project_to_sphere() {
    for (auto vertex : _mesh->vertices()) {
        Point3 &point = _mesh->point(vertex);
        Kernel::Vector_3 vector = point - CGAL::ORIGIN;

        double scale = _radius / std::sqrt(vector.squared_length());
        point = CGAL::ORIGIN + (vector * scale);
    }
}

} // namespace globe
