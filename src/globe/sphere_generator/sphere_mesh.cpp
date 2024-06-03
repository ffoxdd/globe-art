#include "sphere_mesh.h"
#include <CGAL/subdivision_method_3.h>

namespace globe {

SphereMesh &SphereMesh::generate() {
    create_icosahedron();
    subdivide();
    project_to_sphere();

    return *this;
}

SurfaceMesh SphereMesh::mesh() {
    return std::move(_mesh);
}

void SphereMesh::create_icosahedron() {
    CGAL::make_icosahedron(_mesh, _center, _radius);
}

void SphereMesh::subdivide() {
    CGAL::Subdivision_method_3::Loop_subdivision(
        _mesh,
        CGAL::parameters::number_of_iterations(_iterations)
    );
}

void SphereMesh::project_to_sphere() {
    for (auto vertex : _mesh.vertices()) {
        Point3 &point = _mesh.point(vertex);
        Kernel::Vector_3 vector = point - _center;

        double scale = _radius / std::sqrt(vector.squared_length());
        point = _center + (vector * scale);
    }
}

} // namespace globe
