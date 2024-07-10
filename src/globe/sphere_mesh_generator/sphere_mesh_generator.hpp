#ifndef GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_

#include "../types.hpp"
#include <CGAL/subdivision_method_3.h>

namespace globe {

class SphereMeshGenerator {
 public:
    static SurfaceMesh generate(double radius, int iterations, Point3 center);

 private:
    class SphereMesh {
     public:
        explicit SphereMesh(
            double radius = 1.0,
            int iterations = 3,
            Point3 center = Point3(0, 0, 0)
        ) :
            _center(center),
            _radius(radius),
            _iterations(iterations) { }

        SphereMesh &generate();
        SurfaceMesh mesh();

     private:
        Point3 _center;
        double _radius;
        int _iterations;
        SurfaceMesh _mesh;

        void create_icosahedron();
        void subdivide();
        void project_to_sphere();
    };
};

SurfaceMesh SphereMeshGenerator::generate(double radius, int iterations, Point3 center) {
    return SphereMesh(radius, iterations, center).generate().mesh();
}

SphereMeshGenerator::SphereMesh &SphereMeshGenerator::SphereMesh::generate() {
    create_icosahedron();
    subdivide();
    project_to_sphere();
    return *this;
}

SurfaceMesh SphereMeshGenerator::SphereMesh::mesh() {
    return std::move(_mesh);
}

void SphereMeshGenerator::SphereMesh::create_icosahedron() {
    CGAL::make_icosahedron(_mesh, _center, _radius);
}

void SphereMeshGenerator::SphereMesh::subdivide() {
    CGAL::Subdivision_method_3::Loop_subdivision(
        _mesh,
        CGAL::parameters::number_of_iterations(_iterations)
    );
}

void SphereMeshGenerator::SphereMesh::project_to_sphere() {
    for (auto vertex : _mesh.vertices()) {
        // TODO: make a common utility function for this
        Point3 &point = _mesh.point(vertex);
        Kernel::Vector_3 vector = point - _center;
        double scale = _radius / std::sqrt(vector.squared_length());
        point = _center + (vector * scale);
    }
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_
