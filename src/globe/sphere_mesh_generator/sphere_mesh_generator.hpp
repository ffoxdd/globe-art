#ifndef GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_

#include "../types.hpp"
#include "../geometry/helpers.hpp"
#include <CGAL/subdivision_method_3.h>
#include <CGAL/boost/graph/generators.h>

namespace globe {
    class SphereMeshGenerator {
    public:
        static inline SurfaceMesh generate(double radius, int iterations, Point3 center);

    private:
        class SphereMesh {
        public:
            explicit SphereMesh(
                double radius = 1.0,
                int iterations = 3,
                Point3 center = Point3(0, 0, 0)
            ) : _center(center),
                _radius(radius),
                _iterations(iterations) {
            }

            inline SphereMesh &generate();

            inline SurfaceMesh mesh();

        private:
            Point3 _center;
            double _radius;
            int _iterations;
            SurfaceMesh _mesh;

            inline void create_icosahedron();

            inline void subdivide();

            inline void project_to_sphere();
        };
    };

    inline SurfaceMesh SphereMeshGenerator::generate(double radius, int iterations, Point3 center) {
        return SphereMesh(radius, iterations, center).generate().mesh();
    }

    inline SphereMeshGenerator::SphereMesh &SphereMeshGenerator::SphereMesh::generate() {
        create_icosahedron();
        subdivide();
        project_to_sphere();
        return *this;
    }

    inline SurfaceMesh SphereMeshGenerator::SphereMesh::mesh() {
        return std::move(_mesh);
    }

    inline void SphereMeshGenerator::SphereMesh::create_icosahedron() {
        CGAL::make_icosahedron(_mesh, _center, _radius);
    }

    inline void SphereMeshGenerator::SphereMesh::subdivide() {
        CGAL::Subdivision_method_3::Loop_subdivision(
            _mesh,
            CGAL::parameters::number_of_iterations(_iterations)
        );
    }

    inline void SphereMeshGenerator::SphereMesh::project_to_sphere() {
        for (auto vertex: _mesh.vertices()) {
            Point3 &point = _mesh.point(vertex);
            point = globe::project_to_sphere(point, _center, _radius);
        }
    }
} // namespace globe

#endif //GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_
