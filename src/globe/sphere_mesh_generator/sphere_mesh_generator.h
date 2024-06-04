#ifndef GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_

#include "../types.h"

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

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_MESH_GENERATOR_H_
