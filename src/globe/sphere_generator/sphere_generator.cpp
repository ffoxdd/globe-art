#include "sphere_generator.h"
#include "sphere_mesh.h"

namespace globe {

SurfaceMesh globe::SphereGenerator::generate(double radius, int iterations, Point3 center) {
    return SphereMesh(radius, iterations, center).generate().mesh();
}

} // namespace globe
