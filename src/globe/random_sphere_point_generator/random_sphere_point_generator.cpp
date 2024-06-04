#include "random_sphere_point_generator.h"

namespace globe {

Point3 RandomSpherePointGenerator::generate() {
    return *_cgal_generator++;
}


} // namespace globe