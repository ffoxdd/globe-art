#ifndef GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_GENERATOR_H_

#include "../types.h"

namespace globe {

class SphereGenerator {
 public:
    static SurfaceMesh generate(double radius, int iterations, Point3 center);
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_SPHERE_GENERATOR_SPHERE_GENERATOR_H_
