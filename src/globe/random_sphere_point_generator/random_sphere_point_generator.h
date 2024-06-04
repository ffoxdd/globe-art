#ifndef GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_

#include "../types.h"
#include <CGAL/point_generators_3.h>

namespace globe {

typedef CGAL::Random_points_on_sphere_3<Point3> CGALRandomSpherePointGenerator;

class RandomSpherePointGenerator {
 public:
    explicit RandomSpherePointGenerator(double radius) : _radius(radius), _cgal_generator(radius) { };
    Point3 generate();

 private:
    double _radius;
    CGALRandomSpherePointGenerator _cgal_generator;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_
