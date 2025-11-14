#ifndef GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_

#include "../types.hpp"
#include <CGAL/point_generators_3.h>

namespace globe {

typedef CGAL::Random_points_on_sphere_3<Point3> CGALRandomSpherePointGenerator;

class RandomSpherePointGenerator {
 public:
    explicit RandomSpherePointGenerator(double radius = 1.0);
    RandomSpherePointGenerator(RandomSpherePointGenerator &&other) noexcept;

    Point3 generate();

 private:
    double _radius;
    std::unique_ptr<CGALRandomSpherePointGenerator> _cgal_generator;
};

inline RandomSpherePointGenerator::RandomSpherePointGenerator(double radius) :
    _radius(radius),
    _cgal_generator(std::make_unique<CGALRandomSpherePointGenerator>(CGALRandomSpherePointGenerator())) {
}

inline RandomSpherePointGenerator::RandomSpherePointGenerator(RandomSpherePointGenerator &&other) noexcept:
    _radius(other._radius),
    _cgal_generator(std::move(other._cgal_generator)) {
}

inline Point3 RandomSpherePointGenerator::generate() {
    return *(*_cgal_generator)++;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_
