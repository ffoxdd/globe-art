#ifndef GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_

#include "../types.hpp"
#include "point_generator.hpp"
#include <CGAL/point_generators_3.h>

namespace globe {

typedef CGAL::Random_points_on_sphere_3<Point3> CGALRandomSpherePointGenerator;

class RandomSpherePointGenerator {
 public:
    struct Config {
        double radius = 1.0;

        std::unique_ptr<CGALRandomSpherePointGenerator> cgal_generator =
            std::make_unique<CGALRandomSpherePointGenerator>(CGALRandomSpherePointGenerator());
    };

    explicit RandomSpherePointGenerator(Config &&config) :
        _radius(config.radius),
        _cgal_generator(std::move(config.cgal_generator)) { };

    explicit RandomSpherePointGenerator(double radius) :
        _radius(radius),
        _cgal_generator(std::make_unique<CGALRandomSpherePointGenerator>(CGALRandomSpherePointGenerator())) { };

    RandomSpherePointGenerator(RandomSpherePointGenerator &&other) noexcept:
        _radius(other._radius),
        _cgal_generator(std::move(other._cgal_generator)) { }

    RandomSpherePointGenerator &operator=(RandomSpherePointGenerator &&other) noexcept {
        if (this != &other) {
            _radius = other._radius;
            _cgal_generator = std::move(other._cgal_generator);
        }

        return *this;
    }

    RandomSpherePointGenerator(const RandomSpherePointGenerator &) = delete;
    RandomSpherePointGenerator &operator=(const RandomSpherePointGenerator &) = delete;

    Point3 generate();

 private:
    double _radius;
    std::unique_ptr<CGALRandomSpherePointGenerator> _cgal_generator;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_RANDOM_SPHERE_POINT_GENERATOR_RANDOM_SPHERE_POINT_GENERATOR_H_
