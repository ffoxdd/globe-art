#ifndef GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_

#include "../types.hpp"
#include "../geometry/spherical/spherical_bounding_box.hpp"
#include "../geometry/spherical/spherical_bounding_box_sampler.hpp"
#include "../geometry/spherical/helpers.hpp"
#include "random_point_generator.hpp"

namespace globe {

class RandomSpherePointGenerator {
 public:
    Point3 generate();
    Point3 generate(const SphericalBoundingBox &bounding_box);

 private:
    RandomPointGenerator _cartesian_generator;
    SphericalBoundingBoxSampler _spherical_sampler;
};

inline Point3 RandomSpherePointGenerator::generate() {
    Point3 cartesian_point = _cartesian_generator.generate();
    return project_to_sphere(cartesian_point);
}

inline Point3 RandomSpherePointGenerator::generate(const SphericalBoundingBox &bounding_box) {
    return _spherical_sampler.sample(bounding_box);
}

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_
