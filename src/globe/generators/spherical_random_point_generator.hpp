#ifndef GLOBEART_SRC_GLOBE_GENERATORS_SPHERICAL_RANDOM_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_SPHERICAL_RANDOM_POINT_GENERATOR_HPP_

#include "../types.hpp"
#include "../spherical/spherical_bounding_box.hpp"
#include "../spherical/spherical_bounding_box_sampler.hpp"

namespace globe {

class SphericalRandomPointGenerator {
 public:
    Point3 generate();
    Point3 generate(const SphericalBoundingBox &bounding_box);

 private:
    SphericalBoundingBoxSampler _sampler;
};

inline Point3 SphericalRandomPointGenerator::generate() {
    return generate(SphericalBoundingBox::full_sphere());
}

inline Point3 SphericalRandomPointGenerator::generate(const SphericalBoundingBox &bounding_box) {
    return _sampler.sample(bounding_box);
}

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_SPHERICAL_RANDOM_POINT_GENERATOR_HPP_

