#ifndef GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_POINT_GENERATOR_HPP_

#include "../types.hpp"
#include "../geometry/cartesian/bounding_box.hpp"
#include "../geometry/cartesian/bounding_box_sampler.hpp"

namespace globe {

class RandomPointGenerator {
 public:
    Point3 generate();
    Point3 generate(const BoundingBox &bounding_box);

 private:
    BoundingBoxSampler _sampler;
};

inline Point3 RandomPointGenerator::generate() {
    static const BoundingBox unit_cube = BoundingBox::unit_cube();
    return generate(unit_cube);
}

inline Point3 RandomPointGenerator::generate(const BoundingBox &bounding_box) {
    return _sampler.sample(bounding_box);
}

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_POINT_GENERATOR_HPP_
