#ifndef GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_POINT_GENERATOR_HPP_

#include "../types.hpp"
#include "../geometry/cartesian/bounding_box.hpp"
#include "../geometry/cartesian/cartesian_bounding_box_sampler.hpp"
#include "../geometry/cartesian/uniform_cartesian_bounding_box_sampler.hpp"

namespace globe {

template<CartesianBoundingBoxSampler BoundingBoxSamplerType = UniformCartesianBoundingBoxSampler<>>
class RandomPointGenerator {
 public:
    RandomPointGenerator() = default;

    explicit RandomPointGenerator(BoundingBoxSamplerType sampler)
        : _sampler(std::move(sampler)) {
    }

    Point3 generate();
    Point3 generate(const BoundingBox &bounding_box);

 private:
    BoundingBoxSamplerType _sampler;
};

template<CartesianBoundingBoxSampler BoundingBoxSamplerType>
Point3 RandomPointGenerator<BoundingBoxSamplerType>::generate() {
    static const BoundingBox unit_cube = BoundingBox::unit_cube();
    return generate(unit_cube);
}

template<CartesianBoundingBoxSampler BoundingBoxSamplerType>
Point3 RandomPointGenerator<BoundingBoxSamplerType>::generate(const BoundingBox &bounding_box) {
    return _sampler.sample(bounding_box);
}

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_POINT_GENERATOR_HPP_
