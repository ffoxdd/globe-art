#ifndef GLOBEART_SRC_GLOBE_GENERATORS_CARTESIAN_RANDOM_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_CARTESIAN_RANDOM_POINT_GENERATOR_HPP_

#include "../../cgal_types.hpp"
#include "../../geometry/cartesian/bounding_box.hpp"
#include "../../geometry/cartesian/bounding_box_sampler/bounding_box_sampler.hpp"
#include "../../geometry/cartesian/bounding_box_sampler/uniform_bounding_box_sampler.hpp"
#include <vector>

namespace globe::generators::cartesian {

template<BoundingBoxSampler BoundingBoxSamplerType = UniformBoundingBoxSampler<>>
class RandomPointGenerator {
 public:
    RandomPointGenerator() = default;

    explicit RandomPointGenerator(BoundingBoxSamplerType sampler)
        : _sampler(std::move(sampler)) {
    }

    std::vector<cgal::Point3> generate(size_t count);
    std::vector<cgal::Point3> generate(size_t count, const BoundingBox &bounding_box);

 private:
    BoundingBoxSamplerType _sampler;
};

template<BoundingBoxSampler BoundingBoxSamplerType>
std::vector<cgal::Point3>
RandomPointGenerator<BoundingBoxSamplerType>::generate(size_t count) {
    static const BoundingBox unit_cube = BoundingBox::unit_cube();
    return generate(count, unit_cube);
}

template<BoundingBoxSampler BoundingBoxSamplerType>
std::vector<cgal::Point3>
RandomPointGenerator<BoundingBoxSamplerType>::generate(size_t count, const BoundingBox &bounding_box) {
    std::vector<cgal::Point3> points;
    points.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        points.push_back(_sampler.sample(bounding_box));
    }
    return points;
}

} // namespace globe::generators::cartesian

#endif //GLOBEART_SRC_GLOBE_GENERATORS_CARTESIAN_RANDOM_POINT_GENERATOR_HPP_
