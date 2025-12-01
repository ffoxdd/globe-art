#ifndef GLOBEART_SRC_GLOBE_GENERATORS_SPHERICAL_RANDOM_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_SPHERICAL_RANDOM_POINT_GENERATOR_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../geometry/spherical/spherical_bounding_box_sampler/spherical_bounding_box_sampler.hpp"
#include "../../geometry/spherical/spherical_bounding_box_sampler/uniform_spherical_bounding_box_sampler.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../cartesian/random_point_generator.hpp"
#include "../cartesian/point_generator.hpp"

namespace globe::generators::spherical {

template<
    cartesian::PointGenerator CartesianGeneratorType = cartesian::RandomPointGenerator<>,
    SphericalBoundingBoxSampler SphericalBoundingBoxSamplerType = UniformSphericalBoundingBoxSampler<>
>
class RandomPointGenerator {
 public:
    RandomPointGenerator() = default;

    RandomPointGenerator(
        CartesianGeneratorType cartesian_generator,
        SphericalBoundingBoxSamplerType spherical_sampler
    ) : _cartesian_generator(std::move(cartesian_generator)),
        _spherical_sampler(std::move(spherical_sampler)) {
    }

    std::vector<VectorS2> generate(size_t count);
    std::vector<VectorS2> generate(size_t count, const SphericalBoundingBox &bounding_box);

    [[nodiscard]] size_t last_attempt_count() const { return _last_attempt_count; }

 private:
    CartesianGeneratorType _cartesian_generator;
    SphericalBoundingBoxSamplerType _spherical_sampler;
    size_t _last_attempt_count = 0;
};

template<cartesian::PointGenerator CartesianGeneratorType, SphericalBoundingBoxSampler SphericalBoundingBoxSamplerType>
std::vector<VectorS2> RandomPointGenerator<CartesianGeneratorType, SphericalBoundingBoxSamplerType>::generate(size_t count) {
    auto cartesian_points = _cartesian_generator.generate(count);
    std::vector<VectorS2> sphere_points;
    sphere_points.reserve(count);
    for (const auto& point : cartesian_points) {
        sphere_points.push_back(to_eigen(point).normalized());
    }
    _last_attempt_count = count;
    return sphere_points;
}

template<cartesian::PointGenerator CartesianGeneratorType, SphericalBoundingBoxSampler SphericalBoundingBoxSamplerType>
std::vector<VectorS2> RandomPointGenerator<CartesianGeneratorType, SphericalBoundingBoxSamplerType>::generate(size_t count, const SphericalBoundingBox &bounding_box) {
    std::vector<VectorS2> points;
    points.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        points.push_back(_spherical_sampler.sample(bounding_box));
    }
    _last_attempt_count = count;
    return points;
}

} // namespace globe::generators::spherical

#endif //GLOBEART_SRC_GLOBE_GENERATORS_SPHERICAL_RANDOM_POINT_GENERATOR_HPP_
