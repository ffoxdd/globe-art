#ifndef GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../geometry/spherical/spherical_bounding_box_sampler/spherical_bounding_box_sampler.hpp"
#include "../../geometry/spherical/spherical_bounding_box_sampler/uniform_spherical_bounding_box_sampler.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../point_generator/random_point_generator.hpp"
#include "../point_generator/point_generator.hpp"

namespace globe {

template<
    PointGenerator CartesianGeneratorType = RandomPointGenerator<>,
    SphericalBoundingBoxSampler SphericalBoundingBoxSamplerType = UniformSphericalBoundingBoxSampler<>
>
class RandomSpherePointGenerator {
 public:
    RandomSpherePointGenerator() = default;

    RandomSpherePointGenerator(
        CartesianGeneratorType cartesian_generator,
        SphericalBoundingBoxSamplerType spherical_sampler
    ) : _cartesian_generator(std::move(cartesian_generator)),
        _spherical_sampler(std::move(spherical_sampler)) {
    }

    std::vector<Point3> generate(size_t count);
    std::vector<Point3> generate(size_t count, const SphericalBoundingBox &bounding_box);

 private:
    CartesianGeneratorType _cartesian_generator;
    SphericalBoundingBoxSamplerType _spherical_sampler;
};

template<PointGenerator CartesianGeneratorType, SphericalBoundingBoxSampler SphericalBoundingBoxSamplerType>
std::vector<Point3> RandomSpherePointGenerator<CartesianGeneratorType, SphericalBoundingBoxSamplerType>::generate(size_t count) {
    auto cartesian_points = _cartesian_generator.generate(count);
    std::vector<Point3> sphere_points;
    sphere_points.reserve(count);
    for (const auto& point : cartesian_points) {
        sphere_points.push_back(project_to_sphere(point));
    }
    return sphere_points;
}

template<PointGenerator CartesianGeneratorType, SphericalBoundingBoxSampler SphericalBoundingBoxSamplerType>
std::vector<Point3> RandomSpherePointGenerator<CartesianGeneratorType, SphericalBoundingBoxSamplerType>::generate(size_t count, const SphericalBoundingBox &bounding_box) {
    std::vector<Point3> points;
    points.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        points.push_back(_spherical_sampler.sample(bounding_box));
    }
    return points;
}

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_
