#ifndef GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_

#include "../types.hpp"
#include "../geometry/spherical/spherical_bounding_box.hpp"
#include "../geometry/spherical/spherical_bounding_box_sampler.hpp"
#include "../geometry/spherical/uniform_spherical_bounding_box_sampler.hpp"
#include "../geometry/spherical/helpers.hpp"
#include "random_point_generator.hpp"
#include "point_generator.hpp"

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

    Point3 generate();
    Point3 generate(const SphericalBoundingBox &bounding_box);

 private:
    CartesianGeneratorType _cartesian_generator;
    SphericalBoundingBoxSamplerType _spherical_sampler;
};

template<PointGenerator CartesianGeneratorType, SphericalBoundingBoxSampler SphericalBoundingBoxSamplerType>
Point3 RandomSpherePointGenerator<CartesianGeneratorType, SphericalBoundingBoxSamplerType>::generate() {
    Point3 cartesian_point = _cartesian_generator.generate();
    return project_to_sphere(cartesian_point);
}

template<PointGenerator CartesianGeneratorType, SphericalBoundingBoxSampler SphericalBoundingBoxSamplerType>
Point3 RandomSpherePointGenerator<CartesianGeneratorType, SphericalBoundingBoxSamplerType>::generate(const SphericalBoundingBox &bounding_box) {
    return _spherical_sampler.sample(bounding_box);
}

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_RANDOM_SPHERE_POINT_GENERATOR_HPP_
