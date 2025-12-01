#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_UNIFORM_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_UNIFORM_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_

#include "../../../types.hpp"
#include "../spherical_bounding_box.hpp"
#include "../helpers.hpp"
#include "../../../math/interval_sampler/interval_sampler.hpp"
#include "../../../math/circular_interval_sampler/uniform_circular_interval_sampler.hpp"
#include <cmath>

namespace globe {

template<
    IntervalSampler IntervalSamplerType = UniformIntervalSampler,
    typename CircularIntervalSamplerType = UniformCircularIntervalSampler
>
class UniformSphericalBoundingBoxSampler {
 public:
    UniformSphericalBoundingBoxSampler() = default;

    UniformSphericalBoundingBoxSampler(
        IntervalSamplerType interval_sampler,
        CircularIntervalSamplerType circular_interval_sampler
    ) : _interval_sampler(std::move(interval_sampler)),
        _circular_interval_sampler(std::move(circular_interval_sampler)) {}

    [[nodiscard]] inline VectorS2 sample(const SphericalBoundingBox &bounding_box) {
        double theta_val = _circular_interval_sampler.sample(bounding_box.theta_interval());
        double z = _interval_sampler.sample(bounding_box.z_interval());

        double r = std::sqrt(1.0 - z * z);
        return VectorS2(
            r * std::cos(theta_val),
            r * std::sin(theta_val),
            z
        );
    }

 private:
    IntervalSamplerType _interval_sampler;
    CircularIntervalSamplerType _circular_interval_sampler;
};

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_UNIFORM_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
