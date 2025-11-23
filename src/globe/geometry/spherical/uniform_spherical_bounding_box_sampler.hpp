#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_UNIFORM_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_UNIFORM_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_

#include "../../types.hpp"
#include "spherical_bounding_box.hpp"
#include "../../math/interval_sampler.hpp"
#include <cmath>

namespace globe {

template<IntervalSampler IntervalSamplerType = UniformIntervalSampler>
class UniformSphericalBoundingBoxSampler {
 public:
    UniformSphericalBoundingBoxSampler() = default;

    explicit UniformSphericalBoundingBoxSampler(IntervalSamplerType interval_sampler)
        : _interval_sampler(std::move(interval_sampler)) {
    }

    [[nodiscard]] inline Point3 sample(const SphericalBoundingBox &bounding_box) {
        double theta = sample_theta(bounding_box);
        double z = _interval_sampler.sample(bounding_box.z_interval());
        double r = std::sqrt(1 - (z * z));

        return {
            r * std::cos(theta),
            r * std::sin(theta),
            z
        };
    }

    [[nodiscard]] inline double sample_theta(const SphericalBoundingBox &bounding_box) {
        Interval theta_interval = bounding_box.theta_interval();

        if (!bounding_box.is_theta_wrapped()) {
            return _interval_sampler.sample(theta_interval);
        }

        double span = bounding_box.theta_interval().measure();
        double offset = _interval_sampler.sample(Interval(0.0, span));
        double theta = theta_interval.low() + offset;

        if (theta >= 2.0 * M_PI) {
            theta -= 2.0 * M_PI;
        }

        return theta;
    }

 private:
    IntervalSamplerType _interval_sampler;
};

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_UNIFORM_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
