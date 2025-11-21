#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_

#include "../types.hpp"
#include "../spherical/spherical_bounding_box.hpp"
#include "interval_sampler.hpp"
#include <cmath>

namespace globe {

class SphericalBoundingBoxSampler {
 public:
    SphericalBoundingBoxSampler() = default;
    SphericalBoundingBoxSampler(SphericalBoundingBoxSampler &&other) noexcept = default;
    SphericalBoundingBoxSampler& operator=(SphericalBoundingBoxSampler &&other) noexcept = default;

    SphericalBoundingBoxSampler(const SphericalBoundingBoxSampler&) = delete;
    SphericalBoundingBoxSampler& operator=(const SphericalBoundingBoxSampler&) = delete;

    [[nodiscard]] inline Point3 sample(const SphericalBoundingBox &bounding_box) {
        double theta = sample_theta(bounding_box);
        double z = _interval_sampler.sample(bounding_box.z_interval());
        double r = std::sqrt(1 - (z * z)); // not the sphere's radius, but the radius of the xy circle at that z

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

        double span = bounding_box.theta_measure();
        double offset = _interval_sampler.sample(Interval(0.0, span));
        double theta = theta_interval.low() + offset;

        if (theta >= 2.0 * M_PI) {
            theta -= 2.0 * M_PI;
        }

        return theta;
    }

 private:
    IntervalSampler _interval_sampler;
};

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
