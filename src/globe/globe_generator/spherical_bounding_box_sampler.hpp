#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_

#include "../types.hpp"
#include "spherical_bounding_box.hpp"
#include "interval_sampler.hpp"
#include <cmath>

namespace globe {

class SphericalBoundingBoxSampler {
 public:
    [[nodiscard]] Point3 sample(const SphericalBoundingBox &bounding_box) {
        double theta = _interval_sampler.sample(bounding_box.theta_interval());
        double z = _interval_sampler.sample(bounding_box.z_interval());
        double r = std::sqrt(1 - (z * z)); // not the sphere's radius, but the radius of the xy circle at that z

        return {
            r * std::cos(theta),
            r * std::sin(theta),
            z
        };
    }

 private:
    IntervalSampler _interval_sampler;
};

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
