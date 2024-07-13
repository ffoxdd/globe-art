#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_HPP_

#include "../noise_generator/interval.hpp"
#include <cmath>

namespace globe {

class SphericalBoundingBox {
 public:
    SphericalBoundingBox() :
        SphericalBoundingBox(
            Interval(0, 2 * M_PI),
            Interval(-1, 1)
        ) {
    }

    SphericalBoundingBox(Interval theta_interval, Interval z_interval) :
        _theta_interval(theta_interval),
        _z_interval(z_interval) {
    }

    template<DoubleRange DR>
    SphericalBoundingBox(const DR &theta_values, const DR &z_values) :
        _theta_interval(theta_values),
        _z_interval(z_values) {
    };

    [[nodiscard]] Interval theta_interval() const {
        return _theta_interval;
    }

    [[nodiscard]] Interval z_interval() const {
        return _z_interval;
    }

 private:
    const Interval _theta_interval;
    const Interval _z_interval;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_HPP_
