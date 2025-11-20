#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_HPP_

#include "../scalar_field/interval.hpp"
#include "../types.hpp"
#include <CGAL/assertions.h>
#include <cmath>

namespace globe {

class SphericalBoundingBox {
 public:
    SphericalBoundingBox();
    SphericalBoundingBox(Interval theta_interval, Interval z_interval);

    template<DoubleRange DR>
    SphericalBoundingBox(const DR &theta_values, const DR &z_values);

    [[nodiscard]] Interval theta_interval() const;
    [[nodiscard]] Interval z_interval() const;
    [[nodiscard]] double area() const;
    [[nodiscard]] Point3 center() const;

 private:
    const Interval _theta_interval;
    const Interval _z_interval;
};

inline SphericalBoundingBox::SphericalBoundingBox() :
    SphericalBoundingBox(
        Interval(0, 2 * M_PI),
        Interval(-1, 1)
    ) {
}

inline SphericalBoundingBox::SphericalBoundingBox(Interval theta_interval, Interval z_interval) :
    _theta_interval(theta_interval),
    _z_interval(z_interval) {
    constexpr double tolerance = 1e-10;
    CGAL_precondition(z_interval.low() >= -1.0 - tolerance);
    CGAL_precondition(z_interval.high() <= 1.0 + tolerance);
}

template<DoubleRange DR>
inline SphericalBoundingBox::SphericalBoundingBox(const DR &theta_values, const DR &z_values) :
    _theta_interval(theta_values),
    _z_interval(z_values) {
}

inline Interval SphericalBoundingBox::theta_interval() const {
    return _theta_interval;
}

inline Interval SphericalBoundingBox::z_interval() const {
    return _z_interval;
}

inline double SphericalBoundingBox::area() const {
    return _theta_interval.measure() * _z_interval.measure();
}

inline Point3 SphericalBoundingBox::center() const {
    double theta_mid = _theta_interval.midpoint();
    double z_mid = _z_interval.midpoint();
    double r_mid = std::sqrt(1.0 - z_mid * z_mid);

    return Point3(
        r_mid * std::cos(theta_mid),
        r_mid * std::sin(theta_mid),
        z_mid
    );
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_BOUNDING_BOX_HPP_