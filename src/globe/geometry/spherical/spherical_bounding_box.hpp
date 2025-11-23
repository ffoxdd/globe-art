#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_HPP_

#include "../../math/interval.hpp"
#include "../../types.hpp"
#include "helpers.hpp"
#include <CGAL/assertions.h>
#include <cmath>

namespace globe {

class SphericalBoundingBox {
 public:
    SphericalBoundingBox();
    SphericalBoundingBox(Interval theta_interval, Interval z_interval);

    template<DoubleRange DoubleRangeType>
    SphericalBoundingBox(const DoubleRangeType &theta_values, const DoubleRangeType &z_values);

    [[nodiscard]] Interval theta_interval() const;
    [[nodiscard]] Interval z_interval() const;
    [[nodiscard]] double area() const;
    [[nodiscard]] Point3 center() const;
    [[nodiscard]] bool is_theta_wrapped() const;
    [[nodiscard]] double theta_measure() const;
    [[nodiscard]] double z_measure() const;
    [[nodiscard]] double bounding_sphere_radius() const;
    [[nodiscard]] bool contains(const Point3 &point) const;

    [[nodiscard]] static SphericalBoundingBox full_sphere();

 private:
    [[nodiscard]] bool contains_theta(double theta) const;

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

template<DoubleRange DoubleRangeType>
inline SphericalBoundingBox::SphericalBoundingBox(const DoubleRangeType &theta_values, const DoubleRangeType &z_values) :
    _theta_interval(theta_values),
    _z_interval(z_values) {
}

inline Interval SphericalBoundingBox::theta_interval() const {
    return _theta_interval;
}

inline Interval SphericalBoundingBox::z_interval() const {
    return _z_interval;
}

inline bool SphericalBoundingBox::is_theta_wrapped() const {
    return _theta_interval.high() > 2.0 * M_PI;
}

inline double SphericalBoundingBox::theta_measure() const {
    return _theta_interval.measure();
}

inline double SphericalBoundingBox::z_measure() const {
    return _z_interval.measure();
}

inline double SphericalBoundingBox::area() const {
    return theta_measure() * z_measure();
}

inline Point3 SphericalBoundingBox::center() const {
    double theta_mid = _theta_interval.midpoint();
    if (theta_mid >= 2.0 * M_PI) {
        theta_mid -= 2.0 * M_PI;
    }

    double z_mid = _z_interval.midpoint();
    double r_mid = std::sqrt(1.0 - z_mid * z_mid);

    return Point3(
        r_mid * std::cos(theta_mid),
        r_mid * std::sin(theta_mid),
        z_mid
    );
}

inline double SphericalBoundingBox::bounding_sphere_radius() const {
    double z_span = z_measure();
    double theta_span = theta_measure();

    double r_low = std::sqrt(std::max(0.0, 1.0 - _z_interval.low() * _z_interval.low()));
    double r_high = std::sqrt(std::max(0.0, 1.0 - _z_interval.high() * _z_interval.high()));
    double r_max = std::max(r_low, r_high);
    double chord = 2.0 * r_max * std::sin(theta_span / 2.0);

    return std::sqrt(z_span * z_span + chord * chord);
}

inline bool SphericalBoundingBox::contains_theta(double theta) const {
    if (_theta_interval.contains(theta)) {
        return true;
    }

    if (is_theta_wrapped()) {
        double wrapped_high = _theta_interval.high() - 2.0 * M_PI;
        return theta >= 0 && theta <= wrapped_high;
    }

    return false;
}

inline bool SphericalBoundingBox::contains(const Point3 &point) const {
    double theta_val = theta(point.x(), point.y());
    double z_val = static_cast<double>(point.z());
    return contains_theta(theta_val) && _z_interval.contains(z_val);
}

inline SphericalBoundingBox SphericalBoundingBox::full_sphere() {
    return SphericalBoundingBox(Interval(0, 2 * M_PI), Interval(-1, 1));
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_HPP_