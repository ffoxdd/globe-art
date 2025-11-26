#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_HPP_

#include "../../math/interval.hpp"
#include "../../math/circular_interval.hpp"
#include "../../types.hpp"
#include "helpers.hpp"
#include <CGAL/assertions.h>
#include <cmath>

namespace globe {

using ThetaInterval = CircularInterval<TWO_PI>;

class SphericalBoundingBox {
 public:
    SphericalBoundingBox(ThetaInterval theta_interval, Interval z_interval);

    [[nodiscard]] ThetaInterval theta_interval() const;
    [[nodiscard]] Interval z_interval() const;
    [[nodiscard]] double area() const;
    [[nodiscard]] Point3 center() const;
    [[nodiscard]] double bounding_sphere_radius() const;
    [[nodiscard]] bool contains(const Point3 &point) const;

    [[nodiscard]] static SphericalBoundingBox full_sphere();

 private:
    ThetaInterval _theta_interval;
    Interval _z_interval;
};

inline SphericalBoundingBox::SphericalBoundingBox(ThetaInterval theta_interval, Interval z_interval) :
    _theta_interval(theta_interval),
    _z_interval(z_interval) {
    CGAL_precondition(z_interval.low() >= -1.0 - GEOMETRIC_EPSILON);
    CGAL_precondition(z_interval.high() <= 1.0 + GEOMETRIC_EPSILON);
}

inline ThetaInterval SphericalBoundingBox::theta_interval() const {
    return _theta_interval;
}

inline Interval SphericalBoundingBox::z_interval() const {
    return _z_interval;
}

inline double SphericalBoundingBox::area() const {
    return _theta_interval.measure() * _z_interval.measure();
}

inline Point3 SphericalBoundingBox::center() const {
    double theta_mid = _theta_interval.start() + _theta_interval.measure() / 2.0;
    if (theta_mid >= TWO_PI) {
        theta_mid -= TWO_PI;
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
    double z_span = _z_interval.measure();
    double theta_span = _theta_interval.measure();

    double r_low = std::sqrt(std::max(0.0, 1.0 - _z_interval.low() * _z_interval.low()));
    double r_high = std::sqrt(std::max(0.0, 1.0 - _z_interval.high() * _z_interval.high()));
    double r_max = std::max(r_low, r_high);
    double chord = 2.0 * r_max * std::sin(theta_span / 2.0);

    return std::sqrt(z_span * z_span + chord * chord);
}

inline bool SphericalBoundingBox::contains(const Point3 &point) const {
    double theta_val = theta(point.x(), point.y());
    double z_val = CGAL::to_double(point.z());
    return _theta_interval.contains(theta_val) && _z_interval.contains(z_val);
}

inline SphericalBoundingBox SphericalBoundingBox::full_sphere() {
    return SphericalBoundingBox(ThetaInterval::full(), Interval(-1, 1));
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_HPP_