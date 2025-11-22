#ifndef GLOBEART_SRC_GLOBE_TESTING_GEOMETRIC_ASSERTIONS_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_GEOMETRIC_ASSERTIONS_HPP_

#include "../types.hpp"
#include <cmath>

namespace globe {
namespace testing {

constexpr double DEFAULT_TOLERANCE = 1e-9;

inline bool is_on_unit_sphere(const Point3 &point, double tolerance = DEFAULT_TOLERANCE) {
    double distance = std::sqrt(
        point.x() * point.x() +
        point.y() * point.y() +
        point.z() * point.z()
    );

    return std::abs(distance - 1.0) < tolerance;
}

inline bool points_approximately_equal(
    const Point3 &a,
    const Point3 &b,
    double tolerance = DEFAULT_TOLERANCE
) {
    double dx = a.x() - b.x();
    double dy = a.y() - b.y();
    double dz = a.z() - b.z();
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance < tolerance;
}

}
}

#endif //GLOBEART_SRC_GLOBE_TESTING_GEOMETRIC_ASSERTIONS_HPP_
