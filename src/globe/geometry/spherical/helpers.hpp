#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_

#include <cmath>
#include <algorithm>
#include "../../types.hpp"
#include "../../cgal_types.hpp"

namespace globe {

constexpr double UNIT_SPHERE_AREA = 4.0 * M_PI;

inline double distance(const VectorS2 &a, const VectorS2 &b) {
    CGAL_precondition(std::abs(a.squaredNorm() - 1.0) < GEOMETRIC_EPSILON);
    CGAL_precondition(std::abs(b.squaredNorm() - 1.0) < GEOMETRIC_EPSILON);

    double cos_theta = a.dot(b);
    return std::acos(std::clamp(cos_theta, -1.0, 1.0));
}

inline VectorS2 interpolate(const VectorS2 &source, const VectorS2 &target, double t) {
    double theta = distance(source, target);

    if (theta < GEOMETRIC_EPSILON) {
        return source;
    }

    double sin_theta = std::sin(theta);
    double a = std::sin((1.0 - t) * theta) / sin_theta;
    double b = std::sin(t * theta) / sin_theta;

    VectorS2 result = a * source + b * target;
    return result.normalized();
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_
