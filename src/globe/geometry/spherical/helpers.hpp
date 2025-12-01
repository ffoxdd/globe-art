#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_

#include <cmath>
#include <algorithm>
#include "../../types.hpp"
#include <CGAL/basic.h>

namespace globe::geometry::spherical {

using globe::VectorS2;
using globe::GEOMETRIC_EPSILON;

constexpr double UNIT_SPHERE_AREA = 4.0 * M_PI;

inline double distance(const VectorS2 &a, const VectorS2 &b) {
    CGAL_precondition(std::abs(a.squaredNorm() - 1.0) < GEOMETRIC_EPSILON);
    CGAL_precondition(std::abs(b.squaredNorm() - 1.0) < GEOMETRIC_EPSILON);

    double cos_theta = std::clamp(a.dot(b), -1.0, 1.0);
    return std::acos(cos_theta);
}

} // namespace globe::geometry::spherical

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_
