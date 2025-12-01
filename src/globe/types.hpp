#ifndef GLOBEART_SRC_GLOBE_TYPES_H_
#define GLOBEART_SRC_GLOBE_TYPES_H_

#include <Eigen/Core>

namespace globe {

// Canonical type for points on the unit sphere.
// Represents a normalized 3D vector (direction from origin to point on S2).
using VectorS2 = Eigen::Vector3d;

constexpr double TWO_PI = 2.0 * M_PI;
constexpr double GEOMETRIC_EPSILON = 1e-10;

}

#endif //GLOBEART_SRC_GLOBE_TYPES_H_
