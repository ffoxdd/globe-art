#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SPHERICAL_FIELD_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"
#include <Eigen/Core>

namespace globe {

template<typename T>
concept SphericalField = requires(
    const T& field,
    const Point3& point,
    const PolygonMoments& polygon_moments,
    const ArcMoments& arc_moments
) {
    { field.value(point) } -> std::convertible_to<double>;
    { field.mass(polygon_moments) } -> std::convertible_to<double>;
    { field.edge_integral(arc_moments) } -> std::convertible_to<double>;
    { field.edge_gradient_integral(arc_moments) } -> std::convertible_to<Eigen::Vector3d>;
};

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SPHERICAL_FIELD_HPP_
