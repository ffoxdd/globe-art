#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SPHERICAL_FIELD_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include <Eigen/Core>

namespace globe {

template<typename T>
concept SphericalField = requires(
    const T& field,
    const Point3& point,
    const SphericalPolygon& polygon,
    const SphericalArc& arc
) {
    { field.value(point) } -> std::convertible_to<double>;
    { field.mass(polygon) } -> std::convertible_to<double>;
    { field.total_mass() } -> std::convertible_to<double>;
    { field.edge_integral(arc) } -> std::convertible_to<double>;
    { field.edge_gradient_integral(arc) } -> std::convertible_to<Eigen::Vector3d>;
};

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SPHERICAL_FIELD_HPP_
