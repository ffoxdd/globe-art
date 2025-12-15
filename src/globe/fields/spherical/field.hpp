#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_FIELD_HPP_

#include <cstdint>
#include "../../types.hpp"
#include "../../geometry/spherical/arc.hpp"
#include "../../geometry/spherical/polygon/polygon.hpp"
#include <Eigen/Core>

namespace globe::fields::spherical {

template<typename T>
concept Field = requires(
    const T& field,
    const VectorS2& point,
    const Polygon& polygon,
    const Arc& arc
) {
    { field.value(point) } -> std::convertible_to<double>;
    { field.mass(polygon) } -> std::convertible_to<double>;
    { field.total_mass() } -> std::convertible_to<double>;
    { field.edge_integral(arc) } -> std::convertible_to<double>;
    { field.edge_gradient_integral(arc) } -> std::convertible_to<Eigen::Vector3d>;
};

} // namespace globe::fields::spherical

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_FIELD_HPP_
