#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_INTEGRABLE_FIELD_H_

#include "../../geometry/spherical/spherical_polygon.hpp"

namespace globe {

template<typename T>
concept IntegrableField = requires(
    T integrable_field,
    const SphericalPolygon &polygon
) {
    { integrable_field.integrate(polygon) } -> std::convertible_to<double>;
};

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_INTEGRABLE_FIELD_H_
