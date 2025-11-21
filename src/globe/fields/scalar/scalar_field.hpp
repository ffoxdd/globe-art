#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_

#include "../../types.hpp"
#include "../../math/interval.hpp"

namespace globe {

template<typename T>
concept ScalarField = requires(
    T scalar_field,
    const Point3 &location
) {
    { scalar_field.value(location) } -> std::convertible_to<double>;
    { scalar_field.output_range() } -> std::convertible_to<Interval>;
};

}

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_
