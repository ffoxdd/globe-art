#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_

#include "../../types.hpp"

namespace globe {

template<typename T>
concept ScalarField = requires(
    T scalar_field,
    const Point3 &location
) {
    { scalar_field.value(location) } -> std::convertible_to<double>;
};

template<typename T>
concept BandLimitedScalarField = ScalarField<T> && requires(const T &field) {
    { field.max_frequency() } -> std::convertible_to<double>;
};

}

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_
