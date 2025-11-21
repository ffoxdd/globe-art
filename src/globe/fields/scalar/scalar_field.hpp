#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_

#include "../../types.hpp"
#include "../../math/interval.hpp"

namespace globe {

template<typename T>
concept ScalarField = requires(
    T scalar_field,
    const Point3 &location,
    std::vector<Point3> &sample_points,
    Interval output_range
) {
    { scalar_field.value(location) } -> std::convertible_to<double>;
    { scalar_field.normalize(sample_points, output_range) } -> std::same_as<void>;
};

}

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SCALAR_FIELD_H_
