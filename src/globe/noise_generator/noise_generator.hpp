#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_

#include "../types.hpp"
#include "interval.hpp"

namespace globe {

template<typename T>
concept NoiseGenerator = requires(
    T noise_generator,
    const Point3 &location,
    std::vector<Point3> &sample_points,
    Interval output_range
) {
    { noise_generator.value(location) } -> std::convertible_to<double>;
    { noise_generator.normalize(sample_points, output_range) } -> std::same_as<void>;
};

}

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_
