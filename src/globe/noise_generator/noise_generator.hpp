#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_

#include "../types.hpp"

namespace globe {

template<typename T>
concept NoiseGenerator = requires(T noise_generator, const Point3 &location) {
    { noise_generator.value(location) } -> std::convertible_to<double>;
};

}

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_
