#ifndef GLOBEART_SRC_GLOBE_MATH_CIRCULAR_INTERVAL_SAMPLER_CIRCULAR_INTERVAL_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_MATH_CIRCULAR_INTERVAL_SAMPLER_CIRCULAR_INTERVAL_SAMPLER_HPP_

#include "../circular_interval.hpp"
#include <concepts>

namespace globe::math {

template<typename T, double PERIOD>
concept CircularIntervalSampler = requires(T sampler, const CircularInterval<PERIOD>& interval) {
    { sampler.sample(interval) } -> std::convertible_to<double>;
};

} // namespace globe::math

namespace globe {
using math::CircularIntervalSampler;
}

#endif //GLOBEART_SRC_GLOBE_MATH_CIRCULAR_INTERVAL_SAMPLER_CIRCULAR_INTERVAL_SAMPLER_HPP_
