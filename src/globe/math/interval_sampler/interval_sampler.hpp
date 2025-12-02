#ifndef GLOBEART_SRC_GLOBE_MATH_INTERVAL_SAMPLER_INTERVAL_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_MATH_INTERVAL_SAMPLER_INTERVAL_SAMPLER_HPP_

#include "../interval.hpp"
#include <concepts>

namespace globe::math {

template<typename T>
concept IntervalSampler = requires(T sampler, const Interval& interval) {
    { sampler.sample(interval) } -> std::convertible_to<double>;
};

} // namespace globe::math

namespace globe {
using math::IntervalSampler;
}

#endif //GLOBEART_SRC_GLOBE_MATH_INTERVAL_SAMPLER_INTERVAL_SAMPLER_HPP_
