#ifndef GLOBEART_SRC_GLOBE_MATH_INTERVAL_SAMPLER_INTERVAL_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_MATH_INTERVAL_SAMPLER_INTERVAL_SAMPLER_HPP_

#include "../interval.hpp"
#include "uniform_interval_sampler.hpp"
#include <concepts>

namespace globe {

template<typename T>
concept IntervalSampler = requires(T sampler, const Interval& interval) {
    { sampler.sample(interval) } -> std::convertible_to<double>;
};

}

#endif //GLOBEART_SRC_GLOBE_MATH_INTERVAL_SAMPLER_INTERVAL_SAMPLER_HPP_
