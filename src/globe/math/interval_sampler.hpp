#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_INTERVAL_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_INTERVAL_SAMPLER_HPP_

#include "interval.hpp"
#include <random>

namespace globe {

class IntervalSampler {
 public:
    IntervalSampler() : _random_engine(std::random_device{}()) {
    }

    explicit IntervalSampler(unsigned int seed) : _random_engine(seed) {
    }

    [[nodiscard]] inline double sample(const Interval& interval) {
        return distribution(interval)(_random_engine);
    }

 private:
    std::mt19937 _random_engine;

    static inline std::uniform_real_distribution<double> distribution(const Interval& interval) {
        return std::uniform_real_distribution<>(interval.low(), interval.high());
    }
};

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_INTERVAL_SAMPLER_HPP_
