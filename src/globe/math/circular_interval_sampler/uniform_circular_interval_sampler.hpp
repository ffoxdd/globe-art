#ifndef GLOBEART_SRC_GLOBE_MATH_CIRCULAR_INTERVAL_SAMPLER_UNIFORM_CIRCULAR_INTERVAL_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_MATH_CIRCULAR_INTERVAL_SAMPLER_UNIFORM_CIRCULAR_INTERVAL_SAMPLER_HPP_

#include "../circular_interval.hpp"
#include <random>

namespace globe::math {

class UniformCircularIntervalSampler {
 public:
    UniformCircularIntervalSampler() : _random_engine(std::random_device{}()) {}

    explicit UniformCircularIntervalSampler(unsigned int seed) : _random_engine(seed) {}

    template<double PERIOD>
    [[nodiscard]] inline double sample(const CircularInterval<PERIOD>& interval) {
        double u = _distribution(_random_engine);
        return interval.start() + u * interval.measure();
    }

 private:
    std::mt19937 _random_engine;
    std::uniform_real_distribution<double> _distribution{0.0, 1.0};
};

} // namespace globe::math

namespace globe {
using UniformCircularIntervalSampler = math::UniformCircularIntervalSampler;
}

#endif //GLOBEART_SRC_GLOBE_MATH_CIRCULAR_INTERVAL_SAMPLER_UNIFORM_CIRCULAR_INTERVAL_SAMPLER_HPP_
