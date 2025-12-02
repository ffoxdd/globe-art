#ifndef GLOBEART_SRC_GLOBE_TESTING_MOCKS_CIRCULAR_INTERVAL_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_MOCKS_CIRCULAR_INTERVAL_SAMPLER_HPP_

#include "../../math/circular_interval.hpp"
#include <vector>
#include <utility>

namespace globe::testing::mocks {

class MockCircularIntervalSampler {
 public:
    explicit MockCircularIntervalSampler(std::vector<double> sequence)
        : _sequence(std::move(sequence)), _index(0) {}

    template<double PERIOD>
    [[nodiscard]] double sample(const CircularInterval<PERIOD>& interval) {
        double t = _sequence[_index % _sequence.size()];
        _index++;
        return interval.start() + t * interval.measure();
    }

 private:
    std::vector<double> _sequence;
    size_t _index;
};

} // namespace globe::testing::mocks

#endif //GLOBEART_SRC_GLOBE_TESTING_MOCKS_CIRCULAR_INTERVAL_SAMPLER_HPP_
