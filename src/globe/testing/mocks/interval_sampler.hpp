#ifndef GLOBEART_SRC_GLOBE_TESTING_MOCKS_INTERVAL_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_MOCKS_INTERVAL_SAMPLER_HPP_

#include "../../math/interval.hpp"
#include <vector>
#include <utility>

namespace globe::testing::mocks {

class SequenceIntervalSampler {
 public:
    explicit SequenceIntervalSampler(std::vector<double> sequence)
        : _sequence(std::move(sequence)), _index(0) {}

    [[nodiscard]] double sample(const Interval& interval) {
        double t = _sequence[_index % _sequence.size()];
        _index++;
        return interval.low() + t * (interval.high() - interval.low());
    }

 private:
    std::vector<double> _sequence;
    size_t _index;
};

} // namespace globe::testing::mocks

#endif //GLOBEART_SRC_GLOBE_TESTING_MOCKS_INTERVAL_SAMPLER_HPP_
