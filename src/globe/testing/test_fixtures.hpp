#ifndef GLOBEART_SRC_GLOBE_TESTING_TEST_FIXTURES_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_TEST_FIXTURES_HPP_

#include "../types.hpp"
#include "../math/interval.hpp"
#include "../math/circular_interval.hpp"
#include "../geometry/spherical/spherical_bounding_box.hpp"
#include <vector>
#include <utility>

namespace globe::testing {

class SequencePointGenerator {
 public:
    explicit SequencePointGenerator(std::vector<VectorS2> sequence)
        : _sequence(std::move(sequence)), _index(0), _last_attempt_count(0) {}

    std::vector<VectorS2> generate(size_t count) {
        std::vector<VectorS2> result;
        result.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            result.push_back(_sequence[_index % _sequence.size()]);
            _index++;
        }
        _last_attempt_count = count;
        return result;
    }

    std::vector<VectorS2> generate(size_t count, const SphericalBoundingBox&) {
        return generate(count);
    }

    [[nodiscard]] size_t last_attempt_count() const { return _last_attempt_count; }

 private:
    std::vector<VectorS2> _sequence;
    size_t _index;
    size_t _last_attempt_count;
};

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

class SequenceCircularIntervalSampler {
 public:
    explicit SequenceCircularIntervalSampler(std::vector<double> sequence)
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

}

#endif //GLOBEART_SRC_GLOBE_TESTING_TEST_FIXTURES_HPP_
