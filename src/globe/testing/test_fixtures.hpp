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
    explicit SequencePointGenerator(std::vector<Point3> sequence)
        : _sequence(std::move(sequence)), _index(0) {}

    Point3 generate() {
        Point3 result = _sequence[_index % _sequence.size()];
        _index++;
        return result;
    }

    Point3 generate(const SphericalBoundingBox&) {
        return generate();
    }

 private:
    std::vector<Point3> _sequence;
    size_t _index;
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
