#ifndef GLOBEART_SRC_GLOBE_TESTING_TEST_FIXTURES_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_TEST_FIXTURES_HPP_

#include "../types.hpp"
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

}

#endif //GLOBEART_SRC_GLOBE_TESTING_TEST_FIXTURES_HPP_
