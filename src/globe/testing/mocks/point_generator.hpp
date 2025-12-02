#ifndef GLOBEART_SRC_GLOBE_TESTING_MOCKS_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_MOCKS_POINT_GENERATOR_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/bounding_box.hpp"
#include <vector>
#include <utility>

namespace globe::testing::mocks {

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

} // namespace globe::testing::mocks

#endif //GLOBEART_SRC_GLOBE_TESTING_MOCKS_POINT_GENERATOR_HPP_
