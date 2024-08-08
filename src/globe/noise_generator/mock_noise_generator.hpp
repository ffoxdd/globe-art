#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_NOISE_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_NOISE_GENERATOR_HPP_

#include "../noise_generator/noise_generator.hpp"
#include <gmock/gmock.h>
#include "../types.hpp"
#include <vector>

namespace globe {

class MockNoiseGenerator {
 public:
    MOCK_METHOD(double, value, (const Point3&), (const));
    MOCK_METHOD(void, normalize, (std::vector<Point3> & , Interval), ());
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_NOISE_GENERATOR_HPP_
