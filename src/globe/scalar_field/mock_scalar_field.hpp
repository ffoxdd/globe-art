#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_SCALAR_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_SCALAR_FIELD_HPP_

#include "../scalar_field/scalar_field.hpp"
#include <gmock/gmock.h>
#include <vector>

namespace globe {

class MockScalarField {
 public:
    MOCK_METHOD(double, value, (const Point3&), (const));
    MOCK_METHOD(void, normalize, (std::vector<Point3> & , Interval), ());
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_SCALAR_FIELD_HPP_
