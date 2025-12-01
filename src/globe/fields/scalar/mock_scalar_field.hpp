#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_SCALAR_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_SCALAR_FIELD_HPP_

#include "scalar_field.hpp"
#include <gmock/gmock.h>

namespace globe {

class MockScalarField {
 public:
    MOCK_METHOD(double, value, (const VectorS2&), (const));
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_MOCK_SCALAR_FIELD_HPP_
