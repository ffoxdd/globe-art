#ifndef GLOBEART_SRC_GLOBE_MOCK_POINT_GENERATOR_POINT_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_MOCK_POINT_GENERATOR_POINT_GENERATOR_H_

#include "../types.hpp"
#include <gmock/gmock.h>

namespace globe {

class MockPointGenerator {
 public:
    MOCK_METHOD(Point3, generate, ());
};

}

#endif //GLOBEART_SRC_GLOBE_MOCK_POINT_GENERATOR_POINT_GENERATOR_H_
