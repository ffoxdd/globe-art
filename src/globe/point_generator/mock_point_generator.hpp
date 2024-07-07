#ifndef GLOBEART_SRC_GLOBE_MOCK_POINT_GENERATOR_POINT_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_MOCK_POINT_GENERATOR_POINT_GENERATOR_H_

#include "../types.hpp"
#include "point_generator.hpp"
#include <gmock/gmock.h>

namespace globe {

class MockPointGenerator : public PointGenerator {
 public:
    MOCK_METHOD(Point3, generate, (), (override));
};

}

#endif //GLOBEART_SRC_GLOBE_MOCK_POINT_GENERATOR_POINT_GENERATOR_H_
