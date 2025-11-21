#ifndef GLOBEART_SRC_GLOBE_GENERATORS_MOCK_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_MOCK_POINT_GENERATOR_HPP_

#include "../types.hpp"
#include "../spherical/spherical_bounding_box.hpp"
#include <gmock/gmock.h>

namespace globe {

class MockPointGenerator {
 public:
    MOCK_METHOD(Point3, generate, ());
    MOCK_METHOD(Point3, generate, (const SphericalBoundingBox &bbox));
};

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_MOCK_POINT_GENERATOR_HPP_

