#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./spherical_polygon.hpp"

#include <iostream>

using namespace globe;
using ::testing::Return;

TEST(SphericalPolygonTest, Hemispheres) {
    SphericalCircle3 circle(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0));

    SphericalPolygon spherical_polygon = SphericalPolygon(
        std::vector<Arc>{
//            Arc(circle, SphericalPoint3(-1, 0, 0), SphericalPoint3(1, 0, 0)),
//            Arc(circle, SphericalPoint3(1, 0, 0), SphericalPoint3(-1, 0, 0)),

            Arc(circle, SphericalPoint3(-1, 0, 0), SphericalPoint3(0, -1, 0)),
            Arc(circle, SphericalPoint3(0, -1, 0), SphericalPoint3(1, 0, 0)),
            Arc(circle, SphericalPoint3(1, 0, 0), SphericalPoint3(0, 1, 0)),
            Arc(circle, SphericalPoint3(0, 1, 0), SphericalPoint3(-1, 0, 0)),
        }
    );

//    EXPECT_TRUE(spherical_polygon.contains(Point3(0, 0, 1)));
    EXPECT_FALSE(spherical_polygon.contains(Point3(0, 0, -1)));
}

