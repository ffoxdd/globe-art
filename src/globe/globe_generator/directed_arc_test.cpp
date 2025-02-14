#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./directed_arc.hpp"
#include <cmath>

using namespace globe;
using ::testing::Return;

TEST(DirectedArcTest, Canonicalized) {
    const auto arc = CircularArc3(
        SphericalCircle3(
            SphericalPoint3(0, 0, 0),
            1.0,
            SphericalVector3(1, 0, 0)
        ),
        SphericalPoint3(0, 1, 0),
        SphericalPoint3(0, 0, 1)
    );

    const auto short_arc = DirectedArc(arc);
    EXPECT_DOUBLE_EQ(arc.approximate_angle(), M_PI_2);
}

TEST(DirectedArcTest, NonCanonicalized) {
    const auto arc = CircularArc3(
        SphericalCircle3(
            SphericalPoint3(0, 0, 0),
            1.0,
            SphericalVector3(-1, 0, 0)
        ),
        SphericalPoint3(0, 1, 0),
        SphericalPoint3(0, 0, 1)
    );

    const auto long_arc = DirectedArc(arc);
    EXPECT_DOUBLE_EQ(long_arc.approximate_angle(), (3.0 / 2.0) * M_PI);
}
