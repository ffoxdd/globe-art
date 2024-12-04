#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./directed_arc.hpp"
#include <cmath>

using namespace globe;
using ::testing::Return;

TEST(DirectedArcTest, Canonicalized) {
    CircularArc3 arc = CircularArc3(
        SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0)),
        SphericalPoint3(0, 1, 0),
        SphericalPoint3(0, 0, 1)
    );

    DirectedArc short_arc = DirectedArc(arc, false);
    EXPECT_DOUBLE_EQ(short_arc.approximate_angle(), M_PI_2);

    DirectedArc long_arc = DirectedArc(arc, true);
    EXPECT_DOUBLE_EQ(long_arc.approximate_angle(), (3.0 / 2.0) * M_PI);
}

TEST(DirectedArcTest, NonCanonicalized) {
    CircularArc3 arc = CircularArc3(
        SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(-1, 0, 0)),
        SphericalPoint3(0, 1, 0),
        SphericalPoint3(0, 0, 1)
    );

    // CANONICALIZATION DOES NOT HAPPEN WHERE YOU THINK!

    DirectedArc short_arc = DirectedArc(arc, true);
    EXPECT_DOUBLE_EQ(short_arc.approximate_angle(), M_PI_2);

    DirectedArc long_arc = DirectedArc(arc, false);
    EXPECT_DOUBLE_EQ(long_arc.approximate_angle(), (3.0 / 2.0) * M_PI);
}