#include <gtest/gtest.h>
#include "voronoi_sphere_factory.hpp"
#include "../../testing/geometric_assertions.hpp"

using namespace globe;

TEST(VoronoiSphereFactoryTest, EXPENSIVE_BuildWithConstantDensity) {
    REQUIRE_EXPENSIVE();

    VoronoiSphereFactory factory(5, "constant", 1);

    auto sphere = factory.build();

    EXPECT_EQ(sphere->size(), 5);
}

TEST(VoronoiSphereFactoryTest, EXPENSIVE_BuildWithNoiseDensity) {
    REQUIRE_EXPENSIVE();

    VoronoiSphereFactory factory(5, "noise", 1);

    auto sphere = factory.build();

    EXPECT_EQ(sphere->size(), 5);
}
