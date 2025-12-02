#include <gtest/gtest.h>
#include "factory.hpp"
#include "../../../testing/assertions/geometric.hpp"
#include "../../../testing/macros.hpp"

using namespace globe::voronoi::spherical;

TEST(FactoryTest, EXPENSIVE_BuildWithConstantDensity) {
    REQUIRE_EXPENSIVE();

    Factory factory(5, "constant", "ccvd", 1, 0);

    auto sphere = factory.build();

    EXPECT_EQ(sphere->size(), 5);
}

TEST(FactoryTest, EXPENSIVE_BuildWithNoiseDensity) {
    REQUIRE_EXPENSIVE();

    Factory factory(5, "noise", "ccvd", 1, 0);

    auto sphere = factory.build();

    EXPECT_EQ(sphere->size(), 5);
}

TEST(FactoryTest, EXPENSIVE_BuildWithGradientOptimization) {
    REQUIRE_EXPENSIVE();

    Factory factory(5, "constant", "gradient", 10, 0);

    auto sphere = factory.build();

    EXPECT_EQ(sphere->size(), 5);
}
