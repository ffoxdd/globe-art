#include <gtest/gtest.h>
#include "random_voronoi_sphere_builder.hpp"

using namespace globe;

TEST(RandomVoronoiSphereBuilderTest, CreatesSphereWithCorrectSize) {
    RandomVoronoiSphereBuilder builder;

    VoronoiSphere sphere = builder.build(10);
    EXPECT_EQ(sphere.size(), 10);
}

