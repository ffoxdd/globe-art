#include <gtest/gtest.h>
#include "random_voronoi_sphere_builder.hpp"
#include "../../testing/test_fixtures.hpp"

using namespace globe;
using globe::testing::SequencePointGenerator;

TEST(RandomVoronoiSphereBuilderTest, CreatesSphereWithCorrectSize) {
    RandomVoronoiSphereBuilder builder;

    auto sphere = builder.build(10);
    EXPECT_EQ(sphere->size(), 10);
}

TEST(RandomVoronoiSphereBuilderTest, UsesProvidedGenerator) {
    Point3 fixed_point(1, 0, 0);
    SequencePointGenerator generator({fixed_point});
    RandomVoronoiSphereBuilder<SequencePointGenerator> builder(generator);

    auto sphere = builder.build(1);

    EXPECT_EQ(sphere->size(), 1);

    Point3 site = sphere->site(0);
    EXPECT_DOUBLE_EQ(site.x(), fixed_point.x());
    EXPECT_DOUBLE_EQ(site.y(), fixed_point.y());
    EXPECT_DOUBLE_EQ(site.z(), fixed_point.z());
}

