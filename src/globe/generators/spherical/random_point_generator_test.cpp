#include <gtest/gtest.h>
#include "random_point_generator.hpp"
#include "../../geometry/spherical/bounding_box.hpp"
#include "../../testing/assertions/geometric.hpp"
#include "../../testing/assertions/statistical.hpp"
#include "../../testing/macros.hpp"
#include "../../math/interval.hpp"

using namespace globe::generators::spherical;
using globe::SphericalBoundingBox;
using globe::ThetaInterval;
using globe::Interval;
using globe::VectorS2;
using globe::testing::is_on_unit_sphere;
using globe::testing::compute_statistics;
using globe::testing::compute_coordinate_statistics;
using globe::testing::expect_mean;

TEST(RandomPointGeneratorTest, GenerateWithoutBoundingBoxReturnsPointOnSphere) {
    RandomPointGenerator generator;

    VectorS2 point = generator.generate(1)[0];

    EXPECT_NEAR(point.norm(), 1.0, 1e-9)
        << "Point (" << point.x() << ", " << point.y() << ", " << point.z()
        << ") is not on unit sphere";
}

TEST(RandomPointGeneratorTest, GenerateWithBoundingBoxReturnsPointInBox) {
    RandomPointGenerator generator;
    SphericalBoundingBox box = SphericalBoundingBox::full_sphere();

    VectorS2 point = generator.generate(1, box)[0];

    EXPECT_NEAR(point.norm(), 1.0, 1e-9);
    EXPECT_TRUE(box.contains(point));
}

TEST(RandomPointGeneratorTest, EXPENSIVE_AllPointsOnUnitSphere) {
    REQUIRE_EXPENSIVE();

    RandomPointGenerator generator;
    constexpr size_t sample_count = 10000;

    auto points = generator.generate(sample_count);
    for (const auto& point : points) {
        EXPECT_NEAR(point.norm(), 1.0, 1e-9);
    }
}

TEST(RandomPointGeneratorTest, EXPENSIVE_UniformDistributionOnSphere) {
    REQUIRE_EXPENSIVE();

    RandomPointGenerator generator;
    constexpr size_t sample_count = 10000;

    auto points = generator.generate(sample_count);
    auto stats = compute_coordinate_statistics(points);

    expect_mean(stats.x, 0.0, 0.05);
    expect_mean(stats.y, 0.0, 0.05);
    expect_mean(stats.z, 0.0, 0.05);
}

TEST(RandomPointGeneratorTest, EXPENSIVE_BoundedPointsInBox) {
    REQUIRE_EXPENSIVE();

    RandomPointGenerator generator;
    SphericalBoundingBox box(ThetaInterval(0.0, M_PI), Interval(0.0, 1.0));
    constexpr size_t sample_count = 10000;

    auto points = generator.generate(sample_count, box);
    size_t in_box_count = 0;

    for (const auto& point : points) {
        EXPECT_NEAR(point.norm(), 1.0, 1e-9);
        EXPECT_TRUE(box.contains(point));

        if (box.contains(point)) {
            in_box_count++;
        }
    }

    EXPECT_EQ(in_box_count, sample_count);
}
