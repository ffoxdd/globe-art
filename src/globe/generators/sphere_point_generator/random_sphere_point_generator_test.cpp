#include <gtest/gtest.h>
#include "random_sphere_point_generator.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../testing/geometric_assertions.hpp"
#include "../../testing/statistical_assertions.hpp"
#include "../../math/interval.hpp"

using namespace globe;
using globe::testing::is_on_unit_sphere;
using globe::testing::compute_statistics;
using globe::testing::compute_coordinate_statistics;
using globe::testing::expect_mean;

TEST(RandomSpherePointGeneratorTest, GenerateWithoutBoundingBoxReturnsPointOnSphere) {
    RandomSpherePointGenerator generator;

    Point3 point = generator.generate(1)[0];

    EXPECT_TRUE(is_on_unit_sphere(point))
        << "Point (" << point.x() << ", " << point.y() << ", " << point.z()
        << ") is not on unit sphere";
}

TEST(RandomSpherePointGeneratorTest, GenerateWithBoundingBoxReturnsPointInBox) {
    RandomSpherePointGenerator generator;
    SphericalBoundingBox box = SphericalBoundingBox::full_sphere();

    Point3 point = generator.generate(1, box)[0];

    EXPECT_TRUE(is_on_unit_sphere(point));
    EXPECT_TRUE(box.contains(point));
}

TEST(RandomSpherePointGeneratorTest, EXPENSIVE_AllPointsOnUnitSphere) {
    REQUIRE_EXPENSIVE();

    RandomSpherePointGenerator generator;
    constexpr size_t sample_count = 10000;

    auto points = generator.generate(sample_count);
    for (const auto& point : points) {
        EXPECT_TRUE(is_on_unit_sphere(point));
    }
}

TEST(RandomSpherePointGeneratorTest, EXPENSIVE_UniformDistributionOnSphere) {
    REQUIRE_EXPENSIVE();

    RandomSpherePointGenerator generator;
    constexpr size_t sample_count = 10000;

    auto points = generator.generate(sample_count);
    auto stats = compute_coordinate_statistics(points);

    expect_mean(stats.x, 0.0, 0.05);
    expect_mean(stats.y, 0.0, 0.05);
    expect_mean(stats.z, 0.0, 0.05);
}

TEST(RandomSpherePointGeneratorTest, EXPENSIVE_BoundedPointsInBox) {
    REQUIRE_EXPENSIVE();

    RandomSpherePointGenerator generator;
    SphericalBoundingBox box(ThetaInterval(0.0, M_PI), Interval(0.0, 1.0));
    constexpr size_t sample_count = 10000;

    auto points = generator.generate(sample_count, box);
    size_t in_box_count = 0;

    for (const auto& point : points) {
        EXPECT_TRUE(is_on_unit_sphere(point));
        EXPECT_TRUE(box.contains(point));

        if (box.contains(point)) {
            in_box_count++;
        }
    }

    EXPECT_EQ(in_box_count, sample_count);
}
