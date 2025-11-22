#include <gtest/gtest.h>
#include "random_sphere_point_generator.hpp"
#include "../spherical/spherical_bounding_box.hpp"
#include "../testing/geometric_assertions.hpp"
#include "../testing/statistical_assertions.hpp"
#include "../math/interval.hpp"

using namespace globe;
using globe::testing::is_on_unit_sphere;
using globe::testing::compute_statistics;
using globe::testing::expect_uniform_distribution_mean;

TEST(RandomSpherePointGeneratorTest, GenerateWithoutBoundingBoxReturnsPointOnSphere) {
    RandomSpherePointGenerator generator;

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate();

        EXPECT_TRUE(is_on_unit_sphere(point))
            << "Point (" << point.x() << ", " << point.y() << ", " << point.z()
            << ") is not on unit sphere";
    }
}

TEST(RandomSpherePointGeneratorTest, GenerateWithBoundingBoxReturnsPointInBox) {
    RandomSpherePointGenerator generator;
    SphericalBoundingBox box = SphericalBoundingBox::full_sphere();

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate(box);

        EXPECT_TRUE(is_on_unit_sphere(point));
        EXPECT_TRUE(box.contains(point));
    }
}

TEST(RandomSpherePointGeneratorTest, EXPENSIVE_AllPointsOnUnitSphere) {
    SKIP_IF_EXPENSIVE();

    RandomSpherePointGenerator generator;
    constexpr size_t sample_count = 10000;

    for (size_t i = 0; i < sample_count; ++i) {
        Point3 point = generator.generate();
        EXPECT_TRUE(is_on_unit_sphere(point));
    }
}

TEST(RandomSpherePointGeneratorTest, EXPENSIVE_UniformDistributionOnSphere) {
    SKIP_IF_EXPENSIVE();

    RandomSpherePointGenerator generator;
    constexpr size_t sample_count = 10000;

    auto x_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate();
            EXPECT_TRUE(is_on_unit_sphere(point));
            return point.x();
        },
        sample_count
    );

    auto y_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate();
            EXPECT_TRUE(is_on_unit_sphere(point));
            return point.y();
        },
        sample_count
    );

    auto z_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate();
            EXPECT_TRUE(is_on_unit_sphere(point));
            return point.z();
        },
        sample_count
    );

    expect_uniform_distribution_mean(x_stats, 0.0, 0.05);
    expect_uniform_distribution_mean(y_stats, 0.0, 0.05);
    expect_uniform_distribution_mean(z_stats, 0.0, 0.05);
}

TEST(RandomSpherePointGeneratorTest, EXPENSIVE_BoundedPointsInBox) {
    SKIP_IF_EXPENSIVE();

    RandomSpherePointGenerator generator;
    SphericalBoundingBox box(Interval(0.0, M_PI), Interval(0.0, 1.0));
    constexpr size_t sample_count = 10000;

    size_t in_box_count = 0;

    for (size_t i = 0; i < sample_count; ++i) {
        Point3 point = generator.generate(box);

        EXPECT_TRUE(is_on_unit_sphere(point));
        EXPECT_TRUE(box.contains(point));

        if (box.contains(point)) {
            in_box_count++;
        }
    }

    EXPECT_EQ(in_box_count, sample_count);
}
