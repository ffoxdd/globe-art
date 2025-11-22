#include <gtest/gtest.h>
#include "random_point_generator.hpp"
#include "../math/bounding_box.hpp"
#include "../math/interval.hpp"
#include "../testing/geometric_assertions.hpp"
#include "../testing/statistical_assertions.hpp"

using namespace globe;
using globe::testing::compute_statistics;
using globe::testing::expect_uniform_distribution_mean;
using globe::testing::expect_uniform_distribution_variance;
using globe::testing::uniform_distribution_mean;
using globe::testing::uniform_distribution_variance;

TEST(RandomPointGeneratorTest, GenerateWithoutBoundingBoxReturnsPointInUnitCube) {
    RandomPointGenerator generator;

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate();

        EXPECT_GE(point.x(), -1.0);
        EXPECT_LE(point.x(), 1.0);
        EXPECT_GE(point.y(), -1.0);
        EXPECT_LE(point.y(), 1.0);
        EXPECT_GE(point.z(), -1.0);
        EXPECT_LE(point.z(), 1.0);
    }
}

TEST(RandomPointGeneratorTest, GenerateWithBoundingBoxReturnsPointInBox) {
    RandomPointGenerator generator;
    BoundingBox box(Interval(0.0, 1.0), Interval(0.0, 2.0), Interval(-0.5, 0.5));

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate(box);

        EXPECT_GE(point.x(), 0.0);
        EXPECT_LE(point.x(), 1.0);
        EXPECT_GE(point.y(), 0.0);
        EXPECT_LE(point.y(), 2.0);
        EXPECT_GE(point.z(), -0.5);
        EXPECT_LE(point.z(), 0.5);
    }
}

TEST(RandomPointGeneratorTest, EXPENSIVE_UniformDistributionInUnitCube) {
    SKIP_IF_EXPENSIVE();

    RandomPointGenerator generator;
    constexpr size_t sample_count = 10000;

    auto x_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate();
            EXPECT_GE(point.x(), -1.0);
            EXPECT_LE(point.x(), 1.0);
            return point.x();
        },
        sample_count
    );

    auto y_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate();
            EXPECT_GE(point.y(), -1.0);
            EXPECT_LE(point.y(), 1.0);
            return point.y();
        },
        sample_count
    );

    auto z_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate();
            EXPECT_GE(point.z(), -1.0);
            EXPECT_LE(point.z(), 1.0);
            return point.z();
        },
        sample_count
    );

    expect_uniform_distribution_mean(x_stats, uniform_distribution_mean(-1.0, 1.0), 0.05);
    expect_uniform_distribution_mean(y_stats, uniform_distribution_mean(-1.0, 1.0), 0.05);
    expect_uniform_distribution_mean(z_stats, uniform_distribution_mean(-1.0, 1.0), 0.05);

    expect_uniform_distribution_variance(x_stats, uniform_distribution_variance(-1.0, 1.0), 0.1);
    expect_uniform_distribution_variance(y_stats, uniform_distribution_variance(-1.0, 1.0), 0.1);
    expect_uniform_distribution_variance(z_stats, uniform_distribution_variance(-1.0, 1.0), 0.1);
}

TEST(RandomPointGeneratorTest, EXPENSIVE_UniformDistributionInCustomBox) {
    SKIP_IF_EXPENSIVE();

    RandomPointGenerator generator;
    BoundingBox box(Interval(0.0, 1.0), Interval(0.0, 2.0), Interval(-0.5, 0.5));
    constexpr size_t sample_count = 10000;

    auto x_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate(box);
            EXPECT_GE(point.x(), 0.0);
            EXPECT_LE(point.x(), 1.0);
            return point.x();
        },
        sample_count
    );

    auto y_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate(box);
            EXPECT_GE(point.y(), 0.0);
            EXPECT_LE(point.y(), 2.0);
            return point.y();
        },
        sample_count
    );

    auto z_stats = compute_statistics(
        [&]() {
            Point3 point = generator.generate(box);
            EXPECT_GE(point.z(), -0.5);
            EXPECT_LE(point.z(), 0.5);
            return point.z();
        },
        sample_count
    );

    expect_uniform_distribution_mean(x_stats, uniform_distribution_mean(0.0, 1.0), 0.05);
    expect_uniform_distribution_mean(y_stats, uniform_distribution_mean(0.0, 2.0), 0.05);
    expect_uniform_distribution_mean(z_stats, uniform_distribution_mean(-0.5, 0.5), 0.05);

    expect_uniform_distribution_variance(x_stats, uniform_distribution_variance(0.0, 1.0), 0.1);
    expect_uniform_distribution_variance(y_stats, uniform_distribution_variance(0.0, 2.0), 0.1);
    expect_uniform_distribution_variance(z_stats, uniform_distribution_variance(-0.5, 0.5), 0.1);
}
