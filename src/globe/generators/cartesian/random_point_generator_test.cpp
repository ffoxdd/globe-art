#include <gtest/gtest.h>
#include "random_point_generator.hpp"
#include "../../geometry/cartesian/bounding_box.hpp"
#include "../../math/interval.hpp"
#include "../../testing/assertions/geometric.hpp"
#include "../../testing/assertions/statistical.hpp"
#include "../../testing/macros.hpp"

using namespace globe::generators::cartesian;
using globe::BoundingBox;
using globe::Interval;
using globe::Vector3;
using globe::testing::compute_statistics;
using globe::testing::compute_coordinate_statistics;
using globe::testing::expect_mean;
using globe::testing::expect_variance;
using globe::testing::uniform_distribution_mean;
using globe::testing::uniform_distribution_variance;

TEST(RandomPointGeneratorTest, GenerateWithoutBoundingBoxReturnsPointInUnitCube) {
    RandomPointGenerator generator;

    Vector3 point = generator.generate(1)[0];

    EXPECT_GE(point.x(), -1.0);
    EXPECT_LE(point.x(), 1.0);
    EXPECT_GE(point.y(), -1.0);
    EXPECT_LE(point.y(), 1.0);
    EXPECT_GE(point.z(), -1.0);
    EXPECT_LE(point.z(), 1.0);
}

TEST(RandomPointGeneratorTest, GenerateWithBoundingBoxReturnsPointInBox) {
    RandomPointGenerator generator;
    BoundingBox box(Interval(0.0, 1.0), Interval(0.0, 2.0), Interval(-0.5, 0.5));

    Vector3 point = generator.generate(1, box)[0];

    EXPECT_GE(point.x(), 0.0);
    EXPECT_LE(point.x(), 1.0);
    EXPECT_GE(point.y(), 0.0);
    EXPECT_LE(point.y(), 2.0);
    EXPECT_GE(point.z(), -0.5);
    EXPECT_LE(point.z(), 0.5);
}

TEST(RandomPointGeneratorTest, EXPENSIVE_UniformDistributionInUnitCube) {
    REQUIRE_EXPENSIVE();

    RandomPointGenerator generator;
    constexpr size_t sample_count = 10000;

    auto points = generator.generate(sample_count);
    auto stats = compute_coordinate_statistics(points);

    expect_mean(stats.x, uniform_distribution_mean(-1.0, 1.0), 0.05);
    expect_mean(stats.y, uniform_distribution_mean(-1.0, 1.0), 0.05);
    expect_mean(stats.z, uniform_distribution_mean(-1.0, 1.0), 0.05);

    expect_variance(stats.x, uniform_distribution_variance(-1.0, 1.0), 0.1);
    expect_variance(stats.y, uniform_distribution_variance(-1.0, 1.0), 0.1);
    expect_variance(stats.z, uniform_distribution_variance(-1.0, 1.0), 0.1);
}

TEST(RandomPointGeneratorTest, EXPENSIVE_UniformDistributionInCustomBox) {
    REQUIRE_EXPENSIVE();

    RandomPointGenerator generator;
    BoundingBox box(Interval(0.0, 1.0), Interval(0.0, 2.0), Interval(-0.5, 0.5));
    constexpr size_t sample_count = 10000;

    auto points = generator.generate(sample_count, box);
    auto stats = compute_coordinate_statistics(points);

    expect_mean(stats.x, uniform_distribution_mean(0.0, 1.0), 0.05);
    expect_mean(stats.y, uniform_distribution_mean(0.0, 2.0), 0.05);
    expect_mean(stats.z, uniform_distribution_mean(-0.5, 0.5), 0.05);

    expect_variance(stats.x, uniform_distribution_variance(0.0, 1.0), 0.1);
    expect_variance(stats.y, uniform_distribution_variance(0.0, 2.0), 0.1);
    expect_variance(stats.z, uniform_distribution_variance(-0.5, 0.5), 0.1);
}
