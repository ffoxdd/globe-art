#include <gtest/gtest.h>
#include "noise_field.hpp"
#include "../../testing/statistical_assertions.hpp"
#include "../../math/interval.hpp"
#include "../../generators/random_point_generator.hpp"
#include <cstdlib>

using namespace globe;
using globe::testing::compute_statistics_with_boundary_check;
using globe::testing::compute_range_coverage;
using globe::testing::compute_clipping_ratio;

TEST(NoiseFieldTest, ValueMethodReturnsConsistentResult) {
    Point3 location = {0.1, 0.2, 0.3};
    NoiseField noise_field;

    double value1 = noise_field.value(location);
    double value2 = noise_field.value(location);

    EXPECT_DOUBLE_EQ(value1, value2);
}

TEST(NoiseFieldTest, ValueMethodDifferentLocations) {
    Point3 location1 = {0.1, 0.2, 0.3};
    Point3 location2 = {0.4, 0.5, 0.6};
    NoiseField noise_field;

    double value1 = noise_field.value(location1);
    double value2 = noise_field.value(location2);

    EXPECT_NE(value1, value2);
}

constexpr int SAMPLE_COUNT = 200;
constexpr int TEST_SEED_COUNT = 5000;

void require_expensive_tests() {
    if (!std::getenv("EXPENSIVE")) {
        GTEST_SKIP() << "Skipping expensive test. Set EXPENSIVE=1 to enable.";
    }
}

std::vector<int> test_seeds() {
    std::vector<int> seeds;
    for (int i = 0; i < TEST_SEED_COUNT; ++i) {
        seeds.push_back(i * 137 + 42);
    }
    return seeds;
}

TEST(NoiseFieldTest, CanConfigureOutputRange) {
    require_expensive_tests();

    Interval output_range = Interval(-0.01, 0.02);
    RandomPointGenerator point_generator;

    for (int seed : test_seeds()) {
        NoiseField noise_field(output_range, seed);
        Interval expected_range = noise_field.output_range();

        auto metrics = compute_statistics_with_boundary_check(
            [&]() {
                Point3 point = point_generator.generate();
                return noise_field.value(point);
            },
            expected_range,
            SAMPLE_COUNT
        );

        EXPECT_GE(metrics.stats.min_value, output_range.low())
            << "Failed for seed " << seed << " (min_value was " << metrics.stats.min_value << ")";

        EXPECT_LE(metrics.stats.max_value, output_range.high())
            << "Failed for seed " << seed << " (max_value was " << metrics.stats.max_value << ")";
    }
}

TEST(NoiseFieldTest, OutputDistributionUsesFullRange) {
    require_expensive_tests();

    RandomPointGenerator point_generator;

    for (int seed : test_seeds()) {
        NoiseField noise_field(Interval(0, 1), seed);
        Interval expected_range = noise_field.output_range();

        auto metrics = compute_statistics_with_boundary_check(
            [&]() {
                Point3 point = point_generator.generate();
                return noise_field.value(point);
            },
            expected_range,
            SAMPLE_COUNT
        );

        double range_coverage = compute_range_coverage(metrics.stats, expected_range);
        double clipping_ratio = compute_clipping_ratio(metrics);

        EXPECT_GT(range_coverage, 0.8) << "Failed for seed " << seed;
        EXPECT_LT(clipping_ratio, 0.1) << "Failed for seed " << seed;
    }
}

