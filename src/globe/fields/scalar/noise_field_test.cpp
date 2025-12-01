#include <gtest/gtest.h>
#include "noise_field.hpp"
#include "../../testing/geometric_assertions.hpp"
#include "../../testing/statistical_assertions.hpp"
#include "../../math/interval.hpp"
#include "../../generators/sphere_point_generator/random_sphere_point_generator.hpp"

using namespace globe::fields::scalar;
using globe::Interval;
using globe::RandomSpherePointGenerator;
using globe::VectorS2;
using globe::testing::compute_statistics;
using globe::testing::expect_range_coverage;

TEST(NoiseFieldTest, ValueMethodReturnsConsistentResult) {
    VectorS2 location = {0.1, 0.2, 0.3};
    NoiseField noise_field;

    double value1 = noise_field.value(location);
    double value2 = noise_field.value(location);

    EXPECT_DOUBLE_EQ(value1, value2);
}

TEST(NoiseFieldTest, ValueMethodDifferentLocations) {
    VectorS2 location1 = {0.1, 0.2, 0.3};
    VectorS2 location2 = {0.4, 0.5, 0.6};
    NoiseField noise_field;

    double value1 = noise_field.value(location1);
    double value2 = noise_field.value(location2);

    EXPECT_NE(value1, value2);
}

constexpr int SAMPLE_COUNT = 200;
constexpr int TEST_SEED_COUNT = 5000;

std::vector<int> test_seeds() {
    std::vector<int> seeds;
    for (int i = 0; i < TEST_SEED_COUNT; ++i) {
        seeds.push_back(i * 137 + 42);
    }
    return seeds;
}

TEST(NoiseFieldTest, EXPENSIVE_CanConfigureOutputRange) {
    REQUIRE_EXPENSIVE();

    Interval output_range(-0.01, 0.02);
    RandomSpherePointGenerator point_generator;

    for (int seed : test_seeds()) {
        NoiseField noise_field(output_range, seed);

        auto points = point_generator.generate(SAMPLE_COUNT);
        std::vector<double> values;
        values.reserve(SAMPLE_COUNT);
        for (const auto& point : points) {
            values.push_back(noise_field.value(point));
        }

        auto metrics = compute_statistics(values);

        EXPECT_GE(metrics.min_value, output_range.low())
            << "Failed for seed " << seed << " (min_value was " << metrics.min_value << ")";

        EXPECT_LE(metrics.max_value, output_range.high())
            << "Failed for seed " << seed << " (max_value was " << metrics.max_value << ")";
    }
}

TEST(NoiseFieldTest, EXPENSIVE_OutputDistributionUsesFullRange) {
    REQUIRE_EXPENSIVE();

    Interval expected_range(0, 1);
    RandomSpherePointGenerator point_generator;

    for (int seed : test_seeds()) {
        NoiseField noise_field(expected_range, seed);

        auto points = point_generator.generate(SAMPLE_COUNT);
        std::vector<double> values;
        values.reserve(SAMPLE_COUNT);
        for (const auto& point : points) {
            values.push_back(noise_field.value(point));
        }

        auto metrics = compute_statistics(values);

        expect_range_coverage(metrics, expected_range, 0.8);
        EXPECT_LT(metrics.clipping_ratio, 0.1) << "Failed for seed " << seed;
    }
}
