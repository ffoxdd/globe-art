#include <gtest/gtest.h>
#include "noise_field.hpp"
#include <random>
#include <functional>
#include <limits>
#include <cstdlib>

using namespace globe;

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

class RandomPointGenerator {
 public:
    RandomPointGenerator() : _random_engine(std::random_device{}()) {
    }

    Point3 generate() {
        return Point3(
            random_coordinate(),
            random_coordinate(),
            random_coordinate()
        );
    }

 private:
    double random_coordinate() {
        return _distribution(_random_engine);
    }

    std::mt19937 _random_engine;
    std::uniform_real_distribution<> _distribution{-1.0, 1.0};
};

struct DistributionMetrics {
    double min_value;
    double max_value;
    int values_at_min;
    int values_at_max;
    double range_coverage;
    double clipping_ratio;
};

DistributionMetrics measure_distribution(
    std::function<double(const Point3&)> field_func,
    Interval expected_range,
    int sample_count
) {
    RandomPointGenerator point_generator;

    double min_value = std::numeric_limits<double>::max();
    double max_value = std::numeric_limits<double>::lowest();

    int values_at_min = 0;
    int values_at_max = 0;

    for (int i = 0; i < sample_count; ++i) {
        Point3 point = point_generator.generate();
        double value = field_func(point);

        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);

        if (std::abs(value - expected_range.low()) < 1e-10) {
            values_at_min++;
        }

        if (std::abs(value - expected_range.high()) < 1e-10) {
            values_at_max++;
        }
    }

    double range_coverage = (max_value - min_value) / expected_range.measure();
    double clipping_ratio = static_cast<double>(values_at_min + values_at_max) / sample_count;

    return {
        min_value,
        max_value,
        values_at_min,
        values_at_max,
        range_coverage,
        clipping_ratio,
    };
}

TEST(NoiseFieldTest, CanConfigureOutputRange) {
    require_expensive_tests();

    Interval output_range = Interval(-0.01, 0.02);

    for (int seed : test_seeds()) {
        NoiseField noise_field(output_range, seed);
        Interval expected_range = noise_field.output_range();

        auto metrics = measure_distribution(
            [&](const Point3& p) { return noise_field.value(p); },
            expected_range,
            SAMPLE_COUNT
        );

        EXPECT_GE(metrics.min_value, output_range.low())
            << "Failed for seed " << seed << " (min_value was " << metrics.min_value << ")";

        EXPECT_LE(metrics.max_value, output_range.high())
            << "Failed for seed " << seed << " (max_value was " << metrics.max_value << ")";
    }
}

TEST(NoiseFieldTest, OutputDistributionUsesFullRange) {
    require_expensive_tests();

    for (int seed : test_seeds()) {
        NoiseField noise_field(Interval(0, 1), seed);
        Interval expected_range = noise_field.output_range();

        auto metrics = measure_distribution(
            [&](const Point3& p) { return noise_field.value(p); },
            expected_range,
            SAMPLE_COUNT
        );

        EXPECT_GT(metrics.range_coverage, 0.8) << "Failed for seed " << seed;
        EXPECT_LT(metrics.clipping_ratio, 0.1) << "Failed for seed " << seed;
    }
}

