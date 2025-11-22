#ifndef GLOBEART_SRC_GLOBE_TESTING_STATISTICAL_ASSERTIONS_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_STATISTICAL_ASSERTIONS_HPP_

#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include "../math/interval.hpp"

namespace globe::testing {

struct DistributionStatistics {
    double mean;
    double variance;
    double min_value;
    double max_value;
    size_t sample_count;
};

template<typename SampleFunc>
DistributionStatistics compute_statistics(SampleFunc sample_func, size_t sample_count) {
    std::vector<double> samples;
    samples.reserve(sample_count);

    for (size_t i = 0; i < sample_count; ++i) {
        samples.push_back(sample_func());
    }

    double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    double mean = sum / sample_count;

    double variance_sum = 0.0;
    for (double value : samples) {
        double diff = value - mean;
        variance_sum += diff * diff;
    }
    double variance = variance_sum / sample_count;

    auto [min_it, max_it] = std::minmax_element(samples.begin(), samples.end());

    return {
        mean,
        variance,
        *min_it,
        *max_it,
        sample_count
    };
}

struct DistributionMetrics {
    DistributionStatistics stats;
    size_t values_at_min;
    size_t values_at_max;
};

template<typename SampleFunc>
DistributionMetrics compute_statistics_with_boundary_check(
    SampleFunc sample_func,
    const Interval& expected_range,
    size_t sample_count,
    double epsilon = 1e-10
) {
    std::vector<double> samples;
    samples.reserve(sample_count);

    size_t values_at_min = 0;
    size_t values_at_max = 0;

    for (size_t i = 0; i < sample_count; ++i) {
        double value = sample_func();
        samples.push_back(value);

        if (std::abs(value - expected_range.low()) < epsilon) {
            values_at_min++;
        }
        if (std::abs(value - expected_range.high()) < epsilon) {
            values_at_max++;
        }
    }

    double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    double mean = sum / sample_count;

    double variance_sum = 0.0;
    for (double value : samples) {
        double diff = value - mean;
        variance_sum += diff * diff;
    }
    double variance = variance_sum / sample_count;

    auto [min_it, max_it] = std::minmax_element(samples.begin(), samples.end());

    DistributionStatistics stats = {
        mean,
        variance,
        *min_it,
        *max_it,
        sample_count
    };

    return {stats, values_at_min, values_at_max};
}

inline double compute_range_coverage(
    const DistributionStatistics& stats,
    const Interval& expected_range
) {
    double range_span = stats.max_value - stats.min_value;
    double expected_span = expected_range.measure();
    if (expected_span == 0.0) {
        return 0.0;
    }
    return range_span / expected_span;
}

inline double compute_clipping_ratio(const DistributionMetrics& metrics) {
    return static_cast<double>(metrics.values_at_min + metrics.values_at_max) / metrics.stats.sample_count;
}

inline void expect_uniform_distribution_mean(
    const DistributionStatistics& stats,
    double expected_mean,
    double tolerance
) {
    EXPECT_NEAR(stats.mean, expected_mean, tolerance)
        << "Expected mean " << expected_mean << " but got " << stats.mean
        << " (sample count: " << stats.sample_count << ")";
}

inline void expect_uniform_distribution_variance(
    const DistributionStatistics& stats,
    double expected_variance,
    double tolerance
) {
    EXPECT_NEAR(stats.variance, expected_variance, tolerance)
        << "Expected variance " << expected_variance << " but got " << stats.variance
        << " (sample count: " << stats.sample_count << ")";
}

inline double uniform_distribution_variance(double range_low, double range_high) {
    double range = range_high - range_low;
    return (range * range) / 12.0;
}

inline double uniform_distribution_mean(double range_low, double range_high) {
    return (range_low + range_high) / 2.0;
}

}

#endif //GLOBEART_SRC_GLOBE_TESTING_STATISTICAL_ASSERTIONS_HPP_
