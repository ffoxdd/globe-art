#ifndef GLOBEART_SRC_GLOBE_TESTING_STATISTICAL_ASSERTIONS_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_STATISTICAL_ASSERTIONS_HPP_

#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include "../types.hpp"
#include "../math/interval.hpp"

namespace globe::testing {

struct DistributionMetrics {
    double mean;
    double variance;
    double min_value;
    double max_value;
    size_t sample_count;
    size_t values_at_min;
    size_t values_at_max;
    double range_span;
};

struct CoordinateMetrics {
    DistributionMetrics x;
    DistributionMetrics y;
    DistributionMetrics z;
};

template<typename SampleFunc>
DistributionMetrics compute_statistics(
    SampleFunc sample_func,
    size_t sample_count,
    const Interval* expected_range = nullptr,
    double epsilon = 1e-10
) {
    std::vector<double> samples;
    samples.reserve(sample_count);

    size_t values_at_min = 0;
    size_t values_at_max = 0;

    for (size_t i = 0; i < sample_count; ++i) {
        double value = sample_func();
        samples.push_back(value);

        if (expected_range != nullptr) {
            if (std::abs(value - expected_range->low()) < epsilon) {
                values_at_min++;
            }

            if (std::abs(value - expected_range->high()) < epsilon) {
                values_at_max++;
            }
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

    double range_span = *max_it - *min_it;

    return {
        mean,
        variance,
        *min_it,
        *max_it,
        sample_count,
        values_at_min,
        values_at_max,
        range_span
    };
}

template<typename PointFunc>
CoordinateMetrics compute_coordinate_statistics(
    PointFunc point_func,
    size_t sample_count
) {
    std::vector<Point3> points;
    points.reserve(sample_count);

    for (size_t i = 0; i < sample_count; ++i) {
        points.push_back(point_func());
    }

    auto compute_from_coordinate = [&points](auto coordinate_extractor) {
        std::vector<double> values;
        values.reserve(points.size());

        for (const auto& point : points) {
            values.push_back(coordinate_extractor(point));
        }

        double sum = std::accumulate(values.begin(), values.end(), 0.0);
        double mean = sum / values.size();

        double variance_sum = 0.0;

        for (double value : values) {
            double diff = value - mean;
            variance_sum += diff * diff;
        }

        double variance = variance_sum / values.size();
        auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
        double range_span = *max_it - *min_it;

        return DistributionMetrics{
            mean,
            variance,
            *min_it,
            *max_it,
            values.size(),
            0,
            0,
            range_span
        };
    };

    return CoordinateMetrics{
        compute_from_coordinate([](const Point3& p) { return p.x(); }),
        compute_from_coordinate([](const Point3& p) { return p.y(); }),
        compute_from_coordinate([](const Point3& p) { return p.z(); })
    };
}

inline double compute_clipping_ratio(const DistributionMetrics& metrics) {
    return static_cast<double>(metrics.values_at_min + metrics.values_at_max) / metrics.sample_count;
}

inline void expect_uniform_distribution_mean(
    const DistributionMetrics& metrics,
    double expected_mean,
    double tolerance
) {
    EXPECT_NEAR(metrics.mean, expected_mean, tolerance)
        << "Expected mean " << expected_mean << " but got " << metrics.mean
        << " (sample count: " << metrics.sample_count << ")";
}

inline void expect_uniform_distribution_variance(
    const DistributionMetrics& metrics,
    double expected_variance,
    double tolerance
) {
    EXPECT_NEAR(metrics.variance, expected_variance, tolerance)
        << "Expected variance " << expected_variance << " but got " << metrics.variance
        << " (sample count: " << metrics.sample_count << ")";
}

inline double uniform_distribution_variance(double range_low, double range_high) {
    double range = range_high - range_low;
    return (range * range) / 12.0;
}

inline double uniform_distribution_mean(double range_low, double range_high) {
    return (range_low + range_high) / 2.0;
}

inline void expect_range_coverage(
    const DistributionMetrics& metrics,
    const Interval& expected_range,
    double expected_coverage,
    double tolerance = 0.0
) {
    double expected_span = expected_range.measure();
    double actual_coverage = 0.0;

    if (expected_span != 0.0) {
        actual_coverage = metrics.range_span / expected_span;
    }

    if (tolerance > 0.0) {
        EXPECT_NEAR(actual_coverage, expected_coverage, tolerance)
            << "Expected range coverage " << expected_coverage << " but got " << actual_coverage
            << " (range span: " << metrics.range_span << ", expected span: " << expected_span << ")";
    } else {
        EXPECT_GE(actual_coverage, expected_coverage)
            << "Expected range coverage >= " << expected_coverage << " but got " << actual_coverage
            << " (range span: " << metrics.range_span << ", expected span: " << expected_span << ")";
    }
}

}

#endif //GLOBEART_SRC_GLOBE_TESTING_STATISTICAL_ASSERTIONS_HPP_
