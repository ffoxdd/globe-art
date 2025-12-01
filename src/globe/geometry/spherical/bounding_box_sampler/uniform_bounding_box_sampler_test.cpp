#include "gtest/gtest.h"
#include "bounding_box_sampler.hpp"
#include "uniform_bounding_box_sampler.hpp"
#include "../bounding_box.hpp"
#include "../../../math/interval.hpp"
#include "../../../testing/geometric_assertions.hpp"
#include "../../../testing/statistical_assertions.hpp"
#include "../../../testing/test_fixtures.hpp"
#include <cmath>

using namespace globe;
using globe::testing::compute_statistics;
using globe::testing::expect_mean;
using globe::testing::expect_variance;
using globe::testing::expect_range_coverage;
using globe::testing::uniform_distribution_mean;
using globe::testing::uniform_distribution_variance;
using globe::testing::SequenceIntervalSampler;
using globe::testing::SequenceCircularIntervalSampler;

TEST(UniformSphericalBoundingBoxSamplerTest, ProducesPointOnUnitSphere) {
    SequenceIntervalSampler z_sampler({0.5});
    SequenceCircularIntervalSampler theta_sampler({0.25});
    UniformSphericalBoundingBoxSampler sampler(z_sampler, theta_sampler);
    SphericalBoundingBox box{ThetaInterval::full(), Interval(-1.0, 1.0)};

    VectorS2 sample = sampler.sample(box);

    EXPECT_NEAR(sample.norm(), 1.0, 1e-9);
}

TEST(UniformSphericalBoundingBoxSamplerTest, UsesZFromIntervalSampler) {
    SequenceIntervalSampler z_sampler({0.75});
    SequenceCircularIntervalSampler theta_sampler({0.0});
    UniformSphericalBoundingBoxSampler sampler(z_sampler, theta_sampler);
    SphericalBoundingBox box{ThetaInterval::full(), Interval(0.0, 1.0)};

    VectorS2 sample = sampler.sample(box);

    EXPECT_NEAR(sample.z(), 0.75, 1e-9);
}

TEST(UniformSphericalBoundingBoxSamplerTest, UsesThetaFromCircularIntervalSampler) {
    SequenceIntervalSampler z_sampler({0.5});
    SequenceCircularIntervalSampler theta_sampler({0.5});
    UniformSphericalBoundingBoxSampler sampler(z_sampler, theta_sampler);
    SphericalBoundingBox box{ThetaInterval(0.0, M_PI), Interval(-1.0, 1.0)};

    VectorS2 sample = sampler.sample(box);
    double theta = std::atan2(sample.y(), sample.x());

    EXPECT_NEAR(theta, M_PI / 2.0, 1e-9);
}

TEST(UniformSphericalBoundingBoxSamplerTest, HandlesWrappedThetaInterval) {
    SequenceIntervalSampler z_sampler({0.5});
    SequenceCircularIntervalSampler theta_sampler({0.0});
    UniformSphericalBoundingBoxSampler sampler(z_sampler, theta_sampler);
    SphericalBoundingBox box{ThetaInterval(5.5, 1.5), Interval(-1.0, 1.0)};

    VectorS2 sample = sampler.sample(box);

    EXPECT_TRUE(box.contains(sample));
}

TEST(UniformSphericalBoundingBoxSamplerTest, EXPENSIVE_AllPointsOnUnitSphere) {
    REQUIRE_EXPENSIVE();

    UniformSphericalBoundingBoxSampler<> sampler;
    SphericalBoundingBox box{ThetaInterval::full(), Interval(0.0, 0.5)};
    constexpr size_t sample_count = 10000;

    for (size_t i = 0; i < sample_count; ++i) {
        VectorS2 sample = sampler.sample(box);

        EXPECT_NEAR(sample.norm(), 1.0, 1e-9);
        EXPECT_TRUE(box.contains(sample));
    }
}

TEST(UniformSphericalBoundingBoxSamplerTest, EXPENSIVE_UniformZDistribution) {
    REQUIRE_EXPENSIVE();

    UniformSphericalBoundingBoxSampler<> sampler;
    Interval z_interval(-0.3, 0.7);
    SphericalBoundingBox box{ThetaInterval::full(), z_interval};
    constexpr size_t sample_count = 50000;

    auto stats = compute_statistics(
        [&]() { return sampler.sample(box).z(); },
        sample_count
    );

    expect_mean(stats, uniform_distribution_mean(z_interval.low(), z_interval.high()), 0.02);
    expect_variance(stats, uniform_distribution_variance(z_interval.low(), z_interval.high()), 0.01);
    expect_range_coverage(stats, z_interval, 0.95);
}

TEST(UniformSphericalBoundingBoxSamplerTest, EXPENSIVE_WrappedIntervalContainment) {
    REQUIRE_EXPENSIVE();

    UniformSphericalBoundingBoxSampler<> sampler;
    SphericalBoundingBox box{ThetaInterval(5.5, 1.58), Interval(-0.2, 0.2)};
    constexpr size_t sample_count = 10000;

    for (size_t i = 0; i < sample_count; ++i) {
        VectorS2 sample = sampler.sample(box);

        EXPECT_NEAR(sample.norm(), 1.0, 1e-9);
        EXPECT_TRUE(box.contains(sample));
    }
}

TEST(UniformSphericalBoundingBoxSamplerTest, EXPENSIVE_CrossingZeroContainment) {
    REQUIRE_EXPENSIVE();

    UniformSphericalBoundingBoxSampler<> sampler;
    SphericalBoundingBox box{ThetaInterval(2 * M_PI - 0.5, 1.0), Interval(0.0, 0.5)};
    constexpr size_t sample_count = 10000;

    for (size_t i = 0; i < sample_count; ++i) {
        VectorS2 sample = sampler.sample(box);

        EXPECT_TRUE(box.contains(sample));
    }
}
