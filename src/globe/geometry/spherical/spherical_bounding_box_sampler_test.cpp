#include "gtest/gtest.h"
#include "spherical_bounding_box_sampler.hpp"
#include "uniform_spherical_bounding_box_sampler.hpp"
#include "spherical_bounding_box.hpp"
#include "../../math/interval.hpp"
#include <cmath>

using namespace globe;

TEST(SphericalBoundingBoxSamplerTest, SamplesPointsOnUnitSphereBand) {
    UniformSphericalBoundingBoxSampler sampler;
    SphericalBoundingBox box{Interval(0, 2 * M_PI), Interval(0.0, 0.5)};

    for (int i = 0; i < 1000; ++i) {
        Point3 sample = sampler.sample(box);
        double radius = std::sqrt(sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z());

        EXPECT_NEAR(radius, 1.0, 1e-9);
        EXPECT_GE(sample.z(), box.z_interval().low());
        EXPECT_LE(sample.z(), box.z_interval().high());
    }
}

TEST(SphericalBoundingBoxSamplerTest, RespectsThetaInterval) {
    UniformSphericalBoundingBoxSampler sampler;
    SphericalBoundingBox box{Interval(M_PI / 2, 3 * M_PI / 4), Interval(-0.2, 0.2)};

    for (int i = 0; i < 500; ++i) {
        Point3 sample = sampler.sample(box);
        double theta = std::atan2(sample.y(), sample.x());
        if (theta < 0.0) theta += 2.0 * M_PI;

        EXPECT_GE(theta, box.theta_interval().low());
        EXPECT_LE(theta, box.theta_interval().high());
    }
}

TEST(SphericalBoundingBoxSamplerTest, HandlesWrappedThetaInterval) {
    UniformSphericalBoundingBoxSampler sampler;
    double theta_low = 5.5;
    double theta_high_logical = 0.8;
    SphericalBoundingBox box{Interval(theta_low, theta_high_logical + 2.0 * M_PI), Interval(-0.2, 0.2)};

    EXPECT_TRUE(box.is_theta_wrapped());

    for (int i = 0; i < 500; ++i) {
        Point3 sample = sampler.sample(box);
        double theta = std::atan2(sample.y(), sample.x());
        if (theta < 0.0) theta += 2.0 * M_PI;

        bool in_wrapped_interval = (theta >= theta_low) || (theta <= theta_high_logical);
        EXPECT_TRUE(in_wrapped_interval)
            << "theta = " << theta
            << " not in wrapped interval [" << theta_low
            << ", " << theta_high_logical << "]";

        double radius = std::sqrt(sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z());
        EXPECT_NEAR(radius, 1.0, 1e-9);
        EXPECT_GE(sample.z(), box.z_interval().low());
        EXPECT_LE(sample.z(), box.z_interval().high());
    }
}

TEST(SphericalBoundingBoxSamplerTest, WrappedIntervalCrossingZero) {
    UniformSphericalBoundingBoxSampler sampler;
    double theta_low = 2 * M_PI - 0.5;
    double theta_high_logical = 0.5;
    SphericalBoundingBox box{Interval(theta_low, theta_high_logical + 2.0 * M_PI), Interval(0.0, 0.5)};

    EXPECT_TRUE(box.is_theta_wrapped());

    for (int i = 0; i < 500; ++i) {
        Point3 sample = sampler.sample(box);
        double theta = std::atan2(sample.y(), sample.x());
        if (theta < 0.0) theta += 2.0 * M_PI;

        bool in_wrapped_interval = (theta >= theta_low) || (theta <= theta_high_logical);
        EXPECT_TRUE(in_wrapped_interval)
            << "theta = " << theta
            << " not in wrapped interval [" << theta_low
            << ", " << theta_high_logical << "]";
    }
}

