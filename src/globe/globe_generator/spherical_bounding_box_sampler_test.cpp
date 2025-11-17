#include "gtest/gtest.h"
#include "./spherical_bounding_box_sampler.hpp"
#include "./spherical_bounding_box.hpp"
#include "../scalar_field/interval.hpp"
#include <cmath>

using namespace globe;

TEST(SphericalBoundingBoxSamplerTest, SamplesPointsOnUnitSphereBand) {
    SphericalBoundingBoxSampler sampler;
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
    SphericalBoundingBoxSampler sampler;
    SphericalBoundingBox box{Interval(-M_PI / 4, M_PI / 4), Interval(-0.2, 0.2)};

    for (int i = 0; i < 500; ++i) {
        Point3 sample = sampler.sample(box);
        double theta = std::atan2(sample.y(), sample.x());

        EXPECT_GE(theta, box.theta_interval().low());
        EXPECT_LE(theta, box.theta_interval().high());
    }
}
