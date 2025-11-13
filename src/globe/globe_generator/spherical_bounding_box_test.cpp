#include "gtest/gtest.h"
#include "./spherical_bounding_box.hpp"
#include <vector>

using namespace globe;

TEST(SphericalBoundingBoxTest, DefaultsToFullSphere) {
    SphericalBoundingBox box;

    EXPECT_DOUBLE_EQ(box.theta_interval().low(), 0.0);
    EXPECT_DOUBLE_EQ(box.theta_interval().high(), 2 * M_PI);
    EXPECT_DOUBLE_EQ(box.z_interval().low(), -1.0);
    EXPECT_DOUBLE_EQ(box.z_interval().high(), 1.0);
    EXPECT_NEAR(box.area(), 4 * M_PI, 1e-9);
}

TEST(SphericalBoundingBoxTest, RespectsExplicitIntervals) {
    Interval theta_interval(-M_PI / 3, M_PI / 6);
    Interval z_interval(0.2, 0.7);

    SphericalBoundingBox box(theta_interval, z_interval);

    EXPECT_DOUBLE_EQ(box.theta_interval().low(), theta_interval.low());
    EXPECT_DOUBLE_EQ(box.theta_interval().high(), theta_interval.high());
    EXPECT_DOUBLE_EQ(box.z_interval().low(), z_interval.low());
    EXPECT_DOUBLE_EQ(box.z_interval().high(), z_interval.high());
    EXPECT_DOUBLE_EQ(box.area(), theta_interval.measure() * z_interval.measure());
}

TEST(SphericalBoundingBoxTest, ConstructsFromRanges) {
    std::vector<double> theta_values{-M_PI / 2, 0.0, M_PI / 4};
    std::vector<double> z_values{-0.3, 0.1, 0.8};

    SphericalBoundingBox box(theta_values, z_values);

    EXPECT_DOUBLE_EQ(box.theta_interval().low(), -M_PI / 2);
    EXPECT_DOUBLE_EQ(box.theta_interval().high(), M_PI / 4);
    EXPECT_DOUBLE_EQ(box.z_interval().low(), -0.3);
    EXPECT_DOUBLE_EQ(box.z_interval().high(), 0.8);
}
