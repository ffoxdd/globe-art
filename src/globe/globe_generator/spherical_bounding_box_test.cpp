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

TEST(SphericalBoundingBoxTest, CenterReturnsPointOnUnitSphere) {
    Interval theta_interval(0.0, M_PI / 2);
    Interval z_interval(0.0, 0.5);
    SphericalBoundingBox box(theta_interval, z_interval);

    Point3 center = box.center();

    double distance = std::sqrt(
        center.x() * center.x() +
        center.y() * center.y() +
        center.z() * center.z()
    );
    EXPECT_NEAR(distance, 1.0, 1e-9);
}

TEST(SphericalBoundingBoxTest, CenterUsesIntervalMidpoints) {
    Interval theta_interval(0.0, M_PI / 2);
    Interval z_interval(-0.5, 0.5);
    SphericalBoundingBox box(theta_interval, z_interval);

    Point3 center = box.center();
    double expected_theta = M_PI / 4;
    double expected_z = 0.0;
    double expected_r = 1.0;

    EXPECT_NEAR(center.z(), expected_z, 1e-9);
    EXPECT_NEAR(center.x(), expected_r * std::cos(expected_theta), 1e-9);
    EXPECT_NEAR(center.y(), expected_r * std::sin(expected_theta), 1e-9);
}
