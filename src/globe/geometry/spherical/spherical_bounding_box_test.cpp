#include "gtest/gtest.h"
#include "spherical_bounding_box.hpp"
#include "../../testing/geometric_assertions.hpp"

using namespace globe;
using globe::testing::expect_intervals_equal;

TEST(SphericalBoundingBoxTest, DefaultsToFullSphere) {
    SphericalBoundingBox box;

    expect_intervals_equal(box.theta_interval(), Interval(0.0, 2 * M_PI));
    expect_intervals_equal(box.z_interval(), Interval(-1.0, 1.0));
    EXPECT_NEAR(box.area(), 4 * M_PI, 1e-9);
}

TEST(SphericalBoundingBoxTest, RespectsExplicitIntervals) {
    Interval theta_interval(-M_PI / 3, M_PI / 6);
    Interval z_interval(0.2, 0.7);

    SphericalBoundingBox box(theta_interval, z_interval);

    expect_intervals_equal(box.theta_interval(), theta_interval);
    expect_intervals_equal(box.z_interval(), z_interval);
    EXPECT_DOUBLE_EQ(box.area(), theta_interval.measure() * z_interval.measure());
}

TEST(SphericalBoundingBoxTest, ConstructsFromIntervals) {
    Interval theta_interval(-M_PI / 2, M_PI / 4);
    Interval z_interval(-0.3, 0.8);

    SphericalBoundingBox box(theta_interval, z_interval);

    expect_intervals_equal(box.theta_interval(), theta_interval);
    expect_intervals_equal(box.z_interval(), z_interval);
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

TEST(SphericalBoundingBoxTest, HandlesWraparoundThetaInterval) {
    Interval wrapped_theta(5.5, 0.8 + 2.0 * M_PI);
    Interval z_interval(0.0, 0.5);
    SphericalBoundingBox box(wrapped_theta, z_interval);

    EXPECT_TRUE(box.is_theta_wrapped());

    double expected_measure = (2.0 * M_PI - 5.5) + 0.8;
    EXPECT_NEAR(box.theta_interval().measure(), expected_measure, 1e-9);

    Point3 center = box.center();
    double expected_theta = (5.5 + 0.8 + 2.0 * M_PI) / 2.0 - 2.0 * M_PI;
    double expected_z = 0.25;
    double expected_r = std::sqrt(1.0 - expected_z * expected_z);

    EXPECT_NEAR(center.z(), expected_z, 1e-9);
    EXPECT_NEAR(center.x(), expected_r * std::cos(expected_theta), 1e-9);
    EXPECT_NEAR(center.y(), expected_r * std::sin(expected_theta), 1e-9);
}

TEST(SphericalBoundingBoxTest, BoundingSphereRadiusEnclosesBox) {
    Interval theta_interval(0.0, M_PI / 2);
    Interval z_interval(0.0, 0.5);
    SphericalBoundingBox box(theta_interval, z_interval);

    double radius = box.bounding_sphere_radius();
    Point3 center = box.center();

    double theta_span = box.theta_interval().measure();
    double z_span = box.z_interval().measure();
    double r_max = std::sqrt(1.0 - z_interval.low() * z_interval.low());
    double chord = 2.0 * r_max * std::sin(theta_span / 2.0);
    double expected_radius = std::sqrt(z_span * z_span + chord * chord);

    EXPECT_NEAR(radius, expected_radius, 1e-9);
    EXPECT_GT(radius, 0.0);
}
