#include "gtest/gtest.h"
#include "bounding_box.hpp"
#include "../../testing/geometric_assertions.hpp"

using namespace globe;
using globe::testing::expect_intervals_equal;

TEST(BoundingBoxTest, DefaultConstructorCreatesUnitCube) {
    BoundingBox box;

    expect_intervals_equal(box.x_interval(), Interval(-1.0, 1.0));
    expect_intervals_equal(box.y_interval(), Interval(-1.0, 1.0));
    expect_intervals_equal(box.z_interval(), Interval(-1.0, 1.0));
}

TEST(BoundingBoxTest, ConstructorWithExplicitIntervals) {
    Interval x_interval(0.0, 2.0);
    Interval y_interval(-1.0, 1.0);
    Interval z_interval(0.5, 1.5);
    BoundingBox box(x_interval, y_interval, z_interval);

    expect_intervals_equal(box.x_interval(), x_interval);
    expect_intervals_equal(box.y_interval(), y_interval);
    expect_intervals_equal(box.z_interval(), z_interval);
}

TEST(BoundingBoxTest, CenterReturnsMidpointOfIntervals) {
    Interval x_interval(0.0, 4.0);
    Interval y_interval(-2.0, 2.0);
    Interval z_interval(1.0, 3.0);
    BoundingBox box(x_interval, y_interval, z_interval);

    Point3 center = box.center();

    EXPECT_DOUBLE_EQ(center.x(), 2.0);
    EXPECT_DOUBLE_EQ(center.y(), 0.0);
    EXPECT_DOUBLE_EQ(center.z(), 2.0);
}

TEST(BoundingBoxTest, CenterWithNegativeIntervals) {
    Interval x_interval(-5.0, -1.0);
    Interval y_interval(-3.0, 1.0);
    Interval z_interval(-2.0, 0.0);
    BoundingBox box(x_interval, y_interval, z_interval);

    Point3 center = box.center();

    EXPECT_DOUBLE_EQ(center.x(), -3.0);
    EXPECT_DOUBLE_EQ(center.y(), -1.0);
    EXPECT_DOUBLE_EQ(center.z(), -1.0);
}

TEST(BoundingBoxTest, ContainsPointInsideBox) {
    BoundingBox box(Interval(0.0, 2.0), Interval(-1.0, 1.0), Interval(0.5, 1.5));
    Point3 point(1.0, 0.0, 1.0);

    EXPECT_TRUE(box.contains(point));
}

TEST(BoundingBoxTest, ContainsPointOnBoundary) {
    BoundingBox box(Interval(0.0, 2.0), Interval(-1.0, 1.0), Interval(0.5, 1.5));
    Point3 point_on_x_low(0.0, 0.0, 1.0);
    Point3 point_on_x_high(2.0, 0.0, 1.0);
    Point3 point_on_y_low(1.0, -1.0, 1.0);
    Point3 point_on_y_high(1.0, 1.0, 1.0);
    Point3 point_on_z_low(1.0, 0.0, 0.5);
    Point3 point_on_z_high(1.0, 0.0, 1.5);

    EXPECT_TRUE(box.contains(point_on_x_low));
    EXPECT_TRUE(box.contains(point_on_x_high));
    EXPECT_TRUE(box.contains(point_on_y_low));
    EXPECT_TRUE(box.contains(point_on_y_high));
    EXPECT_TRUE(box.contains(point_on_z_low));
    EXPECT_TRUE(box.contains(point_on_z_high));
}

TEST(BoundingBoxTest, DoesNotContainPointOutsideBox) {
    BoundingBox box(Interval(0.0, 2.0), Interval(-1.0, 1.0), Interval(0.5, 1.5));
    Point3 point_below_x(3.0, 0.0, 1.0);
    Point3 point_above_x(-1.0, 0.0, 1.0);
    Point3 point_below_y(1.0, -2.0, 1.0);
    Point3 point_above_y(1.0, 2.0, 1.0);
    Point3 point_below_z(1.0, 0.0, 0.0);
    Point3 point_above_z(1.0, 0.0, 2.0);

    EXPECT_FALSE(box.contains(point_below_x));
    EXPECT_FALSE(box.contains(point_above_x));
    EXPECT_FALSE(box.contains(point_below_y));
    EXPECT_FALSE(box.contains(point_above_y));
    EXPECT_FALSE(box.contains(point_below_z));
    EXPECT_FALSE(box.contains(point_above_z));
}

TEST(BoundingBoxTest, UnitCubeStaticMethod) {
    BoundingBox unit = BoundingBox::unit_cube();

    expect_intervals_equal(unit.x_interval(), Interval(-1.0, 1.0));
    expect_intervals_equal(unit.y_interval(), Interval(-1.0, 1.0));
    expect_intervals_equal(unit.z_interval(), Interval(-1.0, 1.0));
}

TEST(BoundingBoxTest, UnitCubeMatchesDefaultConstructor) {
    BoundingBox default_box;
    BoundingBox unit_box = BoundingBox::unit_cube();

    expect_intervals_equal(default_box.x_interval(), unit_box.x_interval());
    expect_intervals_equal(default_box.y_interval(), unit_box.y_interval());
    expect_intervals_equal(default_box.z_interval(), unit_box.z_interval());
}

