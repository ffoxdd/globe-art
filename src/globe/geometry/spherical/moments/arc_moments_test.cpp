#include "arc_moments.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace globe;

TEST(ArcMomentsTest, ZeroLengthArc) {
    Point3 p(1, 0, 0);
    ArcMoments moments = compute_arc_moments(p, p);

    EXPECT_NEAR(moments.length, 0.0, 1e-10);
    EXPECT_NEAR(moments.first_moment.norm(), 0.0, 1e-10);
    EXPECT_NEAR(moments.second_moment.norm(), 0.0, 1e-10);
}

TEST(ArcMomentsTest, QuarterArcAlongEquator) {
    Point3 p1(1, 0, 0);
    Point3 p2(0, 1, 0);
    ArcMoments moments = compute_arc_moments(p1, p2);

    EXPECT_NEAR(moments.length, M_PI / 2.0, 1e-10);

    EXPECT_NEAR(moments.first_moment.x(), 1.0, 1e-10);
    EXPECT_NEAR(moments.first_moment.y(), 1.0, 1e-10);
    EXPECT_NEAR(moments.first_moment.z(), 0.0, 1e-10);
}

TEST(ArcMomentsTest, QuarterArcTowardsPole) {
    Point3 p1(1, 0, 0);
    Point3 p2(0, 0, 1);
    ArcMoments moments = compute_arc_moments(p1, p2);

    EXPECT_NEAR(moments.length, M_PI / 2.0, 1e-10);

    EXPECT_NEAR(moments.first_moment.x(), 1.0, 1e-10);
    EXPECT_NEAR(moments.first_moment.y(), 0.0, 1e-10);
    EXPECT_NEAR(moments.first_moment.z(), 1.0, 1e-10);
}

TEST(ArcMomentsTest, NearlyHalfArc) {
    Point3 p1(1, 0, 0);
    double norm = std::sqrt(1.0 + 0.01 * 0.01);
    Point3 p2(-1.0 / norm, 0.01 / norm, 0);
    ArcMoments moments = compute_arc_moments(p1, p2);

    EXPECT_GT(moments.length, M_PI - 0.1);
    EXPECT_LT(moments.length, M_PI + 0.001);
    EXPECT_GT(moments.first_moment.y(), 0.0);
}

TEST(ArcMomentsTest, AntipodalPointsHaveDefinedMoments) {
    Point3 p1(1, 0, 0);
    Point3 p2(-1, 0, 0);
    ArcMoments moments = compute_arc_moments(p1, p2);

    EXPECT_NEAR(moments.length, M_PI, 1e-10);
    EXPECT_NEAR(moments.first_moment.norm(), 2.0, 1e-10);
}

TEST(ArcMomentsTest, SecondMomentSymmetric) {
    Point3 p1(1, 0, 0);
    Point3 p2(0, 1, 0);
    ArcMoments moments = compute_arc_moments(p1, p2);

    EXPECT_NEAR(moments.second_moment(0, 1), moments.second_moment(1, 0), 1e-10);
    EXPECT_NEAR(moments.second_moment(0, 2), moments.second_moment(2, 0), 1e-10);
    EXPECT_NEAR(moments.second_moment(1, 2), moments.second_moment(2, 1), 1e-10);
}

TEST(ArcMomentsTest, SecondMomentTrace) {
    Point3 p1(1, 0, 0);
    Point3 p2(0, 1, 0);
    ArcMoments moments = compute_arc_moments(p1, p2);

    double trace = moments.second_moment.trace();
    EXPECT_NEAR(trace, moments.length, 1e-10);
}
