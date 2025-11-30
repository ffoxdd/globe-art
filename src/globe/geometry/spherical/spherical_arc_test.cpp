#include <gtest/gtest.h>
#include "spherical_arc.hpp"
#include "../../testing/arc_factory.hpp"
#include <cmath>

using namespace globe;
using globe::testing::make_arc;

TEST(SphericalArcTest, LengthOfQuarterArc) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    EXPECT_NEAR(arc.length(), M_PI / 2.0, 1e-10);
}

TEST(SphericalArcTest, LengthOfZeroArc) {
    Point3 p(1, 0, 0);
    SphericalArc arc(p, p, Vector3(0, 0, 1));

    EXPECT_NEAR(arc.length(), 0.0, 1e-10);
}

TEST(SphericalArcTest, LengthDoesNotComputeMoments) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    double length1 = arc.length();
    double length2 = arc.length();

    EXPECT_DOUBLE_EQ(length1, length2);
}

TEST(SphericalArcTest, FirstMomentOfQuarterArc) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    Eigen::Vector3d moment = arc.first_moment();

    EXPECT_NEAR(moment.x(), 1.0, 1e-10);
    EXPECT_NEAR(moment.y(), 1.0, 1e-10);
    EXPECT_NEAR(moment.z(), 0.0, 1e-10);
}

TEST(SphericalArcTest, FirstMomentTowardsPole) {
    SphericalArc arc = make_arc(Vector3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, 1));

    Eigen::Vector3d moment = arc.first_moment();

    EXPECT_NEAR(moment.x(), 1.0, 1e-10);
    EXPECT_NEAR(moment.y(), 0.0, 1e-10);
    EXPECT_NEAR(moment.z(), 1.0, 1e-10);
}

TEST(SphericalArcTest, SecondMomentIsSymmetric) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    Eigen::Matrix3d moment = arc.second_moment();

    EXPECT_NEAR(moment(0, 1), moment(1, 0), 1e-10);
    EXPECT_NEAR(moment(0, 2), moment(2, 0), 1e-10);
    EXPECT_NEAR(moment(1, 2), moment(2, 1), 1e-10);
}

TEST(SphericalArcTest, SecondMomentTraceEqualsLength) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    double trace = arc.second_moment().trace();

    EXPECT_NEAR(trace, arc.length(), 1e-10);
}

TEST(SphericalArcTest, ZeroArcHasZeroMoments) {
    Point3 p(1, 0, 0);
    SphericalArc arc(p, p, Vector3(0, 0, 1));

    EXPECT_NEAR(arc.first_moment().norm(), 0.0, 1e-10);
    EXPECT_NEAR(arc.second_moment().norm(), 0.0, 1e-10);
}

TEST(SphericalArcTest, ContainsSourceAndTarget) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    EXPECT_TRUE(arc.contains(arc.source()));
    EXPECT_TRUE(arc.contains(arc.target()));
}

TEST(SphericalArcTest, ContainsMidpoint) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    double inv_sqrt2 = 1.0 / std::sqrt(2.0);
    Point3 midpoint(inv_sqrt2, inv_sqrt2, 0);

    EXPECT_TRUE(arc.contains(midpoint));
}

TEST(SphericalArcTest, DoesNotContainPointOffArc) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    EXPECT_FALSE(arc.contains(Point3(-1, 0, 0)));
    EXPECT_FALSE(arc.contains(Point3(0, 0, 1)));
}

TEST(SphericalArcTest, SubarcHasShorterLength) {
    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    double inv_sqrt2 = 1.0 / std::sqrt(2.0);
    Point3 midpoint(inv_sqrt2, inv_sqrt2, 0);

    SphericalArc subarc = arc.subarc(midpoint);

    EXPECT_LT(subarc.length(), arc.length());
    EXPECT_NEAR(subarc.length(), arc.length() / 2.0, 1e-10);
}
