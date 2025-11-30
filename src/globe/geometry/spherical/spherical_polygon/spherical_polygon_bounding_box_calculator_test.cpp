#include "gtest/gtest.h"
#include "spherical_polygon_bounding_box_calculator.hpp"
#include "../spherical_arc.hpp"
#include "../../../testing/arc_factory.hpp"
#include "../../../testing/geometric_assertions.hpp"
#include <cmath>

using namespace globe;
using globe::testing::make_arc;


TEST(SphericalPolygonBoundingBoxCalculatorTest, ArcCrossingEquator) {
    std::vector<SphericalArc> arcs{
        make_arc(Vector3(0, 1, 0), Point3(0.5, 0, std::sqrt(0.75)), Point3(0, -1, 0)),
        make_arc(Vector3(0, 1, 0), Point3(0, -1, 0), Point3(-0.5, 0, std::sqrt(0.75))),
        make_arc(Vector3(0, 1, 0), Point3(-0.5, 0, std::sqrt(0.75)), Point3(0.5, 0, std::sqrt(0.75))),
    };

    auto bounding_box = SphericalPolygonBoundingBoxCalculator(arcs).calculate();

    EXPECT_LE(bounding_box.z_interval().low(), 0.0);
    EXPECT_GE(bounding_box.z_interval().high(), std::sqrt(0.75));
}

TEST(SphericalPolygonBoundingBoxCalculatorTest, HorizontalArc) {
    std::vector<SphericalArc> arcs{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(1, 0, 0)),
    };

    auto bounding_box = SphericalPolygonBoundingBoxCalculator(arcs).calculate();

    EXPECT_NEAR(bounding_box.z_interval().low(), 0.0, 1e-9);
    EXPECT_NEAR(bounding_box.z_interval().high(), 0.0, 1e-9);
}


TEST(SphericalPolygonBoundingBoxCalculatorTest, SmallArcNearEquator) {
    const double eps = 0.1;
    const double z_low = std::sqrt(1 - eps * eps);

    std::vector<SphericalArc> arcs{
        make_arc(Vector3(0, 0, 1), Point3(eps, 0, z_low), Point3(0, eps, z_low)),
        make_arc(Vector3(0, 0, 1), Point3(0, eps, z_low), Point3(eps, 0, z_low)),
    };

    auto bounding_box = SphericalPolygonBoundingBoxCalculator(arcs).calculate();

    EXPECT_GE(bounding_box.z_interval().low(), z_low - 1e-6);
    EXPECT_LE(bounding_box.z_interval().high(), z_low + 1e-6);
}

TEST(SphericalPolygonBoundingBoxCalculatorTest, LargeArcAlmostFullCircle) {
    const double sqrt2 = std::sqrt(2.0);
    const double eps = 0.01;

    std::vector<SphericalArc> arcs{
        make_arc(Vector3(0, 0, 1), Point3(sqrt2/2, sqrt2/2, 0), Point3(-sqrt2/2 + eps, sqrt2/2, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-sqrt2/2 + eps, sqrt2/2, 0), Point3(sqrt2/2, sqrt2/2, 0)),
    };

    auto bounding_box = SphericalPolygonBoundingBoxCalculator(arcs).calculate();

    EXPECT_NEAR(bounding_box.z_interval().low(), 0.0, 1e-2);
    EXPECT_NEAR(bounding_box.z_interval().high(), 0.0, 1e-2);
}

TEST(SphericalPolygonBoundingBoxCalculatorTest, PolygonWithNorthPoleContained) {
    std::vector<SphericalArc> arcs{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
    };

    auto bounding_box = SphericalPolygonBoundingBoxCalculator(arcs).calculate();

    EXPECT_NEAR(bounding_box.z_interval().low(), 0.0, 1e-9);
    EXPECT_NEAR(bounding_box.z_interval().high(), 1.0, 1e-9);
    EXPECT_TRUE(bounding_box.theta_interval().is_full());
}

TEST(SphericalPolygonBoundingBoxCalculatorTest, PolygonWithSouthPoleContained) {
    std::vector<SphericalArc> arcs{
        make_arc(Vector3(0, 0, -1), Point3(1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, -1), Point3(0, -1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, -1), Point3(-1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, -1), Point3(0, 1, 0), Point3(1, 0, 0)),
    };

    auto bounding_box = SphericalPolygonBoundingBoxCalculator(arcs).calculate();

    EXPECT_NEAR(bounding_box.z_interval().low(), -1.0, 1e-9);
    EXPECT_NEAR(bounding_box.z_interval().high(), 0.0, 1e-9);
    EXPECT_TRUE(bounding_box.theta_interval().is_full());
}


TEST(SphericalPolygonBoundingBoxCalculatorTest, ThetaWrappingAroundZero) {
    double theta1 = 5.8;
    double theta2 = 0.4;
    double z_low = 0.0;
    double z_high = 0.3;
    double r_low = std::sqrt(1.0 - z_low * z_low);
    double r_high = std::sqrt(1.0 - z_high * z_high);

    Point3 p1(r_low * std::cos(theta1), r_low * std::sin(theta1), z_low);
    Point3 p2(r_low * std::cos(theta2), r_low * std::sin(theta2), z_low);
    Point3 p3(r_high * std::cos(theta2), r_high * std::sin(theta2), z_high);
    Point3 p4(r_high * std::cos(theta1), r_high * std::sin(theta1), z_high);

    Vector3 v1 = CGAL::cross_product(to_position_vector(p1), to_position_vector(p2));
    Vector3 v2 = CGAL::cross_product(to_position_vector(p2), to_position_vector(p3));
    Vector3 v3 = CGAL::cross_product(to_position_vector(p3), to_position_vector(p4));
    Vector3 v4 = CGAL::cross_product(to_position_vector(p4), to_position_vector(p1));

    std::vector<SphericalArc> arcs{
        make_arc(v1, p1, p2),
        make_arc(v2, p2, p3),
        make_arc(v3, p3, p4),
        make_arc(v4, p4, p1)
    };

    auto bounding_box = SphericalPolygonBoundingBoxCalculator(arcs).calculate();

    EXPECT_GT(bounding_box.theta_interval().start(), M_PI);
    EXPECT_GT(bounding_box.theta_interval().end(), TWO_PI);
}
