#include "polygon_moments.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace globe;

namespace {

SphericalPolygon make_triangle(const Point3& a, const Point3& b, const Point3& c) {
    std::vector<Arc> arcs;
    auto circle_ab = detail::SphericalKernel::Circle_3(
        detail::SphericalKernel::Point_3(0, 0, 0),
        detail::SphericalKernel::Point_3(a.x(), a.y(), a.z()),
        detail::SphericalKernel::Point_3(b.x(), b.y(), b.z())
    );
    auto circle_bc = detail::SphericalKernel::Circle_3(
        detail::SphericalKernel::Point_3(0, 0, 0),
        detail::SphericalKernel::Point_3(b.x(), b.y(), b.z()),
        detail::SphericalKernel::Point_3(c.x(), c.y(), c.z())
    );
    auto circle_ca = detail::SphericalKernel::Circle_3(
        detail::SphericalKernel::Point_3(0, 0, 0),
        detail::SphericalKernel::Point_3(c.x(), c.y(), c.z()),
        detail::SphericalKernel::Point_3(a.x(), a.y(), a.z())
    );

    arcs.emplace_back(circle_ab,
        detail::SphericalKernel::Point_3(a.x(), a.y(), a.z()),
        detail::SphericalKernel::Point_3(b.x(), b.y(), b.z()));
    arcs.emplace_back(circle_bc,
        detail::SphericalKernel::Point_3(b.x(), b.y(), b.z()),
        detail::SphericalKernel::Point_3(c.x(), c.y(), c.z()));
    arcs.emplace_back(circle_ca,
        detail::SphericalKernel::Point_3(c.x(), c.y(), c.z()),
        detail::SphericalKernel::Point_3(a.x(), a.y(), a.z()));

    return SphericalPolygon(arcs);
}

}

TEST(PolygonMomentsTest, TriangleArea) {
    Point3 a(1, 0, 0);
    Point3 b(0, 1, 0);
    Point3 c(0, 0, 1);

    auto polygon = make_triangle(a, b, c);
    auto moments = compute_polygon_moments(polygon);

    double expected_area = M_PI / 2.0;
    EXPECT_NEAR(moments.area, expected_area, 1e-6);
}

TEST(PolygonMomentsTest, TriangleFirstMomentDirection) {
    Point3 a(1, 0, 0);
    Point3 b(0, 1, 0);
    Point3 c(0, 0, 1);

    auto polygon = make_triangle(a, b, c);
    auto moments = compute_polygon_moments(polygon);

    EXPECT_GT(moments.first_moment.x(), 0);
    EXPECT_GT(moments.first_moment.y(), 0);
    EXPECT_GT(moments.first_moment.z(), 0);

    EXPECT_NEAR(moments.first_moment.x(), moments.first_moment.y(), 1e-6);
    EXPECT_NEAR(moments.first_moment.y(), moments.first_moment.z(), 1e-6);
}

TEST(PolygonMomentsTest, SecondMomentSymmetric) {
    Point3 a(1, 0, 0);
    Point3 b(0, 1, 0);
    Point3 c(0, 0, 1);

    auto polygon = make_triangle(a, b, c);
    auto moments = compute_polygon_moments(polygon);

    EXPECT_NEAR(moments.second_moment(0, 1), moments.second_moment(1, 0), 1e-10);
    EXPECT_NEAR(moments.second_moment(0, 2), moments.second_moment(2, 0), 1e-10);
    EXPECT_NEAR(moments.second_moment(1, 2), moments.second_moment(2, 1), 1e-10);
}

TEST(PolygonMomentsTest, SecondMomentTrace) {
    Point3 a(1, 0, 0);
    Point3 b(0, 1, 0);
    Point3 c(0, 0, 1);

    auto polygon = make_triangle(a, b, c);
    auto moments = compute_polygon_moments(polygon);

    EXPECT_NEAR(moments.second_moment.trace(), moments.area, 1e-6);
}

TEST(PolygonMomentsTest, AreaMatchesSphericalPolygonArea) {
    Point3 a(1, 0, 0);
    Point3 b(0, 1, 0);
    Point3 c(0, 0, 1);

    auto polygon = make_triangle(a, b, c);
    auto moments = compute_polygon_moments(polygon);

    EXPECT_NEAR(moments.area, polygon.area(), 1e-6);
}
