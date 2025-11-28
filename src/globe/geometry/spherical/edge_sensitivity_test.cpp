#include "edge_sensitivity.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace globe {
namespace {

TEST(EdgeSensitivityTest, DirectionTowardsNeighbor) {
    Point3 site_i(1, 0, 0);
    Point3 site_j(0, 1, 0);
    Point3 v1(0.5, 0.5, std::sqrt(0.5));
    Point3 v2(0.5, 0.5, -std::sqrt(0.5));

    Eigen::Vector3d sensitivity = compute_edge_sensitivity(site_i, site_j, v1, v2);

    EXPECT_GT(sensitivity.y(), 0);
    EXPECT_NEAR(sensitivity.x(), 0, 1e-10);
    EXPECT_NEAR(sensitivity.z(), 0, 1e-10);
}

TEST(EdgeSensitivityTest, TangentToSphere) {
    Point3 site_i(1, 0, 0);
    Point3 site_j(0, 1, 0);
    Point3 v1(0.5, 0.5, std::sqrt(0.5));
    Point3 v2(0.5, 0.5, -std::sqrt(0.5));

    Eigen::Vector3d sensitivity = compute_edge_sensitivity(site_i, site_j, v1, v2);
    Eigen::Vector3d si(1, 0, 0);

    EXPECT_NEAR(sensitivity.dot(si), 0, 1e-10);
}

TEST(EdgeSensitivityTest, MagnitudeProportionalToEdgeLength) {
    Point3 site_i(1, 0, 0);
    Point3 site_j(0, 1, 0);

    double angle = M_PI / 3;
    Point3 v1(0.5, 0.5, std::sin(angle / 2));
    Point3 v2(0.5, 0.5, -std::sin(angle / 2));

    double edge_length = angular_distance(
        to_position_vector(v1),
        to_position_vector(v2)
    );

    Eigen::Vector3d sensitivity = compute_edge_sensitivity(site_i, site_j, v1, v2);

    EXPECT_NEAR(sensitivity.norm(), 0.5 * edge_length, 1e-6);
}

TEST(EdgeSensitivityTest, CoincidentSitesReturnZero) {
    Point3 site(1, 0, 0);
    Point3 v1(0, 1, 0);
    Point3 v2(0, 0, 1);

    Eigen::Vector3d sensitivity = compute_edge_sensitivity(site, site, v1, v2);

    EXPECT_NEAR(sensitivity.norm(), 0, 1e-10);
}

TEST(EdgeSensitivityTest, ZeroLengthEdgeReturnsZero) {
    Point3 site_i(1, 0, 0);
    Point3 site_j(0, 1, 0);
    Point3 v(0.5, 0.5, std::sqrt(0.5));

    Eigen::Vector3d sensitivity = compute_edge_sensitivity(site_i, site_j, v, v);

    EXPECT_NEAR(sensitivity.norm(), 0, 1e-10);
}

}
}
