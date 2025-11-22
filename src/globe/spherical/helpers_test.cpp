#include <gtest/gtest.h>
#include "helpers.hpp"
#include <cmath>

using namespace globe;

TEST(NormalizeVectorTest, NormalizesNonZeroVector) {
    Vector3 v = {3, 4, 0};
    Vector3 result = normalize(v);

    EXPECT_DOUBLE_EQ(result.x(), 0.6);
    EXPECT_DOUBLE_EQ(result.y(), 0.8);
    EXPECT_DOUBLE_EQ(result.z(), 0.0);
    EXPECT_NEAR(result.squared_length(), 1.0, 1e-10);
}

TEST(NormalizeVectorTest, HandlesZeroVector) {
    Vector3 v = {0, 0, 0};
    Vector3 result = normalize(v);

    EXPECT_DOUBLE_EQ(result.x(), 0.0);
    EXPECT_DOUBLE_EQ(result.y(), 0.0);
    EXPECT_DOUBLE_EQ(result.z(), 0.0);
}

TEST(NormalizeVectorTest, PreservesDirectionForUnitVector) {
    Vector3 v = {1, 0, 0};
    Vector3 result = normalize(v);

    EXPECT_DOUBLE_EQ(result.x(), 1.0);
    EXPECT_DOUBLE_EQ(result.y(), 0.0);
    EXPECT_DOUBLE_EQ(result.z(), 0.0);
}


TEST(AngularDistanceTest, ComputesRightAngle) {
    Vector3 a = {1, 0, 0};
    Vector3 b = {0, 1, 0};

    double angle = angular_distance(a, b);

    EXPECT_DOUBLE_EQ(angle, M_PI_2);
}

TEST(AngularDistanceTest, ComputesOppositeVectors) {
    Vector3 a = {1, 0, 0};
    Vector3 b = {-1, 0, 0};

    double angle = angular_distance(a, b);

    EXPECT_DOUBLE_EQ(angle, M_PI);
}

TEST(AngularDistanceTest, ComputesSameDirection) {
    Vector3 a = {1, 0, 0};
    Vector3 b = {2, 0, 0};

    double angle = angular_distance(a, b);

    EXPECT_NEAR(angle, 0.0, 1e-10);
}

TEST(AngularDistanceTest, WorksWithNonUnitVectors) {
    Vector3 a = {3, 0, 0};
    Vector3 b = {0, 5, 0};

    double angle = angular_distance(a, b);

    EXPECT_DOUBLE_EQ(angle, M_PI_2);
}

TEST(SphericalInterpolateTest, InterpolatesAtMidpoint) {
    Point3 p1 = {1, 0, 0};
    Point3 p2 = {0, 1, 0};

    Point3 result = spherical_interpolate(p1, p2, 0.5);

    EXPECT_NEAR(result.x(), M_SQRT2 / 2, 1e-10);
    EXPECT_NEAR(result.y(), M_SQRT2 / 2, 1e-10);
    EXPECT_NEAR(result.z(), 0.0, 1e-10);
}

TEST(SphericalInterpolateTest, ReturnsStartPointAtT0) {
    Point3 p1 = {1, 0, 0};
    Point3 p2 = {0, 1, 0};

    Point3 result = spherical_interpolate(p1, p2, 0.0);

    EXPECT_NEAR(result.x(), 1.0, 1e-10);
    EXPECT_NEAR(result.y(), 0.0, 1e-10);
    EXPECT_NEAR(result.z(), 0.0, 1e-10);
}

TEST(SphericalInterpolateTest, ReturnsEndPointAtT1) {
    Point3 p1 = {1, 0, 0};
    Point3 p2 = {0, 1, 0};

    Point3 result = spherical_interpolate(p1, p2, 1.0);

    EXPECT_NEAR(result.x(), 0.0, 1e-10);
    EXPECT_NEAR(result.y(), 1.0, 1e-10);
    EXPECT_NEAR(result.z(), 0.0, 1e-10);
}

TEST(ThetaTest, ComputesAngleInFirstQuadrant) {
    double angle = theta(1.0, 1.0);
    EXPECT_DOUBLE_EQ(angle, M_PI_4);
}

TEST(ThetaTest, ComputesAngleInSecondQuadrant) {
    double angle = theta(-1.0, 1.0);
    EXPECT_DOUBLE_EQ(angle, 3.0 * M_PI_4);
}

TEST(ThetaTest, ComputesAngleInThirdQuadrant) {
    double angle = theta(-1.0, -1.0);
    EXPECT_DOUBLE_EQ(angle, 5.0 * M_PI_4);
}

TEST(ThetaTest, ComputesAngleInFourthQuadrant) {
    double angle = theta(1.0, -1.0);
    EXPECT_DOUBLE_EQ(angle, 7.0 * M_PI_4);
}

TEST(ThetaTest, HandlesPositiveXAxis) {
    double angle = theta(1.0, 0.0);
    EXPECT_DOUBLE_EQ(angle, 0.0);
}

TEST(ThetaTest, HandlesPositiveYAxis) {
    double angle = theta(0.0, 1.0);
    EXPECT_DOUBLE_EQ(angle, M_PI_2);
}

TEST(ProjectToSphereTest, ProjectsPointToUnitSphere) {
    Point3 p = {2, 0, 0};
    Point3 result = project_to_sphere(p);

    EXPECT_DOUBLE_EQ(result.x(), 1.0);
    EXPECT_DOUBLE_EQ(result.y(), 0.0);
    EXPECT_DOUBLE_EQ(result.z(), 0.0);
}

TEST(ProjectToSphereTest, ProjectsToCustomRadius) {
    Point3 p = {1, 0, 0};
    Point3 result = project_to_sphere(p, ORIGIN, 2.0);

    EXPECT_DOUBLE_EQ(result.x(), 2.0);
    EXPECT_DOUBLE_EQ(result.y(), 0.0);
    EXPECT_DOUBLE_EQ(result.z(), 0.0);
}

TEST(ProjectToSphereTest, PreservesDirection) {
    Point3 p = {3, 4, 0};
    Point3 result = project_to_sphere(p);

    EXPECT_NEAR(result.x(), 0.6, 1e-10);
    EXPECT_NEAR(result.y(), 0.8, 1e-10);
    EXPECT_NEAR(result.z(), 0.0, 1e-10);
}

TEST(ProjectToSphereTest, WorksWithCustomCenter) {
    Point3 center = {1, 0, 0};
    Point3 p = {2, 0, 0};
    Point3 result = project_to_sphere(p, center, 2.0);

    EXPECT_DOUBLE_EQ(result.x(), 3.0);
    EXPECT_DOUBLE_EQ(result.y(), 0.0);
    EXPECT_DOUBLE_EQ(result.z(), 0.0);
}

TEST(AntipodalTest, ComputesOppositePoint) {
    Point3 p = {1, 0, 0};
    Point3 result = antipodal(p);

    EXPECT_DOUBLE_EQ(result.x(), -1.0);
    EXPECT_DOUBLE_EQ(result.y(), 0.0);
    EXPECT_DOUBLE_EQ(result.z(), 0.0);
}

TEST(AntipodalTest, WorksForArbitraryPoint) {
    Point3 p = {0.6, 0.8, 0.0};
    Point3 result = antipodal(p);

    EXPECT_DOUBLE_EQ(result.x(), -0.6);
    EXPECT_DOUBLE_EQ(result.y(), -0.8);
    EXPECT_DOUBLE_EQ(result.z(), 0.0);
}

TEST(BuildTangentBasisTest, ProducesOrthonormalBasis) {
    Vector3 normal = {0, 0, 1};
    TangentBasis basis = build_tangent_basis(normal);

    EXPECT_NEAR(CGAL::scalar_product(basis.tangent_u, normal), 0.0, 1e-10);
    EXPECT_NEAR(CGAL::scalar_product(basis.tangent_v, normal), 0.0, 1e-10);
    EXPECT_NEAR(CGAL::scalar_product(basis.tangent_u, basis.tangent_v), 0.0, 1e-10);

    EXPECT_NEAR(basis.tangent_u.squared_length(), 1.0, 1e-10);
    EXPECT_NEAR(basis.tangent_v.squared_length(), 1.0, 1e-10);
}

TEST(BuildTangentBasisTest, HandlesNormalAlongZAxis) {
    Vector3 normal = {0, 0, 1};
    TangentBasis basis = build_tangent_basis(normal);

    EXPECT_NEAR(CGAL::scalar_product(basis.tangent_u, normal), 0.0, 1e-10);
    EXPECT_NEAR(CGAL::scalar_product(basis.tangent_v, normal), 0.0, 1e-10);
}

TEST(BuildTangentBasisTest, HandlesArbitraryNormal) {
    Vector3 normal = normalize(Vector3(1, 1, 1));
    TangentBasis basis = build_tangent_basis(normal);

    EXPECT_NEAR(CGAL::scalar_product(basis.tangent_u, normal), 0.0, 1e-10);
    EXPECT_NEAR(CGAL::scalar_product(basis.tangent_v, normal), 0.0, 1e-10);
    EXPECT_NEAR(CGAL::scalar_product(basis.tangent_u, basis.tangent_v), 0.0, 1e-10);
}

TEST(BuildTangentBasisTest, ProducesRightHandedBasis) {
    Vector3 normal = {0, 0, 1};
    TangentBasis basis = build_tangent_basis(normal);

    Vector3 cross = CGAL::cross_product(basis.tangent_u, basis.tangent_v);
    Vector3 normalized_cross = normalize(cross);

    EXPECT_NEAR(normalized_cross.x(), normal.x(), 1e-10);
    EXPECT_NEAR(normalized_cross.y(), normal.y(), 1e-10);
    EXPECT_NEAR(normalized_cross.z(), normal.z(), 1e-10);
}

TEST(PositionVectorTest, ConvertsPointToVector) {
    Point3 p = {1, 2, 3};
    Vector3 v = position_vector(p);

    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(v.y(), 2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.0);
}

TEST(GeometryHelpersSphericalAngleTest, FindsTheAngleBetweenPointsOnASphere) {
    Vector3 a = {1, 0, 0};
    Vector3 b = {0, 1, 0};
    Vector3 c = {0, 0, 1};

    double angle = spherical_angle(a, b, c);

    EXPECT_DOUBLE_EQ(angle, M_PI_2);
}

TEST(GeometryHelpersSphericalAngleTest, WorksForAcuteAngles) {
    Vector3 a = {1, 0, 0};
    Vector3 b = {0, 1, 0};
    Vector3 c = {M_SQRT2 / 2, 0, M_SQRT2 / 2};

    double angle = spherical_angle(a, b, c);

    EXPECT_DOUBLE_EQ(angle, M_PI_4);
}

TEST(GeometryHelpersSphericalAngleTest, WorksForObtuseAngles) {
    Vector3 a = {1, 0, 0};
    Vector3 b = {0, 1, 0};
    Vector3 c = {-(M_SQRT2 / 2), 0, M_SQRT2 / 2};

    double angle = spherical_angle(a, b, c);

    EXPECT_DOUBLE_EQ(angle, 3 * M_PI_4);
}