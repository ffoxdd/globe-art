#include <gtest/gtest.h>
#include "helpers.hpp"
#include "../types.hpp"
#include <cmath>

using namespace globe;

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