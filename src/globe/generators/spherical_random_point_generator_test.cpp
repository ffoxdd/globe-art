#include <gtest/gtest.h>
#include "spherical_random_point_generator.hpp"
#include "../spherical/spherical_bounding_box.hpp"
#include <cmath>

using namespace globe;

const double TOLERANCE = 1e-9;

bool is_on_unit_sphere(const Point3 &point) {
    double distance = std::sqrt(
        point.x() * point.x() +
        point.y() * point.y() +
        point.z() * point.z()
    );

    return std::abs(distance - 1.0) < TOLERANCE;
}

TEST(SphericalRandomPointGeneratorTest, GenerateWithoutBoundingBoxReturnsPointOnSphere) {
    SphericalRandomPointGenerator generator;

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate();

        EXPECT_TRUE(is_on_unit_sphere(point))
            << "Point (" << point.x() << ", " << point.y() << ", " << point.z()
            << ") is not on unit sphere";
    }
}

TEST(SphericalRandomPointGeneratorTest, GenerateWithBoundingBoxReturnsPointInBox) {
    SphericalRandomPointGenerator generator;
    SphericalBoundingBox box = SphericalBoundingBox::full_sphere();

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate(box);

        EXPECT_TRUE(is_on_unit_sphere(point));
        EXPECT_TRUE(box.contains(point));
    }
}

