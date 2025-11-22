#include <gtest/gtest.h>
#include "random_sphere_point_generator.hpp"
#include "../spherical/spherical_bounding_box.hpp"
#include "../testing/geometric_assertions.hpp"

using namespace globe;
using globe::testing::is_on_unit_sphere;

TEST(RandomSpherePointGeneratorTest, GenerateWithoutBoundingBoxReturnsPointOnSphere) {
    RandomSpherePointGenerator generator;

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate();

        EXPECT_TRUE(is_on_unit_sphere(point))
            << "Point (" << point.x() << ", " << point.y() << ", " << point.z()
            << ") is not on unit sphere";
    }
}

TEST(RandomSpherePointGeneratorTest, GenerateWithBoundingBoxReturnsPointInBox) {
    RandomSpherePointGenerator generator;
    SphericalBoundingBox box = SphericalBoundingBox::full_sphere();

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate(box);

        EXPECT_TRUE(is_on_unit_sphere(point));
        EXPECT_TRUE(box.contains(point));
    }
}
