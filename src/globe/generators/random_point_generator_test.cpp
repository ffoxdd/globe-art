#include <gtest/gtest.h>
#include "random_point_generator.hpp"
#include "../math/bounding_box.hpp"
#include "../math/interval.hpp"

using namespace globe;

TEST(RandomPointGeneratorTest, GenerateWithoutBoundingBoxReturnsPointInUnitCube) {
    RandomPointGenerator generator;

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate();

        EXPECT_GE(point.x(), -1.0);
        EXPECT_LE(point.x(), 1.0);
        EXPECT_GE(point.y(), -1.0);
        EXPECT_LE(point.y(), 1.0);
        EXPECT_GE(point.z(), -1.0);
        EXPECT_LE(point.z(), 1.0);
    }
}

TEST(RandomPointGeneratorTest, GenerateWithBoundingBoxReturnsPointInBox) {
    RandomPointGenerator generator;
    BoundingBox box(Interval(0.0, 1.0), Interval(0.0, 2.0), Interval(-0.5, 0.5));

    for (int i = 0; i < 10; ++i) {
        Point3 point = generator.generate(box);

        EXPECT_GE(point.x(), 0.0);
        EXPECT_LE(point.x(), 1.0);
        EXPECT_GE(point.y(), 0.0);
        EXPECT_LE(point.y(), 2.0);
        EXPECT_GE(point.z(), -0.5);
        EXPECT_LE(point.z(), 0.5);
    }
}
