#include <gtest/gtest.h>
#include "random_point_generator.hpp"
#include "../math/bounding_box.hpp"
#include "../math/interval.hpp"
#include "../testing/geometric_assertions.hpp"

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

TEST(RandomPointGeneratorTest, EXPENSIVE_UniformDistributionInUnitCube) {
    SKIP_IF_EXPENSIVE();

    RandomPointGenerator generator;
    constexpr size_t sample_count = 10000;

    double x_sum = 0.0;
    double y_sum = 0.0;
    double z_sum = 0.0;

    for (size_t i = 0; i < sample_count; ++i) {
        Point3 point = generator.generate();

        EXPECT_GE(point.x(), -1.0);
        EXPECT_LE(point.x(), 1.0);
        EXPECT_GE(point.y(), -1.0);
        EXPECT_LE(point.y(), 1.0);
        EXPECT_GE(point.z(), -1.0);
        EXPECT_LE(point.z(), 1.0);

        x_sum += point.x();
        y_sum += point.y();
        z_sum += point.z();
    }

    double x_mean = x_sum / sample_count;
    double y_mean = y_sum / sample_count;
    double z_mean = z_sum / sample_count;

    EXPECT_NEAR(x_mean, 0.0, 0.05);
    EXPECT_NEAR(y_mean, 0.0, 0.05);
    EXPECT_NEAR(z_mean, 0.0, 0.05);
}

TEST(RandomPointGeneratorTest, EXPENSIVE_UniformDistributionInCustomBox) {
    SKIP_IF_EXPENSIVE();

    RandomPointGenerator generator;
    BoundingBox box(Interval(0.0, 1.0), Interval(0.0, 2.0), Interval(-0.5, 0.5));
    constexpr size_t sample_count = 10000;

    double x_sum = 0.0;
    double y_sum = 0.0;
    double z_sum = 0.0;

    for (size_t i = 0; i < sample_count; ++i) {
        Point3 point = generator.generate(box);

        EXPECT_GE(point.x(), 0.0);
        EXPECT_LE(point.x(), 1.0);
        EXPECT_GE(point.y(), 0.0);
        EXPECT_LE(point.y(), 2.0);
        EXPECT_GE(point.z(), -0.5);
        EXPECT_LE(point.z(), 0.5);

        x_sum += point.x();
        y_sum += point.y();
        z_sum += point.z();
    }

    double x_mean = x_sum / sample_count;
    double y_mean = y_sum / sample_count;
    double z_mean = z_sum / sample_count;

    EXPECT_NEAR(x_mean, 0.5, 0.05);
    EXPECT_NEAR(y_mean, 1.0, 0.05);
    EXPECT_NEAR(z_mean, 0.0, 0.05);
}
