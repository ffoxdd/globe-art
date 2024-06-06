#include <gtest/gtest.h>
#include "noise_generator.h"

using namespace globe;

TEST(NoiseGeneratorTest, ValueMethodReturnsDouble) {
    Point3 location = {0.1, 0.2, 0.3};

    std::vector<Point3> points;
    points.emplace_back(location);

    NoiseGenerator generator(-1, 1, points);

    double value = generator.value(location);
    EXPECT_TRUE(typeid(value) == typeid(double));
}

TEST(NoiseGeneratorTest, ValueMethodReturnsConsistentResult) {
    Point3 location = {0.1, 0.2, 0.3};

    std::vector<Point3> points;
    points.emplace_back(location);
    points.emplace_back(Point3(0.4, 0.5, 0.6));

    NoiseGenerator generator(-1, 1, points);

    double value1 = generator.value(location);
    double value2 = generator.value(location);

    EXPECT_DOUBLE_EQ(value1, value2);
}

TEST(NoiseGeneratorTest, ValueMethodDifferentLocations) {
    Point3 location1 = {0.1, 0.2, 0.3};
    Point3 location2 = {0.4, 0.5, 0.6};

    std::vector<Point3> points;
    points.emplace_back(location1);
    points.emplace_back(location2);

    NoiseGenerator generator(-1, 1, points);

    double value1 = generator.value(location1);
    double value2 = generator.value(location2);

    EXPECT_NE(value1, value2);
}

TEST(NoiseGeneratorTest, CanNormalizeOutputOverSamplePoints) {
    std::vector<Point3> points;
    points.emplace_back(0.1, 0.2, 0.3);
    points.emplace_back(0.4, 0.5, 0.6);
    points.emplace_back(0.7, 0.8, 0.9);

    double low = -0.01;
    double high = 0.02;

    NoiseGenerator generator(low, high, points);

    for (auto & point : points) {
        double value = generator.value(point);

        EXPECT_GE(value, low);
        EXPECT_LE(value, high);
    }
}
