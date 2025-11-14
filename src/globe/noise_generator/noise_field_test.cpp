#include <gtest/gtest.h>
#include "noise_field.hpp"

using namespace globe;

TEST(NoiseFieldTest, ValueMethodReturnsDouble) {
    Point3 location = {0.1, 0.2, 0.3};
    NoiseField noise_field;

    double value = noise_field.value(location);

    EXPECT_TRUE(typeid(value) == typeid(double));
}

TEST(NoiseFieldTest, ValueMethodReturnsConsistentResult) {
    Point3 location = {0.1, 0.2, 0.3};

    std::vector<Point3> points;
    points.emplace_back(location);
    points.emplace_back(0.4, 0.5, 0.6);

    NoiseField noise_field;

    double value1 = noise_field.value(location);
    double value2 = noise_field.value(location);

    EXPECT_DOUBLE_EQ(value1, value2);
}

TEST(NoiseFieldTest, ValueMethodDifferentLocations) {
    Point3 location1 = {0.1, 0.2, 0.3};
    Point3 location2 = {0.4, 0.5, 0.6};

    std::vector<Point3> points;
    points.emplace_back(location1);
    points.emplace_back(location2);

    NoiseField noise_field;

    double value1 = noise_field.value(location1);
    double value2 = noise_field.value(location2);

    EXPECT_NE(value1, value2);
}

TEST(NoiseFieldTest, CanNormalizeOutputOverSamplePoints) {
    std::vector<Point3> sample_points;
    sample_points.emplace_back(0.1, 0.2, 0.3);
    sample_points.emplace_back(0.4, 0.5, 0.6);
    sample_points.emplace_back(0.7, 0.8, 0.9);

    Interval output_range = Interval(-0.01, 0.02);

    NoiseField noise_field;
    noise_field.normalize(sample_points, output_range);

    for (auto &point : sample_points) {
        double value = noise_field.value(point);

        EXPECT_GE(value, output_range.low());
        EXPECT_LE(value, output_range.high());
    }
}

