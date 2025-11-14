#include <gtest/gtest.h>
#include "anl_scalar_field.hpp"

using namespace globe;

TEST(NoiseGeneratorTest, ValueMethodReturnsDouble) {
    Point3 location = {0.1, 0.2, 0.3};
    AnlScalarField noise_generator;

    double value = noise_generator.value(location);

    EXPECT_TRUE(typeid(value) == typeid(double));
}

TEST(NoiseGeneratorTest, ValueMethodReturnsConsistentResult) {
    Point3 location = {0.1, 0.2, 0.3};

    std::vector<Point3> points;
    points.emplace_back(location);
    points.emplace_back(0.4, 0.5, 0.6);

    AnlScalarField noise_generator;

    double value1 = noise_generator.value(location);
    double value2 = noise_generator.value(location);

    EXPECT_DOUBLE_EQ(value1, value2);
}

TEST(NoiseGeneratorTest, ValueMethodDifferentLocations) {
    Point3 location1 = {0.1, 0.2, 0.3};
    Point3 location2 = {0.4, 0.5, 0.6};

    std::vector<Point3> points;
    points.emplace_back(location1);
    points.emplace_back(location2);

    AnlScalarField noise_generator;

    double value1 = noise_generator.value(location1);
    double value2 = noise_generator.value(location2);

    EXPECT_NE(value1, value2);
}

TEST(NoiseGeneratorTest, CanNormalizeOutputOverSamplePoints) {
    std::vector<Point3> sample_points;
    sample_points.emplace_back(0.1, 0.2, 0.3);
    sample_points.emplace_back(0.4, 0.5, 0.6);
    sample_points.emplace_back(0.7, 0.8, 0.9);

    Interval output_range = Interval(-0.01, 0.02);

    AnlScalarField noise_generator;
    noise_generator.normalize(sample_points, output_range);

    for (auto &point : sample_points) {
        double value = noise_generator.value(point);

        EXPECT_GE(value, output_range.low());
        EXPECT_LE(value, output_range.high());
    }
}
