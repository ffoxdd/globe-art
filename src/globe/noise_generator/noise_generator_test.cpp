#include <gtest/gtest.h>
#include "noise_generator.h"

using namespace globe;

TEST(NoiseGeneratorTest, ValueMethodReturnsDouble) {
    NoiseGenerator generator;
    Point3 location = {1.0, 2.0, 3.0};

    double value = generator.value(location);

    EXPECT_TRUE(typeid(value) == typeid(double));
}

TEST(NoiseGeneratorTest, ValueMethodReturnsConsistentResult) {
    NoiseGenerator generator;
    Point3 location = {1.0, 2.0, 3.0};

    double value1 = generator.value(location);
    double value2 = generator.value(location);

    EXPECT_DOUBLE_EQ(value1, value2);
}

TEST(NoiseGeneratorTest, ValueMethodDifferentLocations) {
    NoiseGenerator generator;
    Point3 location1 = {0.1, 0.2, 0.3};
    Point3 location2 = {0.4, 0.5, 0.6};

    double value1 = generator.value(location1);
    double value2 = generator.value(location2);

    EXPECT_NE(value1, value2);
}
