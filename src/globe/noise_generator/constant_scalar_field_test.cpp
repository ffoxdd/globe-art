#include <gtest/gtest.h>
#include "constant_scalar_field.hpp"

using namespace globe;

TEST(ConstantScalarFieldTest, ReturnsConfiguredValue) {
    ConstantScalarField scalar_field(1.0);
    EXPECT_DOUBLE_EQ(scalar_field.value(Point3(0, 0, 0)), 1.0);

    ConstantScalarField other_field(2.5);
    EXPECT_DOUBLE_EQ(other_field.value(Point3(1, 0, 0)), 2.5);
}

TEST(ConstantScalarFieldTest, NormalizeCentersValueWithinInterval) {
    ConstantScalarField scalar_field(5.0);
    std::vector<Point3> sample_points{
        Point3(0.1, 0.2, 0.3),
        Point3(0.4, 0.5, 0.6)
    };

    Interval interval(-1.0, 1.0);
    scalar_field.normalize(sample_points, interval);

    EXPECT_DOUBLE_EQ(scalar_field.value(Point3(0.7, 0.8, 0.9)), 0.0);

    Interval positive_interval(2.0, 4.0);
    scalar_field.normalize(sample_points, positive_interval);
    EXPECT_DOUBLE_EQ(scalar_field.value(Point3(0.7, 0.8, 0.9)), 3.0);
}

