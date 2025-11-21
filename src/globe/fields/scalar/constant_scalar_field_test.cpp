#include <gtest/gtest.h>
#include "constant_scalar_field.hpp"

using namespace globe;

TEST(ConstantScalarFieldTest, ReturnsConfiguredValue) {
    ConstantScalarField scalar_field(1.0);
    EXPECT_DOUBLE_EQ(scalar_field.value(Point3(0, 0, 0)), 1.0);

    ConstantScalarField other_field(2.5);
    EXPECT_DOUBLE_EQ(other_field.value(Point3(1, 0, 0)), 2.5);
}

