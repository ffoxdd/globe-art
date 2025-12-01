#include "constant_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace globe::fields::spherical;
using globe::SphericalArc;
using globe::SphericalPolygon;
using globe::VectorS2;

TEST(ConstantFieldTest, ValueIsConstant) {
    ConstantField field(2.5);

    EXPECT_DOUBLE_EQ(field.value(VectorS2(1, 0, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 1, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, 1)), 2.5);
}

TEST(ConstantFieldTest, MassIsValueTimesArea) {
    ConstantField field(3.0);

    SphericalPolygon polygon(std::vector<SphericalArc>{
        SphericalArc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
    });

    double expected = 3.0 * polygon.area();
    EXPECT_DOUBLE_EQ(field.mass(polygon), expected);
}

TEST(ConstantFieldTest, EdgeIntegralIsValueTimesLength) {
    ConstantField field(4.0);

    SphericalArc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1));

    double expected = 4.0 * arc.length();
    EXPECT_DOUBLE_EQ(field.edge_integral(arc), expected);
}

TEST(ConstantFieldTest, TotalMassIsValueTimesSphereArea) {
    ConstantField field(2.0);

    EXPECT_DOUBLE_EQ(field.total_mass(), 2.0 * 4.0 * M_PI);
}

TEST(ConstantFieldTest, DefaultValueIsOne) {
    ConstantField field;

    EXPECT_DOUBLE_EQ(field.value(VectorS2(1, 0, 0)), 1.0);
    EXPECT_DOUBLE_EQ(field.total_mass(), 4.0 * M_PI);
}

TEST(ConstantFieldTest, SatisfiesFieldConcept) {
    static_assert(Field<ConstantField>);
}
