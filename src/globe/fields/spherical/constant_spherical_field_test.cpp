#include "constant_spherical_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../testing/arc_factory.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace globe;
using globe::testing::make_arc;

TEST(ConstantSphericalFieldTest, ValueIsConstant) {
    ConstantSphericalField field(2.5);

    EXPECT_DOUBLE_EQ(field.value(Point3(1, 0, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 1, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, 1)), 2.5);
}

TEST(ConstantSphericalFieldTest, MassIsValueTimesArea) {
    ConstantSphericalField field(3.0);

    SphericalPolygon polygon(std::vector<SphericalArc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
    });

    double expected = 3.0 * polygon.area();
    EXPECT_DOUBLE_EQ(field.mass(polygon), expected);
}

TEST(ConstantSphericalFieldTest, EdgeIntegralIsValueTimesLength) {
    ConstantSphericalField field(4.0);

    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    double expected = 4.0 * arc.length();
    EXPECT_DOUBLE_EQ(field.edge_integral(arc), expected);
}

TEST(ConstantSphericalFieldTest, TotalMassIsValueTimesSphereArea) {
    ConstantSphericalField field(2.0);

    EXPECT_DOUBLE_EQ(field.total_mass(), 2.0 * 4.0 * M_PI);
}

TEST(ConstantSphericalFieldTest, DefaultValueIsOne) {
    ConstantSphericalField field;

    EXPECT_DOUBLE_EQ(field.value(Point3(1, 0, 0)), 1.0);
    EXPECT_DOUBLE_EQ(field.total_mass(), 4.0 * M_PI);
}

TEST(ConstantSphericalFieldTest, SatisfiesSphericalFieldConcept) {
    static_assert(SphericalField<ConstantSphericalField>);
}
