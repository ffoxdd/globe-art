#include <gtest/gtest.h>
#include "linear_spherical_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../testing/arc_factory.hpp"

using namespace globe;
using globe::testing::make_arc;

TEST(LinearSphericalFieldTest, ValueReturnsLinearFunctionOfZ) {
    LinearSphericalField field(2.0, 1.0);

    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, 0)), 1.0);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, 1)), 3.0);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, -1)), -1.0);
    EXPECT_DOUBLE_EQ(field.value(Point3(1, 0, 0.5)), 2.0);
}

TEST(LinearSphericalFieldTest, DefaultParametersGiveIdentityOnZ) {
    LinearSphericalField field;

    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, 0)), 0.0);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, 1)), 1.0);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, -1)), -1.0);
}

TEST(LinearSphericalFieldTest, MassUsesFirstMomentZ) {
    LinearSphericalField field(2.0, 3.0);

    SphericalPolygon polygon(std::vector<SphericalArc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
    });

    double expected = 2.0 * polygon.first_moment().z() + 3.0 * polygon.area();
    EXPECT_DOUBLE_EQ(field.mass(polygon), expected);
}

TEST(LinearSphericalFieldTest, EdgeIntegralUsesFirstMomentZ) {
    LinearSphericalField field(2.0, 3.0);

    SphericalArc arc = make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0));

    double expected = 2.0 * arc.first_moment().z() + 3.0 * arc.length();
    EXPECT_DOUBLE_EQ(field.edge_integral(arc), expected);
}

TEST(LinearSphericalFieldTest, TotalMassOnlyIncludesOffset) {
    LinearSphericalField field(2.0, 3.0);

    double expected = 3.0 * UNIT_SPHERE_AREA;
    EXPECT_DOUBLE_EQ(field.total_mass(), expected);
}

TEST(LinearSphericalFieldTest, TotalMassIsZeroWhenOffsetIsZero) {
    LinearSphericalField field(5.0, 0.0);

    EXPECT_DOUBLE_EQ(field.total_mass(), 0.0);
}

TEST(LinearSphericalFieldTest, SatisfiesSphericalFieldConcept) {
    static_assert(SphericalField<LinearSphericalField>);
}
