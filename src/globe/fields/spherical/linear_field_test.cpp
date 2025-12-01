#include <gtest/gtest.h>
#include "linear_field.hpp"
#include "../../geometry/spherical/polygon/polygon.hpp"
#include "../../geometry/spherical/arc.hpp"

using namespace globe::fields::spherical;
using globe::Arc;
using globe::Polygon;
using globe::VectorS2;
using globe::geometry::spherical::UNIT_SPHERE_AREA;

TEST(LinearFieldTest, ValueReturnsLinearFunctionOfZ) {
    LinearField field(2.0, 1.0);

    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, 0)), 1.0);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, 1)), 3.0);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, -1)), -1.0);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(1, 0, 0.5)), 2.0);
}

TEST(LinearFieldTest, DefaultParametersGiveIdentityOnZ) {
    LinearField field;

    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, 0)), 0.0);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, 1)), 1.0);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, -1)), -1.0);
}

TEST(LinearFieldTest, MassUsesFirstMomentZ) {
    LinearField field(2.0, 3.0);

    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
    });

    double expected = 2.0 * polygon.first_moment().z() + 3.0 * polygon.area();
    EXPECT_DOUBLE_EQ(field.mass(polygon), expected);
}

TEST(LinearFieldTest, EdgeIntegralUsesFirstMomentZ) {
    LinearField field(2.0, 3.0);

    Arc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1));

    double expected = 2.0 * arc.first_moment().z() + 3.0 * arc.length();
    EXPECT_DOUBLE_EQ(field.edge_integral(arc), expected);
}

TEST(LinearFieldTest, TotalMassOnlyIncludesOffset) {
    LinearField field(2.0, 3.0);

    double expected = 3.0 * UNIT_SPHERE_AREA;
    EXPECT_DOUBLE_EQ(field.total_mass(), expected);
}

TEST(LinearFieldTest, TotalMassIsZeroWhenOffsetIsZero) {
    LinearField field(5.0, 0.0);

    EXPECT_DOUBLE_EQ(field.total_mass(), 0.0);
}

TEST(LinearFieldTest, SatisfiesFieldConcept) {
    static_assert(Field<LinearField>);
}
