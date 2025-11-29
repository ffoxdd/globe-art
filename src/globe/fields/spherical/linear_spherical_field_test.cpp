#include <gtest/gtest.h>
#include "linear_spherical_field.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"

using namespace globe;

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

    PolygonMoments moments;
    moments.area = 0.5;
    moments.first_moment = Eigen::Vector3d(0.1, 0.2, 0.3);
    moments.second_moment = Eigen::Matrix3d::Zero();

    double expected = 2.0 * 0.3 + 3.0 * 0.5;
    EXPECT_DOUBLE_EQ(field.mass(moments), expected);
}

TEST(LinearSphericalFieldTest, EdgeIntegralUsesFirstMomentZ) {
    LinearSphericalField field(2.0, 3.0);

    ArcMoments moments;
    moments.length = 0.4;
    moments.first_moment = Eigen::Vector3d(0.1, 0.2, 0.3);
    moments.second_moment = Eigen::Matrix3d::Zero();

    double expected = 2.0 * 0.3 + 3.0 * 0.4;
    EXPECT_DOUBLE_EQ(field.edge_integral(moments), expected);
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
