#include "constant_spherical_field.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace globe {
namespace {

TEST(ConstantSphericalFieldTest, ValueIsConstant) {
    ConstantSphericalField field(2.5);

    EXPECT_DOUBLE_EQ(field.value(Point3(1, 0, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 1, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, 1)), 2.5);
}

TEST(ConstantSphericalFieldTest, MassIsValueTimesArea) {
    ConstantSphericalField field(3.0);

    PolygonMoments moments{
        .area = 2.0,
        .first_moment = Eigen::Vector3d::Zero(),
        .second_moment = Eigen::Matrix3d::Zero()
    };

    EXPECT_DOUBLE_EQ(field.mass(moments), 6.0);
}

TEST(ConstantSphericalFieldTest, EdgeIntegralIsValueTimesLength) {
    ConstantSphericalField field(4.0);

    ArcMoments moments{
        .length = 1.5,
        .first_moment = Eigen::Vector3d::Zero(),
        .second_moment = Eigen::Matrix3d::Zero()
    };

    EXPECT_DOUBLE_EQ(field.edge_integral(moments), 6.0);
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

}
}
