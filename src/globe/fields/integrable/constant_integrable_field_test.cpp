#include <gtest/gtest.h>
#include "constant_integrable_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../testing/arc_factory.hpp"
#include "../../types.hpp"

using namespace globe;
using globe::testing::make_arc;

namespace {

SphericalPolygon make_octant() {
    return SphericalPolygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1)),
        make_arc(Vector3(0, 1, 0), Point3(0, 0, 1), Point3(1, 0, 0)),
    });
}

} // namespace

TEST(ConstantIntegrableFieldTest, IntegralEqualsValueTimesArea) {
    ConstantIntegrableField field(2.0);
    SphericalPolygon octant = make_octant();

    double expected_area = M_PI / 2.0;
    EXPECT_NEAR(field.integrate(octant), 2.0 * expected_area, 1e-9);
}

TEST(ConstantIntegrableFieldTest, IntegralOfUnitFieldEqualsArea) {
    ConstantIntegrableField field(1.0);
    SphericalPolygon octant = make_octant();

    EXPECT_NEAR(field.integrate(octant), octant.area(), 1e-9);
}

TEST(ConstantIntegrableFieldTest, TotalIntegralEqualsValueTimesUnitSphereArea) {
    ConstantIntegrableField field(3.0);

    EXPECT_NEAR(field.integrate(), 3.0 * UNIT_SPHERE_AREA, 1e-9);
}

TEST(ConstantIntegrableFieldTest, DefaultValueIsOne) {
    ConstantIntegrableField field;
    SphericalPolygon octant = make_octant();

    EXPECT_NEAR(field.integrate(octant), octant.area(), 1e-9);
}

TEST(ConstantIntegrableFieldTest, MaxFrequencyIsZero) {
    ConstantIntegrableField field(5.0);

    EXPECT_DOUBLE_EQ(field.max_frequency(), 0.0);
}
