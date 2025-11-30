#include <gtest/gtest.h>
#include "sampled_spherical_field.hpp"
#include "../scalar/constant_scalar_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../testing/arc_factory.hpp"

using namespace globe;
using globe::testing::make_arc;

TEST(SampledSphericalFieldTest, SatisfiesSphericalFieldConcept) {
    static_assert(SphericalField<SampledSphericalField<ConstantScalarField>>);
}

TEST(SampledSphericalFieldTest, ValueDelegatesToUnderlyingField) {
    ConstantScalarField scalar_field(2.5);
    SampledSphericalField field(scalar_field);

    EXPECT_DOUBLE_EQ(field.value(Point3(1, 0, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(Point3(0, 0, 1)), 2.5);
}

TEST(SampledSphericalFieldTest, TotalMassApproximatesExpected) {
    ConstantScalarField scalar_field(1.0);
    SampledSphericalField field(scalar_field);

    double expected = UNIT_SPHERE_AREA;
    double tolerance = expected * 0.01;
    EXPECT_NEAR(field.total_mass(), expected, tolerance);
}

TEST(SampledSphericalFieldTest, MassReturnsZeroForEmptySamples) {
    ConstantScalarField scalar_field(1.0);
    SampledSphericalField field(
        scalar_field,
        RandomSpherePointGenerator<>(),
        0
    );

    SphericalPolygon polygon(std::vector<SphericalArc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
    });

    EXPECT_DOUBLE_EQ(field.mass(polygon), 0.0);
}
