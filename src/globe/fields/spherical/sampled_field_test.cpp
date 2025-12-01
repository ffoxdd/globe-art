#include <gtest/gtest.h>
#include "sampled_field.hpp"
#include "../scalar/constant_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"

using namespace globe::fields::spherical;
using globe::fields::scalar::ConstantField;
using globe::RandomSpherePointGenerator;
using globe::SphericalArc;
using globe::SphericalPolygon;
using globe::VectorS2;
using globe::UNIT_SPHERE_AREA;

TEST(SampledFieldTest, SatisfiesFieldConcept) {
    static_assert(Field<SampledField<ConstantField>>);
}

TEST(SampledFieldTest, ValueDelegatesToUnderlyingField) {
    ConstantField scalar_field(2.5);
    SampledField field(scalar_field);

    EXPECT_DOUBLE_EQ(field.value(VectorS2(1, 0, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, 1)), 2.5);
}

TEST(SampledFieldTest, TotalMassApproximatesExpected) {
    ConstantField scalar_field(1.0);
    SampledField field(scalar_field);

    double expected = UNIT_SPHERE_AREA;
    double tolerance = expected * 0.01;
    EXPECT_NEAR(field.total_mass(), expected, tolerance);
}

TEST(SampledFieldTest, MassReturnsZeroForEmptySamples) {
    ConstantField scalar_field(1.0);
    SampledField field(
        scalar_field,
        RandomSpherePointGenerator<>(),
        0
    );

    SphericalPolygon polygon(std::vector<SphericalArc>{
        SphericalArc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
    });

    EXPECT_DOUBLE_EQ(field.mass(polygon), 0.0);
}
