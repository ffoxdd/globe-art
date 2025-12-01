#include <gtest/gtest.h>
#include "monte_carlo_spherical_field.hpp"
#include "../scalar/constant_scalar_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../testing/geometric_assertions.hpp"

using namespace globe;

TEST(MonteCarloSphericalFieldTest, SatisfiesSphericalFieldConcept) {
    static_assert(SphericalField<MonteCarloSphericalField<ConstantScalarField>>);
}

TEST(MonteCarloSphericalFieldTest, ValueDelegatesToUnderlyingField) {
    ConstantScalarField scalar_field(2.5);
    MonteCarloSphericalField field(scalar_field);

    EXPECT_DOUBLE_EQ(field.value(VectorS2(1, 0, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, 1)), 2.5);
}

TEST(MonteCarloSphericalFieldTest, EXPENSIVE_TotalMassConvergesToExpected) {
    REQUIRE_EXPENSIVE();

    ConstantScalarField scalar_field(1.0);
    MonteCarloSphericalField field(scalar_field);

    double expected = UNIT_SPHERE_AREA;
    double tolerance = expected * 0.02;
    EXPECT_NEAR(field.total_mass(), expected, tolerance);
}

TEST(MonteCarloSphericalFieldTest, EXPENSIVE_MassComputesForHemisphere) {
    REQUIRE_EXPENSIVE();

    ConstantScalarField scalar_field(1.0);
    MonteCarloSphericalField field(scalar_field);

    SphericalPolygon hemisphere(std::vector<SphericalArc>{
        SphericalArc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        SphericalArc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
    });

    double expected = 2.0 * M_PI;
    double tolerance = expected * 0.05;
    EXPECT_NEAR(field.mass(hemisphere), expected, tolerance);
}

TEST(MonteCarloSphericalFieldTest, EXPENSIVE_EdgeIntegralConverges) {
    REQUIRE_EXPENSIVE();

    ConstantScalarField scalar_field(3.0);
    MonteCarloSphericalField field(scalar_field);

    SphericalArc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1));
    double arc_length = arc.length();

    double expected = 3.0 * arc_length;
    double tolerance = expected * 0.02;
    EXPECT_NEAR(field.edge_integral(arc), expected, tolerance);
}

TEST(MonteCarloSphericalFieldTest, EXPENSIVE_EdgeGradientIntegralConverges) {
    REQUIRE_EXPENSIVE();

    ConstantScalarField scalar_field(2.0);
    MonteCarloSphericalField field(scalar_field);

    SphericalArc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1));

    Eigen::Vector3d gradient_integral = field.edge_gradient_integral(arc);
    EXPECT_GT(gradient_integral.norm(), 0.0);
}
