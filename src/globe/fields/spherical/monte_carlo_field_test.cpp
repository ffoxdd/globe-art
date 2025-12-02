#include <gtest/gtest.h>
#include "monte_carlo_field.hpp"
#include "../scalar/constant_field.hpp"
#include "../../geometry/spherical/polygon/polygon.hpp"
#include "../../geometry/spherical/arc.hpp"
#include "../../testing/assertions/geometric.hpp"
#include "../../testing/macros.hpp"

using namespace globe::fields::spherical;
using globe::fields::scalar::ConstantField;
using globe::Arc;
using globe::Polygon;
using globe::VectorS2;
using globe::geometry::spherical::UNIT_SPHERE_AREA;

TEST(MonteCarloFieldTest, SatisfiesFieldConcept) {
    static_assert(Field<MonteCarloField<ConstantField>>);
}

TEST(MonteCarloFieldTest, ValueDelegatesToUnderlyingField) {
    ConstantField scalar_field(2.5);
    MonteCarloField field(scalar_field);

    EXPECT_DOUBLE_EQ(field.value(VectorS2(1, 0, 0)), 2.5);
    EXPECT_DOUBLE_EQ(field.value(VectorS2(0, 0, 1)), 2.5);
}

TEST(MonteCarloFieldTest, EXPENSIVE_TotalMassConvergesToExpected) {
    REQUIRE_EXPENSIVE();

    ConstantField scalar_field(1.0);
    MonteCarloField field(scalar_field);

    double expected = UNIT_SPHERE_AREA;
    double tolerance = expected * 0.02;
    EXPECT_NEAR(field.total_mass(), expected, tolerance);
}

TEST(MonteCarloFieldTest, EXPENSIVE_MassComputesForHemisphere) {
    REQUIRE_EXPENSIVE();

    ConstantField scalar_field(1.0);
    MonteCarloField field(scalar_field);

    Polygon hemisphere(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
    });

    double expected = 2.0 * M_PI;
    double tolerance = expected * 0.05;
    EXPECT_NEAR(field.mass(hemisphere), expected, tolerance);
}

TEST(MonteCarloFieldTest, EXPENSIVE_EdgeIntegralConverges) {
    REQUIRE_EXPENSIVE();

    ConstantField scalar_field(3.0);
    MonteCarloField field(scalar_field);

    Arc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1));
    double arc_length = arc.length();

    double expected = 3.0 * arc_length;
    double tolerance = expected * 0.02;
    EXPECT_NEAR(field.edge_integral(arc), expected, tolerance);
}

TEST(MonteCarloFieldTest, EXPENSIVE_EdgeGradientIntegralConverges) {
    REQUIRE_EXPENSIVE();

    ConstantField scalar_field(2.0);
    MonteCarloField field(scalar_field);

    Arc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1));

    Eigen::Vector3d gradient_integral = field.edge_gradient_integral(arc);
    EXPECT_GT(gradient_integral.norm(), 0.0);
}
