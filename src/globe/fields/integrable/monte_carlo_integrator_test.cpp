#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./monte_carlo_integrator.hpp"
#include "../scalar/mock_scalar_field.hpp"
#include "../scalar/constant_scalar_field.hpp"
#include "../../generators/random_sphere_point_generator.hpp"
#include "../../testing/test_fixtures.hpp"
#include "../../testing/geometric_assertions.hpp"
#include "../../testing/arc_factory.hpp"
#include <vector>

using namespace globe;
using globe::testing::SequencePointGenerator;
using globe::testing::make_arc;
using ::testing::Return;
using ::testing::_;

SphericalPolygon make_northern_hemisphere_polygon() {
    return SphericalPolygon(
        std::vector<Arc>{
            make_arc(0, 0, 1, 1, 0, 0, 0, 1, 0),
            make_arc(0, 0, 1, 0, 1, 0, -1, 0, 0),
            make_arc(0, 0, 1, -1, 0, 0, 0, -1, 0),
            make_arc(0, 0, 1, 0, -1, 0, 1, 0, 0),
        }
    );
}

SphericalBoundingBox hemisphere_bounding_box() {
    return {Interval(0, 2 * M_PI), Interval(0, 1)};
}


TEST(MonteCarloIntegratorTest, FiltersOutsidePoints) {
    MockScalarField mock_scalar_field;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 outside_point(0, 0, -1);
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_scalar_field, value(_)).WillRepeatedly(Return(1.0));

    MonteCarloIntegrator<MockScalarField, SequencePointGenerator> monte_carlo_integrator(
        std::ref(spherical_polygon),
        mock_scalar_field,
        SequencePointGenerator({outside_point, inside_point}),
        hemisphere_bounding_box()
    );

    double result = monte_carlo_integrator.integrate();
    EXPECT_NEAR(result, M_PI, 1e-6);
}

TEST(MonteCarloIntegratorTest, CalculatesAreaRatioFromSamples) {
    MockScalarField mock_scalar_field;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();

    std::vector<Point3> points = {
        Point3(0, 0, -1),
        Point3(0, 0, 1)
    };

    EXPECT_CALL(mock_scalar_field, value(_)).WillRepeatedly(Return(1.0));

    MonteCarloIntegrator<MockScalarField, SequencePointGenerator> monte_carlo_integrator(
        std::ref(spherical_polygon),
        mock_scalar_field,
        SequencePointGenerator(points),
        hemisphere_bounding_box()
    );

    double cell_mass = monte_carlo_integrator.integrate();
    EXPECT_NEAR(cell_mass, M_PI, 1e-6);
}

TEST(MonteCarloIntegratorTest, AppliesDensityWeighting) {
    MockScalarField mock_scalar_field;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_scalar_field, value(_)).WillRepeatedly(Return(2.0));

    MonteCarloIntegrator<MockScalarField, SequencePointGenerator> monte_carlo_integrator(
        std::ref(spherical_polygon),
        mock_scalar_field,
        SequencePointGenerator({inside_point}),
        hemisphere_bounding_box()
    );

    EXPECT_NEAR(
        monte_carlo_integrator.integrate(),
        4 * M_PI,
        1e-6
    );
}

TEST(MonteCarloIntegratorTest, ScalesIntegralByDensity) {
    MockScalarField mock_scalar_field;
    const Point3 arbitrary_point(0, 0, 1);

    EXPECT_CALL(mock_scalar_field, value(_)).WillRepeatedly(Return(3.0));

    MonteCarloIntegrator<MockScalarField, SequencePointGenerator> monte_carlo_integrator(
        std::nullopt,
        mock_scalar_field,
        SequencePointGenerator({arbitrary_point})
    );

    EXPECT_NEAR(
        monte_carlo_integrator.integrate(),
        12 * M_PI,
        1e-6
    );
}

TEST(MonteCarloIntegratorTest, EXPENSIVE_ConvergesWithManyRandomSamples) {
    REQUIRE_EXPENSIVE();

    ConstantScalarField scalar_field(1.0);
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();

    MonteCarloIntegrator<ConstantScalarField, RandomSpherePointGenerator<>> monte_carlo_integrator(
        std::ref(spherical_polygon),
        scalar_field,
        RandomSpherePointGenerator<>(),
        hemisphere_bounding_box()
    );

    double result = monte_carlo_integrator.integrate();
    EXPECT_NEAR(result, 2 * M_PI, 0.3);
}

TEST(MonteCarloIntegratorTest, EXPENSIVE_ConvergesForFullSphere) {
    REQUIRE_EXPENSIVE();

    ConstantScalarField scalar_field(1.0);

    MonteCarloIntegrator<ConstantScalarField, RandomSpherePointGenerator<>> monte_carlo_integrator(
        std::nullopt,
        scalar_field,
        RandomSpherePointGenerator<>()
    );

    double result = monte_carlo_integrator.integrate();
    EXPECT_NEAR(result, 4 * M_PI, 0.5);
}
