#include "gtest/gtest.h"
#include "./density_sampled_integrable_field.hpp"
#include "../scalar/constant_scalar_field.hpp"
#include "../../geometry/spherical/spherical_polygon.hpp"
#include "../../generators/random_sphere_point_generator.hpp"
#include "../../testing/test_fixtures.hpp"
#include "../../testing/geometric_assertions.hpp"
#include "../../testing/arc_factory.hpp"
#include <vector>

using namespace globe;
using globe::testing::SequencePointGenerator;
using globe::testing::make_arc;

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

TEST(DensitySampledIntegrableFieldTest, AcceptsAllPointsWithConstantDensity) {
    ConstantScalarField scalar_field(1.0);

    std::vector<Point3> sequence = {
        Point3(1, 0, 0),
        Point3(0, 1, 0),
        Point3(0, 0, 1),
        Point3(-1, 0, 0),
        Point3(0, -1, 0),
        Point3(0, 0, -1)
    };

    SequencePointGenerator generator(sequence);
    UniformIntervalSampler sampler(42);

    DensitySampledIntegrableField<ConstantScalarField, SequencePointGenerator, UniformIntervalSampler> integrable_field(
        scalar_field,
        std::move(generator),
        6,
        1.0,
        std::move(sampler)
    );

    double result = integrable_field.integrate();

    double expected_sphere_surface_area = 4.0 * M_PI;
    EXPECT_NEAR(result, expected_sphere_surface_area, 0.1);
}

TEST(DensitySampledIntegrableFieldTest, EXPENSIVE_IntegratesEntireSphereWithUniformDensity) {
    REQUIRE_EXPENSIVE();
    ConstantScalarField scalar_field(1.0);
    size_t sample_count = 10'000;

    DensitySampledIntegrableField<ConstantScalarField, RandomSpherePointGenerator<>> integrable_field(
        scalar_field,
        RandomSpherePointGenerator<>(),
        sample_count,
        1.0
    );

    double expected = 4 * M_PI;
    double tolerance = expected * 0.05;
    EXPECT_NEAR(integrable_field.integrate(), expected, tolerance);
}

TEST(DensitySampledIntegrableFieldTest, EXPENSIVE_IntegratesPolygonSubsetOfSphere) {
    REQUIRE_EXPENSIVE();

    ConstantScalarField scalar_field(1.0);
    size_t sample_count = 50'000;

    DensitySampledIntegrableField<ConstantScalarField, RandomSpherePointGenerator<>> integrable_field(
        scalar_field,
        RandomSpherePointGenerator<>(),
        sample_count,
        1.0
    );

    SphericalPolygon polygon = make_northern_hemisphere_polygon();

    double result = integrable_field.integrate(polygon);
    double total = integrable_field.integrate();

    EXPECT_GT(result, 0.0);
    EXPECT_LT(result, total);
    EXPECT_GT(result / total, 0.1);
    EXPECT_LT(result / total, 0.9);
}

