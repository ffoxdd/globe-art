#include "gtest/gtest.h"
#include "./sampled_integrable_field.hpp"
#include "../scalar/constant_scalar_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../generators/sphere_point_generator/random_sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/rejection_sampling_sphere_point_generator.hpp"
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
            make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
            make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
            make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
            make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
        }
    );
}

TEST(SampledIntegrableFieldTest, AcceptsAllPointsWithConstantDensity) {
    std::vector<Point3> sequence = {
        Point3(1, 0, 0),
        Point3(0, 1, 0),
        Point3(0, 0, 1),
        Point3(-1, 0, 0),
        Point3(0, -1, 0),
        Point3(0, 0, -1)
    };

    SequencePointGenerator generator(sequence);

    SampledIntegrableField<SequencePointGenerator> integrable_field(
        std::move(generator),
        6,
        1.0
    );

    double result = integrable_field.integrate();

    double expected_sphere_surface_area = 4.0 * M_PI;
    EXPECT_NEAR(result, expected_sphere_surface_area, 0.1);
}

TEST(SampledIntegrableFieldTest, EXPENSIVE_IntegratesEntireSphereWithUniformDensity) {
    REQUIRE_EXPENSIVE();
    ConstantScalarField density_field(1.0);
    size_t sample_count = 10'000;

    RejectionSamplingSpherePointGenerator<ConstantScalarField> generator(density_field, 1.0);

    SampledIntegrableField<RejectionSamplingSpherePointGenerator<ConstantScalarField>> integrable_field(
        std::move(generator),
        sample_count,
        1.0
    );

    double expected = 4 * M_PI;
    double tolerance = expected * 0.05;
    EXPECT_NEAR(integrable_field.integrate(), expected, tolerance);
}

TEST(SampledIntegrableFieldTest, EXPENSIVE_IntegratesPolygonSubsetOfSphere) {
    REQUIRE_EXPENSIVE();

    ConstantScalarField density_field(1.0);
    size_t sample_count = 50'000;

    RejectionSamplingSpherePointGenerator<ConstantScalarField> generator(density_field, 1.0);

    SampledIntegrableField<RejectionSamplingSpherePointGenerator<ConstantScalarField>> integrable_field(
        std::move(generator),
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

TEST(SampledIntegrableFieldTest, MaxFrequencyIsZeroForEmptyField) {
    std::vector<Point3> empty_sequence;
    SequencePointGenerator generator(empty_sequence);

    SampledIntegrableField<SequencePointGenerator> field(
        std::move(generator),
        0,
        1.0
    );

    EXPECT_DOUBLE_EQ(field.max_frequency(), 0.0);
}

TEST(SampledIntegrableFieldTest, MaxFrequencyComputedFromSampleCount) {
    std::vector<Point3> sequence = {
        Point3(1, 0, 0),
        Point3(0, 1, 0),
        Point3(0, 0, 1),
        Point3(-1, 0, 0),
        Point3(0, -1, 0),
        Point3(0, 0, -1)
    };

    SequencePointGenerator generator(sequence);

    SampledIntegrableField<SequencePointGenerator> field(
        std::move(generator),
        6,
        1.0
    );

    double average_spacing = std::sqrt(UNIT_SPHERE_AREA / 6.0);
    double expected_frequency = 1.0 / (2.0 * average_spacing);

    EXPECT_DOUBLE_EQ(field.max_frequency(), expected_frequency);
}

