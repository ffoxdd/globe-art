#include "gtest/gtest.h"
#include "./density_sampled_integrable_field.hpp"
#include "../scalar/constant_scalar_field.hpp"
#include "../../spherical/spherical_polygon.hpp"
#include "../../spherical/spherical_bounding_box.hpp"
#include "../../types.hpp"
#include <vector>
#include <cmath>

using namespace globe;

SphericalPolygon make_northern_hemisphere_polygon() {
    return SphericalPolygon(
        std::vector<Arc>{
            Arc(
                SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
                SphericalPoint3(1, 0, 0),
                SphericalPoint3(0, 1, 0)
            ),
            Arc(
                SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
                SphericalPoint3(0, 1, 0),
                SphericalPoint3(-1, 0, 0)
            ),
            Arc(
                SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
                SphericalPoint3(-1, 0, 0),
                SphericalPoint3(0, -1, 0)
            ),
            Arc(
                SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
                SphericalPoint3(0, -1, 0),
                SphericalPoint3(1, 0, 0)
            ),
        }
    );
}

TEST(DensitySampledIntegrableFieldTest, IntegratesEntireSphereWithUniformDensity) {
    ConstantScalarField scalar_field(1.0);
    size_t sample_count = 10'000;

    DensitySampledIntegrableField<ConstantScalarField> integrable_field(
        scalar_field,
        sample_count,
        1.0
    );

    double expected = 4 * M_PI;
    double tolerance = expected * 0.05;
    EXPECT_NEAR(integrable_field.integrate(), expected, tolerance);
}

TEST(DensitySampledIntegrableFieldTest, IntegratesPolygonSubsetOfSphere) {
    ConstantScalarField scalar_field(1.0);
    size_t sample_count = 50'000;

    DensitySampledIntegrableField<ConstantScalarField> integrable_field(
        scalar_field,
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

