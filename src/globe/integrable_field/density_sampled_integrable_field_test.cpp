#include "gtest/gtest.h"
#include "./density_sampled_integrable_field.hpp"
#include "../scalar_field/constant_scalar_field.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../globe_generator/spherical_bounding_box.hpp"
#include "../types.hpp"
#include <vector>
#include <cmath>

using namespace globe;

namespace {

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

class SequencePointGenerator {
 public:
    explicit SequencePointGenerator(std::vector<Point3> points) :
        _points(std::move(points)),
        _index(0) {
    }

    Point3 generate(const SphericalBoundingBox &) {
        Point3 point = _points[_index % _points.size()];
        _index++;
        return point;
    }

 private:
    std::vector<Point3> _points;
    size_t _index;
};

} // namespace

TEST(DensitySampledIntegrableFieldTest, IntegratesEntireSphereWithUniformDensity) {
    ConstantScalarField scalar_field(1.0);
    std::vector<Point3> points{
        Point3(1, 0, 0),
        Point3(0, 1, 0),
        Point3(0, 0, 1),
        Point3(-1, 0, 0)
    };

    DensitySampledIntegrableField<ConstantScalarField, SequencePointGenerator> integrable_field(
        scalar_field,
        points.size(),
        SequencePointGenerator(points),
        1.0,
        0
    );

    EXPECT_EQ(integrable_field.sample_count(), points.size());
    EXPECT_NEAR(integrable_field.integrate_entire_sphere(), 4 * M_PI, 1e-9);
}

TEST(DensitySampledIntegrableFieldTest, EstimatesHemisphereMassWithBalancedSamples) {
    ConstantScalarField scalar_field(1.0);
    const double balanced_z = std::sqrt(0.5);
    const double oblique_xy = std::sqrt(0.48);

    std::vector<Point3> points{
        Point3(0, 0, 1),
        Point3(0.5, 0.5, balanced_z),
        Point3(-0.6, 0.4, oblique_xy),
        Point3(0, 0, -1),
        Point3(0.5, 0.5, -balanced_z),
        Point3(-0.6, 0.4, -oblique_xy)
    };

    DensitySampledIntegrableField<ConstantScalarField, SequencePointGenerator> integrable_field(
        scalar_field,
        points.size(),
        SequencePointGenerator(points),
        1.0,
        0
    );

    SphericalPolygon polygon = make_northern_hemisphere_polygon();

    EXPECT_NEAR(integrable_field.integrate(polygon), 2 * M_PI, 1e-9);
}

