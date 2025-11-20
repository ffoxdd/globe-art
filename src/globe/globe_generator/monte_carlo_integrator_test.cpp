#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./monte_carlo_integrator.hpp"
#include "../scalar_field/mock_scalar_field.hpp"
#include <utility>
#include <vector>

using namespace globe;
using ::testing::Return;
using ::testing::_;

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

SphericalBoundingBox hemisphere_bounding_box() {
    return {Interval(0, 2 * M_PI), Interval(0, 1)};
}

class ConstantSamplePointGenerator {
 public:
    explicit ConstantSamplePointGenerator(Point3 point) : _point(point) {}
    Point3 generate(const SphericalBoundingBox &) { return _point; }
 private:
    Point3 _point;
};

class TogglingSamplePointGenerator {
 public:
    TogglingSamplePointGenerator(Point3 first_point, Point3 subsequent_point)
        : _first_point(first_point), _subsequent_point(subsequent_point), _first_call(true) {}

    Point3 generate(const SphericalBoundingBox &) {
        if (_first_call) {
            _first_call = false;
            return _first_point;
        }
        return _subsequent_point;
    }
 private:
    Point3 _first_point;
    Point3 _subsequent_point;
    bool _first_call;
};

class SequenceSamplePointGenerator {
 public:
    SequenceSamplePointGenerator(std::vector<Point3> sequence)
        : _sequence(std::move(sequence)), _index(0) {}

    Point3 generate(const SphericalBoundingBox &) {
        Point3 result = _sequence[_index % _sequence.size()];
        _index++;
        return result;
    }
 private:
    std::vector<Point3> _sequence;
    size_t _index;
};

} // namespace

TEST(MonteCarloIntegratorTest, EstimatesHemisphereMassWithUniformDensity) {
    MockScalarField mock_scalar_field;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_scalar_field, value(_)).WillRepeatedly(Return(1.0));

    MonteCarloIntegrator<MockScalarField, ConstantSamplePointGenerator> monte_carlo_integrator(
        std::ref(spherical_polygon),
        mock_scalar_field,
        ConstantSamplePointGenerator(inside_point),
        hemisphere_bounding_box()
    );

    EXPECT_NEAR(
        monte_carlo_integrator.integrate(),
        2 * M_PI,
        1e-6
    );
}

TEST(MonteCarloIntegratorTest, HandlesSamplesOutsideBeforeInside) {
    MockScalarField mock_scalar_field;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);
    const Point3 outside_point(0, 0, -1);

    EXPECT_CALL(mock_scalar_field, value(_)).WillRepeatedly(Return(1.0));


    MonteCarloIntegrator<MockScalarField, TogglingSamplePointGenerator> monte_carlo_integrator(
        std::ref(spherical_polygon),
        mock_scalar_field,
        TogglingSamplePointGenerator(outside_point, inside_point),
        hemisphere_bounding_box()
    );

    double cell_mass = monte_carlo_integrator.integrate();
    EXPECT_NEAR(cell_mass / (2 * M_PI), 1.0, 0.1);
}

TEST(MonteCarloIntegratorTest, AppliesNoiseDensityWeighting) {
    MockScalarField mock_scalar_field;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_scalar_field, value(_))
        .WillOnce(Return(1.0))
        .WillOnce(Return(3.0))
        .WillRepeatedly(Return(2.0));

    MonteCarloIntegrator<MockScalarField, ConstantSamplePointGenerator> monte_carlo_integrator(
        std::ref(spherical_polygon),
        mock_scalar_field,
        ConstantSamplePointGenerator(inside_point),
        hemisphere_bounding_box()
    );

    EXPECT_NEAR(
        monte_carlo_integrator.integrate(),
        4 * M_PI,
        1e-6
    );
}

TEST(MonteCarloIntegratorTest, CalculatesTotalSphereMassWithUniformDensity) {
    MockScalarField mock_scalar_field;
    const Point3 arbitrary_point(0, 0, 1);

    EXPECT_CALL(mock_scalar_field, value(_)).WillRepeatedly(Return(1.0));


    MonteCarloIntegrator<MockScalarField, ConstantSamplePointGenerator> monte_carlo_integrator(
        std::nullopt,
        mock_scalar_field,
        ConstantSamplePointGenerator(arbitrary_point)
    );

    EXPECT_NEAR(
        monte_carlo_integrator.integrate(),
        4 * M_PI,
        1e-6
    );
}

TEST(MonteCarloIntegratorTest, CalculatesTotalSphereMassWithNonUniformDensity) {
    MockScalarField mock_scalar_field;
    const Point3 arbitrary_point(0, 0, 1);

    EXPECT_CALL(mock_scalar_field, value(_))
        .WillOnce(Return(2.0))
        .WillOnce(Return(4.0))
        .WillRepeatedly(Return(3.0));


    MonteCarloIntegrator<MockScalarField, ConstantSamplePointGenerator> monte_carlo_integrator(
        std::nullopt,
        mock_scalar_field,
        ConstantSamplePointGenerator(arbitrary_point)
    );

    EXPECT_NEAR(
        monte_carlo_integrator.integrate(),
        12 * M_PI,
        1e-6
    );
}
