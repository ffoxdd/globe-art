#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./mass_calculator.hpp"
#include "../noise_generator/mock_scalar_field.hpp"
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
    Point3 generate() { return _point; }
 private:
    Point3 _point;
};

class TogglingSamplePointGenerator {
 public:
    TogglingSamplePointGenerator(Point3 first_point, Point3 subsequent_point)
        : _first_point(first_point), _subsequent_point(subsequent_point), _first_call(true) {}

    Point3 generate() {
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

    Point3 generate() {
        Point3 result = _sequence[_index % _sequence.size()];
        _index++;
        return result;
    }
 private:
    std::vector<Point3> _sequence;
    size_t _index;
};

} // namespace

TEST(MassCalculatorTest, EstimatesHemisphereMassWithUniformDensity) {
    MockScalarField mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_noise_generator, value(_)).WillRepeatedly(Return(1.0));

    MassCalculator<MockScalarField, ConstantSamplePointGenerator> mass_calculator(
        std::ref(spherical_polygon),
        mock_noise_generator,
        ConstantSamplePointGenerator(inside_point),
        1e-12,
        3,
        hemisphere_bounding_box()
    );

    EXPECT_NEAR(mass_calculator.mass(), 2 * M_PI, 1e-6);
}

TEST(MassCalculatorTest, HandlesSamplesOutsideBeforeInside) {
    MockScalarField mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);
    const Point3 outside_point(0, 0, -1);

    EXPECT_CALL(mock_noise_generator, value(_)).WillRepeatedly(Return(1.0));

    MassCalculator<MockScalarField, TogglingSamplePointGenerator> mass_calculator(
        std::ref(spherical_polygon),
        mock_noise_generator,
        TogglingSamplePointGenerator(outside_point, inside_point),
        1e-6,
        5,
        hemisphere_bounding_box()
    );

    double cell_mass = mass_calculator.mass();
    EXPECT_NEAR(cell_mass / (2 * M_PI), 1.0, 0.1);
}

TEST(MassCalculatorTest, AppliesNoiseDensityWeighting) {
    MockScalarField mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_noise_generator, value(_))
        .WillOnce(Return(1.0))
        .WillOnce(Return(3.0))
        .WillRepeatedly(Return(2.0));

    MassCalculator<MockScalarField, ConstantSamplePointGenerator> mass_calculator(
        std::ref(spherical_polygon),
        mock_noise_generator,
        ConstantSamplePointGenerator(inside_point),
        1e-12,
        3,
        hemisphere_bounding_box()
    );

    EXPECT_NEAR(mass_calculator.mass(), 4 * M_PI, 1e-6);
}

TEST(MassCalculatorTest, CalculatesTotalSphereMassWithUniformDensity) {
    MockScalarField mock_noise_generator;
    const Point3 arbitrary_point(0, 0, 1);

    EXPECT_CALL(mock_noise_generator, value(_)).WillRepeatedly(Return(1.0));

    MassCalculator<MockScalarField, ConstantSamplePointGenerator> mass_calculator(
        std::nullopt,
        mock_noise_generator,
        ConstantSamplePointGenerator(arbitrary_point),
        1e-12,
        3
    );

    EXPECT_NEAR(mass_calculator.mass(), 4 * M_PI, 1e-6);
}

TEST(MassCalculatorTest, CalculatesTotalSphereMassWithNonUniformDensity) {
    MockScalarField mock_noise_generator;
    const Point3 arbitrary_point(0, 0, 1);

    EXPECT_CALL(mock_noise_generator, value(_))
        .WillOnce(Return(2.0))
        .WillOnce(Return(4.0))
        .WillRepeatedly(Return(3.0));

    MassCalculator<MockScalarField, ConstantSamplePointGenerator> mass_calculator(
        std::nullopt,
        mock_noise_generator,
        ConstantSamplePointGenerator(arbitrary_point),
        1e-12,
        3
    );

    EXPECT_NEAR(mass_calculator.mass(), 12 * M_PI, 1e-6);
}
