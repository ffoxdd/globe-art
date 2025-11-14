#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./centroid_calculator.hpp"
#include "../noise_generator/mock_scalar_field.hpp"
#include "../geometry/helpers.hpp"
#include <cmath>
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

class AlternatingSamplePointGenerator {
 public:
    AlternatingSamplePointGenerator(Point3 point1, Point3 point2)
        : _point1(point1), _point2(point2), _toggle(false) {}

    Point3 generate() {
        _toggle = !_toggle;
        return _toggle ? _point1 : _point2;
    }
 private:
    Point3 _point1;
    Point3 _point2;
    bool _toggle;
};

} // namespace

TEST(CentroidCalculatorTest, ComputesHemisphereCentroidWithUniformDensity) {
    MockScalarField mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_noise_generator, value(_)).WillRepeatedly(Return(1.0));

    CentroidCalculator<MockScalarField, ConstantSamplePointGenerator> centroid_calculator(
        spherical_polygon,
        mock_noise_generator,
        ConstantSamplePointGenerator(inside_point),
        1e-12,
        3,
        hemisphere_bounding_box()
    );

    Point3 centroid = centroid_calculator.centroid();

    EXPECT_NEAR(centroid.x(), 0.0, 1e-6);
    EXPECT_NEAR(centroid.y(), 0.0, 1e-6);
    EXPECT_NEAR(centroid.z(), 1.0, 1e-6);
}

TEST(CentroidCalculatorTest, HandlesSamplesOutsideBeforeInside) {
    MockScalarField mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);
    const Point3 outside_point(0, 0, -1);

    EXPECT_CALL(mock_noise_generator, value(_)).WillRepeatedly(Return(1.0));

    CentroidCalculator<MockScalarField, TogglingSamplePointGenerator> centroid_calculator(
        spherical_polygon,
        mock_noise_generator,
        TogglingSamplePointGenerator(outside_point, inside_point),
        1e-12,
        3,
        hemisphere_bounding_box()
    );

    Point3 centroid = centroid_calculator.centroid();

    EXPECT_NEAR(centroid.x(), 0.0, 1e-6);
    EXPECT_NEAR(centroid.y(), 0.0, 1e-6);
    EXPECT_NEAR(centroid.z(), 1.0, 1e-6);
}

TEST(CentroidCalculatorTest, AppliesNoiseWeighting) {
    MockScalarField mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();

    const Point3 north_pole(0, 0, 1);
    const Point3 equator_point(std::sqrt(2) / 2.0, 0, std::sqrt(2) / 2.0);

    EXPECT_CALL(mock_noise_generator, value(_))
        .WillRepeatedly(testing::Invoke([&](const Point3 &point) {
            return std::abs(point.z() - north_pole.z()) < 1e-9 ? 1.0 : 3.0;
        }));

    CentroidCalculator<MockScalarField, AlternatingSamplePointGenerator> centroid_calculator(
        spherical_polygon,
        mock_noise_generator,
        AlternatingSamplePointGenerator(north_pole, equator_point),
        1e-12,
        3,
        hemisphere_bounding_box()
    );

    Point3 centroid = centroid_calculator.centroid();

    Vector3 expected_vector = position_vector(north_pole) * 1.0 + position_vector(equator_point) * 3.0;
    Point3 expected_point(expected_vector.x(), expected_vector.y(), expected_vector.z());
    expected_point = project_to_sphere(expected_point);

    EXPECT_NEAR(centroid.x(), expected_point.x(), 1e-6);
    EXPECT_NEAR(centroid.y(), expected_point.y(), 1e-6);
    EXPECT_NEAR(centroid.z(), expected_point.z(), 1e-6);
}
