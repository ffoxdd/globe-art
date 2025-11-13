#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./centroid_calculator.hpp"
#include "../noise_generator/mock_noise_generator.hpp"
#include "../geometry/helpers.hpp"
#include <cmath>

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

} // namespace

TEST(CentroidCalculatorTest, ComputesHemisphereCentroidWithUniformDensity) {
    MockNoiseGenerator mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_noise_generator, value(_)).WillRepeatedly(Return(1.0));

    auto sample_generator = [inside_point]() -> Point3 {
        return inside_point;
    };

    CentroidCalculator<MockNoiseGenerator>::Config config{
        .spherical_polygon = spherical_polygon,
        .noise_generator = mock_noise_generator,
        .error_threshold = 1e-12,
        .consecutive_stable_iterations_threshold = 3,
        .sample_point_generator = sample_generator,
        .bounding_box_override = hemisphere_bounding_box()
    };

    CentroidCalculator<MockNoiseGenerator> centroid_calculator(std::move(config));

    Point3 centroid = centroid_calculator.centroid();

    EXPECT_NEAR(centroid.x(), 0.0, 1e-6);
    EXPECT_NEAR(centroid.y(), 0.0, 1e-6);
    EXPECT_NEAR(centroid.z(), 1.0, 1e-6);
}

TEST(CentroidCalculatorTest, HandlesSamplesOutsideBeforeInside) {
    MockNoiseGenerator mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);
    const Point3 outside_point(0, 0, -1);

    EXPECT_CALL(mock_noise_generator, value(_)).WillRepeatedly(Return(1.0));

    auto sample_generator = [inside_point, outside_point, emitted_outside = false]() mutable -> Point3 {
        if (!emitted_outside) {
            emitted_outside = true;
            return outside_point;
        }
        return inside_point;
    };

    CentroidCalculator<MockNoiseGenerator>::Config config{
        .spherical_polygon = spherical_polygon,
        .noise_generator = mock_noise_generator,
        .error_threshold = 1e-12,
        .consecutive_stable_iterations_threshold = 3,
        .sample_point_generator = sample_generator,
        .bounding_box_override = hemisphere_bounding_box()
    };

    CentroidCalculator<MockNoiseGenerator> centroid_calculator(std::move(config));

    Point3 centroid = centroid_calculator.centroid();

    EXPECT_NEAR(centroid.x(), 0.0, 1e-6);
    EXPECT_NEAR(centroid.y(), 0.0, 1e-6);
    EXPECT_NEAR(centroid.z(), 1.0, 1e-6);
}

TEST(CentroidCalculatorTest, AppliesNoiseWeighting) {
    MockNoiseGenerator mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();

    const Point3 north_pole(0, 0, 1);
    const Point3 equator_point(std::sqrt(2) / 2.0, 0, std::sqrt(2) / 2.0);

    EXPECT_CALL(mock_noise_generator, value(_))
        .WillRepeatedly(testing::Invoke([&](const Point3 &point) {
            return std::abs(point.z() - north_pole.z()) < 1e-9 ? 1.0 : 3.0;
        }));

    auto sample_generator = [north_pole, equator_point, toggle = false]() mutable -> Point3 {
        toggle = !toggle;
        return toggle ? north_pole : equator_point;
    };

    CentroidCalculator<MockNoiseGenerator>::Config config{
        .spherical_polygon = spherical_polygon,
        .noise_generator = mock_noise_generator,
        .error_threshold = 1e-12,
        .consecutive_stable_iterations_threshold = 3,
        .sample_point_generator = sample_generator,
        .bounding_box_override = hemisphere_bounding_box()
    };

    CentroidCalculator<MockNoiseGenerator> centroid_calculator(std::move(config));
    Point3 centroid = centroid_calculator.centroid();

    Vector3 expected_vector = position_vector(north_pole) * 1.0 + position_vector(equator_point) * 3.0;
    Point3 expected_point(expected_vector.x(), expected_vector.y(), expected_vector.z());
    expected_point = project_to_sphere(expected_point);

    EXPECT_NEAR(centroid.x(), expected_point.x(), 1e-6);
    EXPECT_NEAR(centroid.y(), expected_point.y(), 1e-6);
    EXPECT_NEAR(centroid.z(), expected_point.z(), 1e-6);
}
