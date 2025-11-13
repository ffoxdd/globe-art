#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./area_calculator.hpp"
#include "../noise_generator/mock_noise_generator.hpp"

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

TEST(AreaCalculatorTest, EstimatesHemisphereAreaWithUniformDensity) {
    MockNoiseGenerator mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_noise_generator, value(_)).WillRepeatedly(Return(1.0));

    auto sample_generator = [inside_point]() -> Point3 {
        return inside_point;
    };

    AreaCalculator<MockNoiseGenerator>::Config config{
        .spherical_polygon = spherical_polygon,
        .noise_generator = mock_noise_generator,
        .error_threshold = 1e-12,
        .consecutive_stable_iterations_threshold = 3,
        .sample_point_generator = sample_generator,
        .bounding_box_override = hemisphere_bounding_box()
    };

    AreaCalculator<MockNoiseGenerator> area_calculator(std::move(config));

    EXPECT_NEAR(area_calculator.area(), 2 * M_PI, 1e-6);
}

TEST(AreaCalculatorTest, HandlesSamplesOutsideBeforeInside) {
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

    AreaCalculator<MockNoiseGenerator>::Config config{
        .spherical_polygon = spherical_polygon,
        .noise_generator = mock_noise_generator,
        .error_threshold = 1e-6,
        .consecutive_stable_iterations_threshold = 5,
        .sample_point_generator = sample_generator,
        .bounding_box_override = hemisphere_bounding_box()
    };

    AreaCalculator<MockNoiseGenerator> area_calculator(std::move(config));

    double area = area_calculator.area();
    EXPECT_NEAR(area / (2 * M_PI), 1.0, 0.1);
}

TEST(AreaCalculatorTest, AppliesNoiseDensityWeighting) {
    MockNoiseGenerator mock_noise_generator;
    SphericalPolygon spherical_polygon = make_northern_hemisphere_polygon();
    const Point3 inside_point(0, 0, 1);

    EXPECT_CALL(mock_noise_generator, value(_))
        .WillOnce(Return(1.0))
        .WillOnce(Return(3.0))
        .WillRepeatedly(Return(2.0));

    auto sample_generator = [inside_point]() -> Point3 {
        return inside_point;
    };

    AreaCalculator<MockNoiseGenerator>::Config config{
        .spherical_polygon = spherical_polygon,
        .noise_generator = mock_noise_generator,
        .error_threshold = 1e-12,
        .consecutive_stable_iterations_threshold = 3,
        .sample_point_generator = sample_generator,
        .bounding_box_override = hemisphere_bounding_box()
    };

    AreaCalculator<MockNoiseGenerator> area_calculator(std::move(config));

    EXPECT_NEAR(area_calculator.area(), 4 * M_PI, 1e-6);
}
