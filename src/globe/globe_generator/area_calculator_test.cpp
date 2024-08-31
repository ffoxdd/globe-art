#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "./area_calculator.hpp"
#include "../noise_generator/mock_noise_generator.hpp"

using namespace globe;
using ::testing::Return;

TEST(AreaCalculatorTest, CalculatesAreaCorrectly) {
    MockNoiseGenerator mock_noise_generator;
    SphericalCircle3 circle(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0));

    SphericalPolygon spherical_polygon = SphericalPolygon(
        std::vector<Arc>{
            Arc(circle, SphericalPoint3(1, 0, 0), SphericalPoint3(0, 1, 0)),
            Arc(circle, SphericalPoint3(0, 1, 0), SphericalPoint3(0, 0, 1)),
            Arc(circle, SphericalPoint3(0, 0, 1), SphericalPoint3(1, 0, 0))
        }
    );

    AreaCalculator<MockNoiseGenerator> area_calculator(
        AreaCalculator<MockNoiseGenerator>::Config{
            spherical_polygon,
            mock_noise_generator,
            1e-6,
            10
        }
    );

    EXPECT_CALL(mock_noise_generator, value(testing::_)).WillRepeatedly(Return(1.0));

    EXPECT_NEAR(area_calculator.area(), M_PI / 2, 1e-6);
}

TEST(AreaCalculatorTest, CalculatesAreaWithAlternatingDensity) {
    MockNoiseGenerator mock_noise_generator;
    SphericalCircle3 circle(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0));

    SphericalPolygon spherical_polygon = SphericalPolygon(
        std::vector<Arc>{
            Arc(circle, SphericalPoint3(1, 0, 0), SphericalPoint3(0, 1, 0)),
            Arc(circle, SphericalPoint3(0, 1, 0), SphericalPoint3(0, 0, 1)),
            Arc(circle, SphericalPoint3(0, 0, 1), SphericalPoint3(1, 0, 0))
        }
    );

    AreaCalculator<MockNoiseGenerator> area_calculator(
        AreaCalculator<MockNoiseGenerator>::Config{
            spherical_polygon,
            mock_noise_generator,
            1e-6,
            10
        }
    );

    static double values[] = {1.0, 2.0};
    static int call_count = 0;

    EXPECT_CALL(mock_noise_generator, value(testing::_)).WillRepeatedly(
        testing::Invoke([](const Point3 &) -> double { return values[call_count++ % 2]; })
    );

    EXPECT_NEAR(area_calculator.area(), (M_PI / 2) * 1.5, 1e-6);
}
