#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "noise_generator.hpp"
#include "anl_noise_generator.hpp"
#include "../point_generator/random_sphere_point_generator.hpp"
#include "../point_generator/mock_point_generator.hpp"

using namespace globe;

TEST(NoiseGeneratorTest, ValueMethodReturnsDouble) {
    Point3 location = {0.1, 0.2, 0.3};

    auto point_generator = std::make_unique<MockPointGenerator>();

    ON_CALL(*point_generator, generate).WillByDefault([]() {
            return Point3(1, 1, 1);
        }
    );

    AnlNoiseGenerator<MockPointGenerator> noise_generator(
        AnlNoiseGenerator<MockPointGenerator>::Config{
            .low =  -1,
            .high =  1,
            .point_generator =  std::move(point_generator)
        }
    );

    double value = noise_generator.value(location);
    EXPECT_TRUE(typeid(value) == typeid(double));
}

TEST(NoiseGeneratorTest, ValueMethodReturnsConsistentResult) {
    Point3 location = {0.1, 0.2, 0.3};

    std::vector<Point3> points;
    points.emplace_back(location);
    points.emplace_back(0.4, 0.5, 0.6);

    auto point_generator = std::make_unique<MockPointGenerator>();

    ON_CALL(*point_generator, generate).WillByDefault([]() {
            return Point3();
        }
    );

    AnlNoiseGenerator<MockPointGenerator> noise_generator(
        AnlNoiseGenerator<MockPointGenerator>::Config{
            .low =  -1,
            .high =  1,
            .point_generator =  std::move(point_generator)
        }
    );

    double value1 = noise_generator.value(location);
    double value2 = noise_generator.value(location);

    EXPECT_DOUBLE_EQ(value1, value2);
}

TEST(NoiseGeneratorTest, ValueMethodDifferentLocations) {
    Point3 location1 = {0.1, 0.2, 0.3};
    Point3 location2 = {0.4, 0.5, 0.6};

    std::vector<Point3> points;
    points.emplace_back(location1);
    points.emplace_back(location2);

    auto point_generator = std::make_unique<MockPointGenerator>();

    ON_CALL(*point_generator, generate).WillByDefault([]() {
            return Point3();
        }
    );

    AnlNoiseGenerator<MockPointGenerator> noise_generator(
        AnlNoiseGenerator<MockPointGenerator>::Config{
            .low =  -1,
            .high =  1,
            .point_generator =  std::move(point_generator)
        }
    );

    double value1 = noise_generator.value(location1);
    double value2 = noise_generator.value(location2);

    EXPECT_NE(value1, value2);
}

TEST(NoiseGeneratorTest, CanNormalizeOutputOverSamplePoints) {
    std::vector<Point3> points;
    points.emplace_back(0.1, 0.2, 0.3);
    points.emplace_back(0.4, 0.5, 0.6);
    points.emplace_back(0.7, 0.8, 0.9);

    double low = -0.01;
    double high = 0.02;

//    auto point_generator = std::make_unique<MockPointGenerator>();
    auto point_generator = std::make_unique<RandomSpherePointGenerator>(1);

//    ON_CALL(*point_generator, generate).WillByDefault([]() {
//        return Point3();
//    });

    AnlNoiseGenerator<RandomSpherePointGenerator> noise_generator(
        AnlNoiseGenerator<RandomSpherePointGenerator>::Config{
            .low =  -1,
            .high =  1,
            .point_generator =  std::move(point_generator)
        }
    );

    for (auto &point : points) {
        double value = noise_generator.value(point);

        EXPECT_GE(value, low);
        EXPECT_LE(value, high);
    }
}
