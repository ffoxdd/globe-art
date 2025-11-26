#include <gtest/gtest.h>
#include "rejection_sampling_sphere_point_generator.hpp"
#include "../../fields/scalar/constant_scalar_field.hpp"
#include "../../testing/geometric_assertions.hpp"

using namespace globe;
using globe::testing::is_on_unit_sphere;

TEST(RejectionSamplingSpherePointGeneratorTest, GenerateReturnsExactCount) {
    RejectionSamplingSpherePointGenerator generator;
    constexpr size_t count = 100;

    auto points = generator.generate(count);

    EXPECT_EQ(points.size(), count);
}

TEST(RejectionSamplingSpherePointGeneratorTest, GenerateZeroReturnsEmpty) {
    RejectionSamplingSpherePointGenerator generator;

    auto points = generator.generate(0);

    EXPECT_TRUE(points.empty());
    EXPECT_EQ(generator.last_attempt_count(), 0);
}

TEST(RejectionSamplingSpherePointGeneratorTest, AllPointsOnUnitSphere) {
    RejectionSamplingSpherePointGenerator generator;
    constexpr size_t count = 50;

    auto points = generator.generate(count);

    for (const auto &point : points) {
        EXPECT_TRUE(is_on_unit_sphere(point));
    }
}

TEST(RejectionSamplingSpherePointGeneratorTest, UniformDensityAcceptsAll) {
    ConstantScalarField uniform_field(1.0);
    RejectionSamplingSpherePointGenerator generator(uniform_field, 1.0);
    constexpr size_t count = 100;

    auto points = generator.generate(count);

    EXPECT_EQ(points.size(), count);
    EXPECT_EQ(generator.last_attempt_count(), count);
}

TEST(RejectionSamplingSpherePointGeneratorTest, HalfDensityRejectsApproximatelyHalf) {
    ConstantScalarField half_density_field(0.5);
    RejectionSamplingSpherePointGenerator generator(half_density_field, 1.0);
    constexpr size_t count = 100;

    auto points = generator.generate(count);

    EXPECT_EQ(points.size(), count);
    EXPECT_GT(generator.last_attempt_count(), count);
    EXPECT_LT(generator.last_attempt_count(), count * 4);
}

TEST(RejectionSamplingSpherePointGeneratorTest, ZeroDensityNeverAccepts) {
    ConstantScalarField zero_field(0.0);
    RejectionSamplingSpherePointGenerator generator(zero_field, 1.0);

    auto points = generator.generate(0);

    EXPECT_TRUE(points.empty());
}

TEST(RejectionSamplingSpherePointGeneratorTest, LastAttemptCountTracksAttempts) {
    ConstantScalarField field(0.25);
    RejectionSamplingSpherePointGenerator generator(field, 1.0);
    constexpr size_t count = 50;

    generator.generate(count);

    EXPECT_GE(generator.last_attempt_count(), count);
}
