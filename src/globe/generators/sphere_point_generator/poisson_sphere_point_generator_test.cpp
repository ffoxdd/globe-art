#include <gtest/gtest.h>
#include "poisson_sphere_point_generator.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../testing/geometric_assertions.hpp"
#include "../../math/interval.hpp"
#include "../../math/circular_interval.hpp"
#include <CGAL/squared_distance_3.h>

using namespace globe;
using globe::testing::is_on_unit_sphere;

TEST(PoissonSpherePointGeneratorTest, GenerateReturnsExactCount) {
    PoissonSpherePointGenerator generator;
    constexpr size_t count = 100;

    auto points = generator.generate(count);

    EXPECT_EQ(points.size(), count);
}

TEST(PoissonSpherePointGeneratorTest, GenerateWithBoundingBoxReturnsExactCount) {
    PoissonSpherePointGenerator generator;
    SphericalBoundingBox box(ThetaInterval(0.0, M_PI), Interval(0.0, 1.0));
    constexpr size_t count = 50;

    auto points = generator.generate(count, box);

    EXPECT_EQ(points.size(), count);
}

TEST(PoissonSpherePointGeneratorTest, AllPointsOnUnitSphere) {
    PoissonSpherePointGenerator generator;
    constexpr size_t count = 100;

    auto points = generator.generate(count);

    for (const auto& point : points) {
        EXPECT_TRUE(is_on_unit_sphere(point))
            << "Point (" << point.x() << ", " << point.y() << ", " << point.z()
            << ") is not on unit sphere";
    }
}

TEST(PoissonSpherePointGeneratorTest, GenerateWithBoundingBoxPointsInBox) {
    PoissonSpherePointGenerator generator;
    SphericalBoundingBox box(ThetaInterval(0.0, M_PI / 2), Interval(0.0, 1.0));
    constexpr size_t count = 50;

    auto points = generator.generate(count, box);

    for (const auto& point : points) {
        EXPECT_TRUE(is_on_unit_sphere(point));
        EXPECT_TRUE(box.contains(point))
            << "Point (" << point.x() << ", " << point.y() << ", " << point.z()
            << ") is not in bounding box";
    }
}

TEST(PoissonSpherePointGeneratorTest, EXPENSIVE_PointsAreWellSeparated) {
    REQUIRE_EXPENSIVE();

    PoissonSpherePointGenerator generator;
    constexpr size_t count = 200;

    auto points = generator.generate(count);

    double sphere_area = 4.0 * M_PI;
    double expected_min_distance = std::sqrt(sphere_area / (count * M_PI)) * 0.95;
    double tolerance_factor = 0.7;
    double min_acceptable_distance = expected_min_distance * tolerance_factor;

    double actual_min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            double dist = std::sqrt(CGAL::squared_distance(points[i], points[j]));
            actual_min_distance = std::min(actual_min_distance, dist);
        }
    }

    EXPECT_GE(actual_min_distance, min_acceptable_distance)
        << "Expected minimum distance >= " << min_acceptable_distance
        << " but got " << actual_min_distance;
}

TEST(PoissonSpherePointGeneratorTest, GenerateZeroPoints) {
    PoissonSpherePointGenerator generator;

    auto points = generator.generate(0);

    EXPECT_EQ(points.size(), 0);
}

TEST(PoissonSpherePointGeneratorTest, EXPENSIVE_BetterDistributionThanUniform) {
    REQUIRE_EXPENSIVE();

    constexpr size_t count = 100;
    PoissonSpherePointGenerator poisson_gen;
    RandomSpherePointGenerator uniform_gen;

    auto poisson_points = poisson_gen.generate(count);
    auto uniform_points = uniform_gen.generate(count);

    auto compute_min_distance = [](const std::vector<Point3>& points) {
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                double dist = std::sqrt(CGAL::squared_distance(points[i], points[j]));
                min_dist = std::min(min_dist, dist);
            }
        }
        return min_dist;
    };

    double poisson_min_dist = compute_min_distance(poisson_points);
    double uniform_min_dist = compute_min_distance(uniform_points);

    EXPECT_GT(poisson_min_dist, uniform_min_dist)
        << "Poisson distribution should have larger minimum distance than uniform random"
        << " (Poisson: " << poisson_min_dist << ", Uniform: " << uniform_min_dist << ")";
}
