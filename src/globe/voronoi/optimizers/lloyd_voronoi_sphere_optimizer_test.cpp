#include "lloyd_voronoi_sphere_optimizer.hpp"
#include "../core/random_voronoi_sphere_builder.hpp"
#include <gtest/gtest.h>

using namespace globe;

TEST(LloydVoronoiSphereOptimizerTest, ReducesDeviation) {
    auto voronoi = RandomVoronoiSphereBuilder().build(10);

    double initial_deviation = 0.0;
    size_t index = 0;
    for (const auto &cell : voronoi->dual_cells()) {
        Point3 site = voronoi->site(index);
        Point3 centroid = cell.centroid();
        double deviation = angular_distance(
            to_position_vector(site),
            to_position_vector(centroid)
        );
        initial_deviation += deviation * deviation;
        index++;
    }
    initial_deviation = std::sqrt(initial_deviation / voronoi->size());

    LloydVoronoiSphereOptimizer optimizer(std::move(voronoi), 5);
    auto optimized = optimizer.optimize();

    double final_deviation = 0.0;
    index = 0;
    for (const auto &cell : optimized->dual_cells()) {
        Point3 site = optimized->site(index);
        Point3 centroid = cell.centroid();
        double deviation = angular_distance(
            to_position_vector(site),
            to_position_vector(centroid)
        );
        final_deviation += deviation * deviation;
        index++;
    }
    final_deviation = std::sqrt(final_deviation / optimized->size());

    EXPECT_LT(final_deviation, initial_deviation);
}

TEST(LloydVoronoiSphereOptimizerTest, PreservesPointCount) {
    auto voronoi = RandomVoronoiSphereBuilder().build(15);
    size_t original_size = voronoi->size();

    LloydVoronoiSphereOptimizer optimizer(std::move(voronoi), 3);
    auto optimized = optimizer.optimize();

    EXPECT_EQ(optimized->size(), original_size);
}
