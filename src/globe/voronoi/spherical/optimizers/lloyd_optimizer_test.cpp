#include "lloyd_optimizer.hpp"
#include "../core/random_builder.hpp"
#include <gtest/gtest.h>

using namespace globe;
using namespace globe::voronoi::spherical;

TEST(LloydOptimizerTest, ReducesDeviation) {
    auto sphere = RandomBuilder().build(10);

    double initial_deviation = 0.0;
    size_t index = 0;
    for (const auto &cell : sphere->cells()) {
        VectorS2 site = to_vector_s2(sphere->site(index));
        VectorS2 centroid = cell.centroid();
        double deviation = distance(site, centroid);
        initial_deviation += deviation * deviation;
        index++;
    }
    initial_deviation = std::sqrt(initial_deviation / sphere->size());

    LloydOptimizer optimizer(std::move(sphere), 5);
    auto optimized = optimizer.optimize();

    double final_deviation = 0.0;
    index = 0;
    for (const auto &cell : optimized->cells()) {
        VectorS2 site = to_vector_s2(optimized->site(index));
        VectorS2 centroid = cell.centroid();
        double deviation = distance(site, centroid);
        final_deviation += deviation * deviation;
        index++;
    }
    final_deviation = std::sqrt(final_deviation / optimized->size());

    EXPECT_LT(final_deviation, initial_deviation);
}

TEST(LloydOptimizerTest, PreservesPointCount) {
    auto sphere = RandomBuilder().build(15);
    size_t original_size = sphere->size();

    LloydOptimizer optimizer(std::move(sphere), 3);
    auto optimized = optimizer.optimize();

    EXPECT_EQ(optimized->size(), original_size);
}
