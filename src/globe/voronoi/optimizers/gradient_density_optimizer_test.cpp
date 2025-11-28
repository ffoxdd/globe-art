#include "gradient_density_optimizer.hpp"
#include "../../fields/spherical/constant_spherical_field.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <memory>

namespace globe {
namespace {

std::unique_ptr<VoronoiSphere> create_test_voronoi(size_t num_points) {
    auto voronoi = std::make_unique<VoronoiSphere>();

    double phi = M_PI * (std::sqrt(5.0) - 1.0);
    for (size_t i = 0; i < num_points; ++i) {
        double y = 1.0 - (i / static_cast<double>(num_points - 1)) * 2.0;
        double radius = std::sqrt(1.0 - y * y);
        double theta = phi * i;

        double x = std::cos(theta) * radius;
        double z = std::sin(theta) * radius;

        voronoi->insert(Point3(x, y, z));
    }

    return voronoi;
}

TEST(GradientDensityOptimizerTest, ConvergesForConstantField) {
    auto voronoi = create_test_voronoi(10);
    ConstantSphericalField field(1.0);

    GradientDensityOptimizer optimizer(
        std::move(voronoi),
        field,
        50,
        0.5
    );

    auto result = optimizer.optimize();

    EXPECT_EQ(result->size(), 10);
}

TEST(GradientDensityOptimizerTest, CellEdgesHaveValidNeighbors) {
    auto voronoi = create_test_voronoi(10);

    for (size_t i = 0; i < voronoi->size(); ++i) {
        auto edges = voronoi->cell_edges(i);
        EXPECT_GT(edges.size(), 0) << "Cell " << i << " has no edges";

        for (const auto& edge : edges) {
            EXPECT_LT(edge.neighbor_index, voronoi->size())
                << "Cell " << i << " has invalid neighbor " << edge.neighbor_index;
            EXPECT_NE(edge.neighbor_index, i)
                << "Cell " << i << " has itself as neighbor";
        }
    }
}

TEST(GradientDensityOptimizerTest, MassErrorsAreComputable) {
    auto voronoi = create_test_voronoi(10);
    ConstantSphericalField field(1.0);

    double target_mass = field.total_mass() / voronoi->size();
    double total_computed_mass = 0.0;

    for (const auto& cell : voronoi->dual_cells()) {
        auto moments = compute_polygon_moments(cell);
        double mass = field.mass(moments);
        total_computed_mass += mass;
    }

    EXPECT_NEAR(total_computed_mass, field.total_mass(), 0.1);
}

}
}
