#include <gtest/gtest.h>
#include "voronoi_sphere.hpp"
#include "../types.hpp"

using namespace globe;

namespace {

VoronoiSphere create_simple_voronoi_sphere() {
    VoronoiSphere sphere;

    sphere.insert(Point3(1, 0, 0));
    sphere.insert(Point3(0, 1, 0));
    sphere.insert(Point3(0, 0, 1));
    sphere.insert(Point3(-1, 0, 0));

    return sphere;
}

} // namespace

TEST(VoronoiSphereTest, DualCellsReturnsSphericalPolygons) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();

    size_t cell_count = 0;
    for (const auto &cell : sphere.dual_cells()) {
        EXPECT_TRUE(cell.is_valid());
        cell_count++;
    }

    EXPECT_EQ(cell_count, sphere.size());
}

TEST(VoronoiSphereTest, DualCellsMatchesDualCellArcs) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();

    size_t index = 0;
    for (const auto &cell : sphere.dual_cells()) {
        std::vector<Arc> expected_arcs = sphere.dual_cell_arcs(index);
        auto cell_arcs = cell.arcs();

        EXPECT_EQ(cell_arcs.size(), expected_arcs.size());
        index++;
    }
}

TEST(VoronoiSphereTest, DualCellsIsEmptyForEmptySphere) {
    VoronoiSphere sphere;

    size_t cell_count = 0;
    for (const auto &cell : sphere.dual_cells()) {
        cell_count++;
    }

    EXPECT_EQ(cell_count, 0);
}

