#include <gtest/gtest.h>
#include "voronoi_sphere.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../testing/assertions/geometric.hpp"

using namespace globe;
using globe::testing::points_approximately_equal;

VoronoiSphere create_simple_voronoi_sphere() {
    VoronoiSphere sphere;

    sphere.insert(cgal::Point3(1, 0, 0));
    sphere.insert(cgal::Point3(0, 1, 0));
    sphere.insert(cgal::Point3(0, 0, 1));
    sphere.insert(cgal::Point3(-1, 0, 0));

    return sphere;
}

TEST(VoronoiSphereTest, DefaultConstructorCreatesEmptySphere) {
    VoronoiSphere sphere;
    EXPECT_EQ(sphere.size(), 0);
}

TEST(VoronoiSphereTest, InsertIncreasesSize) {
    VoronoiSphere sphere;
    EXPECT_EQ(sphere.size(), 0);

    sphere.insert(cgal::Point3(1, 0, 0));
    EXPECT_EQ(sphere.size(), 1);

    sphere.insert(cgal::Point3(0, 1, 0));
    EXPECT_EQ(sphere.size(), 2);

    sphere.insert(cgal::Point3(0, 0, 1));
    EXPECT_EQ(sphere.size(), 3);
}

TEST(VoronoiSphereTest, SiteReturnsInsertedPoint) {
    VoronoiSphere sphere;
    cgal::Point3 point(1, 0, 0);
    sphere.insert(point);

    cgal::Point3 retrieved = sphere.site(0);
    EXPECT_TRUE(points_approximately_equal(point, retrieved));
}

TEST(VoronoiSphereTest, SiteReturnsCorrectPointsForMultipleSites) {
    VoronoiSphere sphere;
    cgal::Point3 p1(1, 0, 0);
    cgal::Point3 p2(0, 1, 0);
    cgal::Point3 p3(0, 0, 1);

    sphere.insert(p1);
    sphere.insert(p2);
    sphere.insert(p3);

    EXPECT_TRUE(points_approximately_equal(sphere.site(0), p1));
    EXPECT_TRUE(points_approximately_equal(sphere.site(1), p2));
    EXPECT_TRUE(points_approximately_equal(sphere.site(2), p3));
}

TEST(VoronoiSphereTest, UpdateSiteChangesPosition) {
    VoronoiSphere sphere;
    cgal::Point3 original(1, 0, 0);
    cgal::Point3 updated(0, 1, 0);

    sphere.insert(original);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), original));

    sphere.update_site(0, updated);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), updated));
}

TEST(VoronoiSphereTest, CellArcsReturnsNonEmptyRange) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();

    size_t arc_count = 0;
    for (const auto &cell : sphere.cells()) {
        arc_count += cell.arcs().size();
    }

    EXPECT_GT(arc_count, 0);
}

TEST(VoronoiSphereTest, CellArcsIsEmptyForEmptySphere) {
    VoronoiSphere sphere;

    size_t arc_count = 0;
    for (const auto &cell : sphere.cells()) {
        arc_count += cell.arcs().size();
    }

    EXPECT_EQ(arc_count, 0);
}

TEST(VoronoiSphereTest, CellArcsIsEmptyForSinglePoint) {
    VoronoiSphere sphere;
    sphere.insert(cgal::Point3(1, 0, 0));

    size_t arc_count = 0;
    for (const auto &cell : sphere.cells()) {
        arc_count += cell.arcs().size();
    }

    EXPECT_EQ(arc_count, 0);
}

TEST(VoronoiSphereTest, DualCellsReturnsPolygons) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();

    size_t cell_count = 0;
    for (const auto &cell : sphere.cells()) {
        // Constructor already validates with CGAL_precondition
        cell_count++;
    }

    EXPECT_EQ(cell_count, sphere.size());
}

TEST(VoronoiSphereTest, DualCellsIsEmptyForEmptySphere) {
    VoronoiSphere sphere;

    size_t cell_count = 0;
    for (const auto &cell : sphere.cells()) {
        cell_count++;
    }

    EXPECT_EQ(cell_count, 0);
}

TEST(VoronoiSphereTest, MultipleUpdatesToSameSite) {
    VoronoiSphere sphere;
    sphere.insert(cgal::Point3(1, 0, 0));

    cgal::Point3 pos1 = cgal::to_point(VectorS2(0, 1, 0).normalized());
    cgal::Point3 pos2 = cgal::to_point(VectorS2(0, 0, 1).normalized());
    cgal::Point3 pos3 = cgal::to_point(VectorS2(1, 1, 1).normalized());

    sphere.update_site(0, pos1);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), pos1));

    sphere.update_site(0, pos2);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), pos2));

    sphere.update_site(0, pos3);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), pos3));
}

TEST(VoronoiSphereTest, UpdateSitePreservesOtherSites) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();
    cgal::Point3 p0 = sphere.site(0);
    cgal::Point3 p1 = sphere.site(1);
    cgal::Point3 p2 = sphere.site(2);
    cgal::Point3 p3 = sphere.site(3);

    // Update one site to a different but valid position
    cgal::Point3 updated_p1 = cgal::to_point(VectorS2(0.707, 0.707, 0).normalized());
    sphere.update_site(1, updated_p1);

    // Other sites should remain unchanged
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), p0));
    EXPECT_TRUE(points_approximately_equal(sphere.site(1), updated_p1));
    EXPECT_TRUE(points_approximately_equal(sphere.site(2), p2));
    EXPECT_TRUE(points_approximately_equal(sphere.site(3), p3));
}

TEST(VoronoiSphereTest, ArcsReturnsAllCellArcs) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();

    size_t arcs_via_cells = 0;
    for (const auto &cell : sphere.cells()) {
        arcs_via_cells += cell.arcs().size();
    }

    size_t arcs_via_arcs = 0;
    for (const auto &arc : sphere.arcs()) {
        (void)arc;
        arcs_via_arcs++;
    }

    EXPECT_EQ(arcs_via_arcs, arcs_via_cells);
    EXPECT_GT(arcs_via_arcs, 0);
}

TEST(VoronoiSphereTest, ArcsIsEmptyForEmptySphere) {
    VoronoiSphere sphere;

    size_t arc_count = 0;
    for (const auto &arc : sphere.arcs()) {
        (void)arc;
        arc_count++;
    }

    EXPECT_EQ(arc_count, 0);
}

TEST(VoronoiSphereTest, ArcsIsEmptyForSinglePoint) {
    VoronoiSphere sphere;
    sphere.insert(cgal::Point3(1, 0, 0));

    size_t arc_count = 0;
    for (const auto &arc : sphere.arcs()) {
        (void)arc;
        arc_count++;
    }

    EXPECT_EQ(arc_count, 0);
}

