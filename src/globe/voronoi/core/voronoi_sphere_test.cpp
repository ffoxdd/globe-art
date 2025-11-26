#include <gtest/gtest.h>
#include "voronoi_sphere.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../testing/geometric_assertions.hpp"

using namespace globe;
using globe::testing::points_approximately_equal;

VoronoiSphere create_simple_voronoi_sphere() {
    VoronoiSphere sphere;

    sphere.insert(Point3(1, 0, 0));
    sphere.insert(Point3(0, 1, 0));
    sphere.insert(Point3(0, 0, 1));
    sphere.insert(Point3(-1, 0, 0));

    return sphere;
}

TEST(VoronoiSphereTest, DefaultConstructorCreatesEmptySphere) {
    VoronoiSphere sphere;
    EXPECT_EQ(sphere.size(), 0);
}

TEST(VoronoiSphereTest, InsertIncreasesSize) {
    VoronoiSphere sphere;
    EXPECT_EQ(sphere.size(), 0);

    sphere.insert(Point3(1, 0, 0));
    EXPECT_EQ(sphere.size(), 1);

    sphere.insert(Point3(0, 1, 0));
    EXPECT_EQ(sphere.size(), 2);

    sphere.insert(Point3(0, 0, 1));
    EXPECT_EQ(sphere.size(), 3);
}

TEST(VoronoiSphereTest, SiteReturnsInsertedPoint) {
    VoronoiSphere sphere;
    Point3 point(1, 0, 0);
    sphere.insert(point);

    Point3 retrieved = sphere.site(0);
    EXPECT_TRUE(points_approximately_equal(point, retrieved));
}

TEST(VoronoiSphereTest, SiteReturnsCorrectPointsForMultipleSites) {
    VoronoiSphere sphere;
    Point3 p1(1, 0, 0);
    Point3 p2(0, 1, 0);
    Point3 p3(0, 0, 1);

    sphere.insert(p1);
    sphere.insert(p2);
    sphere.insert(p3);

    EXPECT_TRUE(points_approximately_equal(sphere.site(0), p1));
    EXPECT_TRUE(points_approximately_equal(sphere.site(1), p2));
    EXPECT_TRUE(points_approximately_equal(sphere.site(2), p3));
}

TEST(VoronoiSphereTest, UpdateSiteChangesPosition) {
    VoronoiSphere sphere;
    Point3 original(1, 0, 0);
    Point3 updated(0, 1, 0);

    sphere.insert(original);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), original));

    sphere.update_site(0, updated);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), updated));
}

TEST(VoronoiSphereTest, DualArcsReturnsNonEmptyRange) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();

    size_t arc_count = 0;
    for (const auto &arc : sphere.dual_arcs()) {
        arc_count++;
    }

    EXPECT_GT(arc_count, 0);
}

TEST(VoronoiSphereTest, DualArcsIsEmptyForEmptySphere) {
    VoronoiSphere sphere;

    size_t arc_count = 0;
    for (const auto &arc : sphere.dual_arcs()) {
        arc_count++;
    }

    EXPECT_EQ(arc_count, 0);
}

TEST(VoronoiSphereTest, DualArcsIsEmptyForSinglePoint) {
    VoronoiSphere sphere;
    sphere.insert(Point3(1, 0, 0));

    size_t arc_count = 0;
    for (const auto &arc : sphere.dual_arcs()) {
        arc_count++;
    }

    EXPECT_EQ(arc_count, 0);
}

TEST(VoronoiSphereTest, DualCellsReturnsSphericalPolygons) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();

    size_t cell_count = 0;
    for (const auto &cell : sphere.dual_cells()) {
        // Constructor already validates with CGAL_precondition
        cell_count++;
    }

    EXPECT_EQ(cell_count, sphere.size());
}

TEST(VoronoiSphereTest, DualCellsIsEmptyForEmptySphere) {
    VoronoiSphere sphere;

    size_t cell_count = 0;
    for (const auto &cell : sphere.dual_cells()) {
        cell_count++;
    }

    EXPECT_EQ(cell_count, 0);
}

TEST(VoronoiSphereTest, MultipleUpdatesToSameSite) {
    VoronoiSphere sphere;
    sphere.insert(Point3(1, 0, 0));

    Point3 pos1 = project_to_sphere(Point3(0, 1, 0));
    Point3 pos2 = project_to_sphere(Point3(0, 0, 1));
    Point3 pos3 = project_to_sphere(Point3(1, 1, 1));

    sphere.update_site(0, pos1);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), pos1));

    sphere.update_site(0, pos2);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), pos2));

    sphere.update_site(0, pos3);
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), pos3));
}

TEST(VoronoiSphereTest, UpdateSitePreservesOtherSites) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();
    Point3 p0 = sphere.site(0);
    Point3 p1 = sphere.site(1);
    Point3 p2 = sphere.site(2);
    Point3 p3 = sphere.site(3);

    // Update one site to a different but valid position
    Point3 updated_p1 = project_to_sphere(Point3(0.707, 0.707, 0));
    sphere.update_site(1, updated_p1);

    // Other sites should remain unchanged
    EXPECT_TRUE(points_approximately_equal(sphere.site(0), p0));
    EXPECT_TRUE(points_approximately_equal(sphere.site(1), updated_p1));
    EXPECT_TRUE(points_approximately_equal(sphere.site(2), p2));
    EXPECT_TRUE(points_approximately_equal(sphere.site(3), p3));
}

