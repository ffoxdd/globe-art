#include <gtest/gtest.h>
#include "voronoi_sphere_builder.hpp"
#include "../../../voronoi/core/voronoi_sphere.hpp"

using namespace globe;
using namespace globe::io::ply::mesh;

VoronoiSphere create_simple_voronoi_sphere() {
    VoronoiSphere sphere;

    sphere.insert(cgal::Point3(1, 0, 0));
    sphere.insert(cgal::Point3(0, 1, 0));
    sphere.insert(cgal::Point3(0, 0, 1));
    sphere.insert(cgal::Point3(-1, 0, 0));

    return sphere;
}

TEST(VoronoiSphereBuilderTest, CreatesMeshFromVoronoiSphere) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();
    VoronoiSphereBuilder builder;

    SurfaceMesh mesh = builder.build(sphere);

    EXPECT_GT(mesh.number_of_vertices(), 0);
    EXPECT_GT(mesh.number_of_faces(), 0);
}

TEST(VoronoiSphereBuilderTest, MeshIsValid) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();
    VoronoiSphereBuilder builder;

    SurfaceMesh mesh = builder.build(sphere);

    EXPECT_TRUE(mesh.is_valid());
    EXPECT_FALSE(mesh.is_empty());
}

TEST(VoronoiSphereBuilderTest, MeshHasVerticesForAllArcs) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();
    VoronoiSphereBuilder builder;

    SurfaceMesh mesh = builder.build(sphere);

    size_t arc_count = 0;
    for (const auto &cell : sphere.cells()) {
        arc_count += cell.arcs().size();
    }

    EXPECT_GT(arc_count, 0);
    EXPECT_GE(mesh.number_of_vertices(), arc_count);
}

TEST(VoronoiSphereBuilderTest, HigherSamplesProduceMoreVertices) {
    VoronoiSphere sphere = create_simple_voronoi_sphere();
    VoronoiSphereBuilder builder_low(5, 0.001);
    VoronoiSphereBuilder builder_high(50, 0.001);

    SurfaceMesh mesh_low = builder_low.build(sphere);
    SurfaceMesh mesh_high = builder_high.build(sphere);

    EXPECT_GT(mesh_high.number_of_vertices(), mesh_low.number_of_vertices());
    EXPECT_GT(mesh_high.number_of_faces(), mesh_low.number_of_faces());
}

TEST(VoronoiSphereBuilderTest, EmptyVoronoiSphereProducesEmptyMesh) {
    VoronoiSphere sphere;
    VoronoiSphereBuilder builder;

    SurfaceMesh mesh = builder.build(sphere);

    EXPECT_EQ(mesh.number_of_vertices(), 0);
    EXPECT_EQ(mesh.number_of_faces(), 0);
    EXPECT_TRUE(mesh.is_empty());
}

TEST(VoronoiSphereBuilderTest, SinglePointProducesNoMesh) {
    VoronoiSphere sphere;
    sphere.insert(cgal::Point3(1, 0, 0));
    VoronoiSphereBuilder builder;

    SurfaceMesh mesh = builder.build(sphere);

    EXPECT_EQ(mesh.number_of_vertices(), 0);
    EXPECT_EQ(mesh.number_of_faces(), 0);
}

TEST(VoronoiSphereBuilderTest, MorePointsProduceMoreDualArcs) {
    VoronoiSphere small_sphere = create_simple_voronoi_sphere();

    VoronoiSphere large_sphere = create_simple_voronoi_sphere();
    large_sphere.insert(cgal::Point3(0, -1, 0));
    large_sphere.insert(cgal::Point3(0, 0, -1));

    VoronoiSphereBuilder builder;

    SurfaceMesh small_mesh = builder.build(small_sphere);
    SurfaceMesh large_mesh = builder.build(large_sphere);

    EXPECT_GT(large_mesh.number_of_vertices(), small_mesh.number_of_vertices());
    EXPECT_GT(large_mesh.number_of_faces(), small_mesh.number_of_faces());
}
