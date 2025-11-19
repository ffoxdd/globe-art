#include <gtest/gtest.h>
#include "mesh_builder.hpp"
#include "../../types.hpp"

using namespace globe;

TEST(MeshBuilderTest, BuildsArcBetweenTwoPoints) {
    MeshBuilder builder(10, 0.001);
    SurfaceMesh mesh;

    Point3 source(1, 0, 0);
    Point3 target(0, 1, 0);

    builder.build_arc(mesh, source, target);

    EXPECT_GT(mesh.number_of_vertices(), 0);
    EXPECT_GT(mesh.number_of_faces(), 0);
}

TEST(MeshBuilderTest, MeshIsValidAfterBuildingArc) {
    MeshBuilder builder(10, 0.001);
    SurfaceMesh mesh;

    Point3 source(1, 0, 0);
    Point3 target(0, 1, 0);

    builder.build_arc(mesh, source, target);

    EXPECT_TRUE(mesh.is_valid());
    EXPECT_FALSE(mesh.is_empty());
}

TEST(MeshBuilderTest, BuildsMultipleArcsInSameMesh) {
    MeshBuilder builder(10, 0.001);
    SurfaceMesh mesh;

    Point3 p1(1, 0, 0);
    Point3 p2(0, 1, 0);
    Point3 p3(0, 0, 1);

    builder.build_arc(mesh, p1, p2);
    builder.build_arc(mesh, p2, p3);
    builder.build_arc(mesh, p3, p1);

    EXPECT_GT(mesh.number_of_vertices(), 0);
    EXPECT_GT(mesh.number_of_faces(), 0);
    EXPECT_TRUE(mesh.is_valid());
}

TEST(MeshBuilderTest, HigherSamplesProduceMoreVertices) {
    MeshBuilder builder_low(5, 0.001);
    MeshBuilder builder_high(50, 0.001);

    SurfaceMesh mesh_low;
    SurfaceMesh mesh_high;

    Point3 source(1, 0, 0);
    Point3 target(0, 1, 0);

    builder_low.build_arc(mesh_low, source, target);
    builder_high.build_arc(mesh_high, source, target);

    EXPECT_GT(mesh_high.number_of_vertices(), mesh_low.number_of_vertices());
    EXPECT_GT(mesh_high.number_of_faces(), mesh_low.number_of_faces());
}

TEST(MeshBuilderTest, DeduplicatesSharedVertices) {
    MeshBuilder builder(10, 0.001);
    SurfaceMesh mesh;

    Point3 p1(1, 0, 0);
    Point3 p2(0, 1, 0);
    Point3 p3(0, 0, 1);

    builder.build_arc(mesh, p1, p2);
    size_t vertices_after_first = mesh.number_of_vertices();

    builder.build_arc(mesh, p2, p3);
    size_t vertices_after_second = mesh.number_of_vertices();

    EXPECT_LT(vertices_after_second - vertices_after_first, vertices_after_first);
}

