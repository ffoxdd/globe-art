#include <gtest/gtest.h>
#include "builder.hpp"

using namespace globe;
using namespace globe::io::ply::mesh;

TEST(BuilderTest, BuildsArcBetweenTwoPoints) {
    Builder builder(10, 0.001);

    Arc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0));

    builder.add_arc(arc);
    SurfaceMesh mesh = builder.build();

    EXPECT_GT(mesh.number_of_vertices(), 0);
    EXPECT_GT(mesh.number_of_faces(), 0);
}

TEST(BuilderTest, MeshIsValidAfterBuildingArc) {
    Builder builder(10, 0.001);

    Arc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0));

    builder.add_arc(arc);
    SurfaceMesh mesh = builder.build();

    EXPECT_TRUE(mesh.is_valid());
    EXPECT_FALSE(mesh.is_empty());
}

TEST(BuilderTest, BuildsMultipleArcsInSameMesh) {
    Builder builder(10, 0.001);

    VectorS2 p1(1, 0, 0);
    VectorS2 p2(0, 1, 0);
    VectorS2 p3(0, 0, 1);

    builder.add_arc(Arc(p1, p2));
    builder.add_arc(Arc(p2, p3));
    builder.add_arc(Arc(p3, p1));
    SurfaceMesh mesh = builder.build();

    EXPECT_GT(mesh.number_of_vertices(), 0);
    EXPECT_GT(mesh.number_of_faces(), 0);
    EXPECT_TRUE(mesh.is_valid());
}

TEST(BuilderTest, HigherSamplesProduceMoreVertices) {
    Builder builder_low(5, 0.001);
    Builder builder_high(50, 0.001);

    Arc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0));

    builder_low.add_arc(arc);
    builder_high.add_arc(arc);

    SurfaceMesh mesh_low = builder_low.build();
    SurfaceMesh mesh_high = builder_high.build();

    EXPECT_GT(mesh_high.number_of_vertices(), mesh_low.number_of_vertices());
    EXPECT_GT(mesh_high.number_of_faces(), mesh_low.number_of_faces());
}

TEST(BuilderTest, DeduplicatesSharedVertices) {
    Builder single_arc_builder(10, 0.001);
    Builder two_arc_builder(10, 0.001);

    VectorS2 p1(1, 0, 0);
    VectorS2 p2(0, 1, 0);
    VectorS2 p3(0, 0, 1);

    single_arc_builder.add_arc(Arc(p1, p2));
    SurfaceMesh single_arc_mesh = single_arc_builder.build();

    two_arc_builder.add_arc(Arc(p1, p2));
    two_arc_builder.add_arc(Arc(p2, p3));
    SurfaceMesh two_arc_mesh = two_arc_builder.build();

    size_t vertices_for_one_arc = single_arc_mesh.number_of_vertices();
    size_t vertices_for_two_arcs = two_arc_mesh.number_of_vertices();

    EXPECT_LT(vertices_for_two_arcs, vertices_for_one_arc * 2);
}

TEST(BuilderTest, BuildClearsMeshForReuse) {
    Builder builder(10, 0.001);

    Arc arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0));

    builder.add_arc(arc);
    SurfaceMesh first_mesh = builder.build();

    builder.add_arc(arc);
    SurfaceMesh second_mesh = builder.build();

    EXPECT_EQ(first_mesh.number_of_vertices(), second_mesh.number_of_vertices());
    EXPECT_EQ(first_mesh.number_of_faces(), second_mesh.number_of_faces());
}
