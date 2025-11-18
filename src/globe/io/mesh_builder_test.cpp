#include <gtest/gtest.h>
#include "mesh_builder.hpp"
#include "../points_collection/voronoi_sphere.hpp"
#include "../types.hpp"

using namespace globe;

namespace {

VoronoiSphere create_simple_points_collection() {
    VoronoiSphere collection;

    collection.insert(Point3(1, 0, 0));
    collection.insert(Point3(0, 1, 0));
    collection.insert(Point3(0, 0, 1));
    collection.insert(Point3(-1, 0, 0));

    return collection;
}

} // namespace

TEST(MeshBuilderTest, CreatesMeshFromVoronoiSphere) {
    VoronoiSphere collection = create_simple_points_collection();
    MeshBuilder builder;

    SurfaceMesh mesh = builder.build(collection);

    EXPECT_GT(mesh.number_of_vertices(), 0);
    EXPECT_GT(mesh.number_of_faces(), 0);
}

TEST(MeshBuilderTest, MeshHasVerticesForAllDualArcs) {
    VoronoiSphere collection = create_simple_points_collection();
    MeshBuilder builder;

    SurfaceMesh mesh = builder.build(collection);

    size_t arc_count = 0;
    for (const auto &arc : collection.dual_arcs()) {
        arc_count++;
    }

    EXPECT_GT(arc_count, 0);
    EXPECT_GE(mesh.number_of_vertices(), arc_count);
}

TEST(MeshBuilderTest, MeshIsValid) {
    VoronoiSphere collection = create_simple_points_collection();
    MeshBuilder builder;

    SurfaceMesh mesh = builder.build(collection);

    EXPECT_TRUE(mesh.is_valid());
    EXPECT_FALSE(mesh.is_empty());
}

TEST(MeshBuilderTest, RespectsConfigurationParameters) {
    VoronoiSphere collection = create_simple_points_collection();
    MeshBuilder builder_low_samples(5, 0.001);
    MeshBuilder builder_high_samples(50, 0.001);

    SurfaceMesh mesh_low = builder_low_samples.build(collection);
    SurfaceMesh mesh_high = builder_high_samples.build(collection);

    EXPECT_GT(mesh_high.number_of_vertices(), mesh_low.number_of_vertices());
    EXPECT_GT(mesh_high.number_of_faces(), mesh_low.number_of_faces());
}

