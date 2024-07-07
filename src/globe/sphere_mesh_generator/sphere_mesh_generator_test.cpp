#include <gtest/gtest.h>
#include "sphere_mesh_generator.hpp"
#include <cmath>

using namespace globe;

const double MAX_ERROR_THRESHOLD = 1e-6;

double compute_max_deviation(const SurfaceMesh &mesh, const Point3 &center, double radius) {
    std::vector<double> squared_deviations(mesh.number_of_vertices());
    double squared_radius = radius * radius;

    std::transform(mesh.vertices().begin(), mesh.vertices().end(), squared_deviations.begin(),
        [&](const SurfaceMesh::Vertex_index &vertex) {
            return std::abs(CGAL::squared_distance(mesh.point(vertex), center) - squared_radius);
        }
    );

    double max_squared_deviation = *std::max_element(squared_deviations.begin(), squared_deviations.end());
    return std::sqrt(max_squared_deviation);
}

TEST(SphereMeshGeneratorTest, GeneratesASphericalMesh) {
    double radius = 1.0;
    int iterations = 3;
    Point3 center(0, 0, 0);

    SurfaceMesh mesh = globe::SphereMeshGenerator::generate(radius, iterations, center);
    double max_deviation = compute_max_deviation(mesh, center, radius);

    EXPECT_FALSE(mesh.is_empty());
    EXPECT_LT(max_deviation, MAX_ERROR_THRESHOLD) << "Maximum distance is above the threshold.";
}

TEST(SphereMeshGeneratorTest, WorksWithAnArbitraryCenterPoint) {
    double radius = 1.0;
    int iterations = 3;
    Point3 center(1.5, 2.4, -4.7);

    SurfaceMesh mesh = globe::SphereMeshGenerator::generate(radius, iterations, center);
    double max_deviation = compute_max_deviation(mesh, center, radius);

    EXPECT_FALSE(mesh.is_empty());
    EXPECT_LT(max_deviation, MAX_ERROR_THRESHOLD) << "Maximum distance is above the threshold.";
}

TEST(SphereMeshGeneratorTest, WorksWithAnArbitraryRadius) {
    double radius = 5.8;
    int iterations = 3;
    Point3 center(0, 0, 0);

    SurfaceMesh mesh = globe::SphereMeshGenerator::generate(radius, iterations, center);
    double max_deviation = compute_max_deviation(mesh, center, radius);

    EXPECT_FALSE(mesh.is_empty());
    EXPECT_LT(max_deviation, MAX_ERROR_THRESHOLD) << "Maximum distance is above the threshold.";
}
