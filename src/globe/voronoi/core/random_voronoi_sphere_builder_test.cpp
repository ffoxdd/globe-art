#include <gtest/gtest.h>
#include "random_voronoi_sphere_builder.hpp"
#include "../../spherical/spherical_bounding_box.hpp"

using namespace globe;

class ConstantPointGenerator {
 public:
    explicit ConstantPointGenerator(Point3 point) : _point(point) {}

    Point3 generate() {
        return _point;
    }

    Point3 generate(const SphericalBoundingBox&) {
        return generate();
    }

 private:
    Point3 _point;
};

TEST(RandomVoronoiSphereBuilderTest, CreatesSphereWithCorrectSize) {
    RandomVoronoiSphereBuilder builder;

    auto sphere = builder.build(10);
    EXPECT_EQ(sphere->size(), 10);
}

TEST(RandomVoronoiSphereBuilderTest, UsesProvidedGenerator) {
    Point3 fixed_point(1, 0, 0);
    ConstantPointGenerator generator(fixed_point);
    RandomVoronoiSphereBuilder<ConstantPointGenerator> builder(generator);

    auto sphere = builder.build(1);

    EXPECT_EQ(sphere->size(), 1);

    Point3 site = sphere->site(0);
    EXPECT_DOUBLE_EQ(site.x(), fixed_point.x());
    EXPECT_DOUBLE_EQ(site.y(), fixed_point.y());
    EXPECT_DOUBLE_EQ(site.z(), fixed_point.z());
}

