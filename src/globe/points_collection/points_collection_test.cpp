#include <gtest/gtest.h>
#include "points_collection.hpp"

using namespace globe;

template<typename T>
bool contains(const std::vector<T> &vector, const T &element) {
    return std::find(vector.begin(), vector.end(), element) != vector.end();
}

TEST(PointsCollectionTest, InsertAndEmptyTest) {
    PointsCollection points_collection;
    Point3 point(1.0, 2.0, 3.0);

    EXPECT_TRUE(points_collection.empty());

    points_collection.insert(point);
    EXPECT_FALSE(points_collection.empty());
}

TEST(PointsCollectionTest, NearbyPointsTest) {
    PointsCollection points_collection;
    Point3 point1(1.0, 2.0, 3.0);
    Point3 point2(2.0, 3.0, 4.0);
    Point3 point3(10.0, 10.0, 10.0);

    points_collection.insert(point1);
    points_collection.insert(point2);
    points_collection.insert(point3);

    Point3 search_point(1.5, 2.5, 3.5);
    double radius = 5.0;

    auto nearby_points = points_collection.nearby_points(search_point, radius);

    EXPECT_EQ(nearby_points.size(), 2);
    EXPECT_TRUE(contains(nearby_points, point1));
    EXPECT_TRUE(contains(nearby_points, point2));
    EXPECT_FALSE(contains(nearby_points, point3));
}
