#include "gtest/gtest.h"
#include "spherical_polygon.hpp"
#include "../../../testing/arc_factory.hpp"
#include <cmath>

using namespace globe;
using globe::testing::make_arc;

TEST(SphericalPolygonTest, SimplePolygon) {
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
           make_arc(Vector3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1)),
           make_arc(Vector3(0, 1, 0), Point3(0, 0, 1), Point3(1, 0, 0)),
       }
   );

   const double sqrt3 = std::sqrt(3);

   EXPECT_TRUE(spherical_polygon.contains(Point3(1 / sqrt3, 1 / sqrt3, 1 / sqrt3)));
   EXPECT_FALSE(spherical_polygon.contains(Point3(-1, 0, 0)));
   EXPECT_FALSE(spherical_polygon.contains(Point3(-1 / sqrt3, -1 / sqrt3, -1 / sqrt3))); // opposite hemisphere
}

TEST(SphericalPolygonTest, InsideOutPolygon) {
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           make_arc(Vector3(0, -1, 0), Point3(1, 0, 0), Point3(0, 0, 1)),
           make_arc(Vector3(-1, 0, 0), Point3(0, 0, 1), Point3(0, 1, 0)),
           make_arc(Vector3(0, 0, -1), Point3(0, 1, 0), Point3(1, 0, 0)),
       }
   );

   const double sqrt3 = std::sqrt(3);

   EXPECT_FALSE(spherical_polygon.contains(Point3(1 / sqrt3, 1 / sqrt3, 1 / sqrt3)));
   EXPECT_TRUE(spherical_polygon.contains(Point3(-1, 0, 0)));
   EXPECT_TRUE(spherical_polygon.contains(Point3(-1 / sqrt3, -1 / sqrt3, -1 / sqrt3))); // opposite hemisphere
}

TEST(SphericalPolygonTest, Hemisphere) {
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
           make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
           make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
           make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
       }
   );


   EXPECT_TRUE(spherical_polygon.contains(Point3(0, 0, 1)));
   EXPECT_FALSE(spherical_polygon.contains(Point3(0, 0, -1)));
}

TEST(SphericalPolygonTest, PathologicalHemisphere) {
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(-1, 0, 0)),
           make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(1, 0, 0)),
       }
   );

   EXPECT_TRUE(spherical_polygon.contains(Point3(0, 0, 1)));
   EXPECT_FALSE(spherical_polygon.contains(Point3(0, 0, -1)));
}

TEST(SphericalPolygonTest, PointOnArcCircumcircle) {
   // Create a polygon with an arc in the z=0 plane (equator)
   // The arc goes from (1, 0, 0) to (0, 1, 0) along the equator
   // A point on the arc's supporting circle but on the arc itself should be inside
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
           make_arc(Vector3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1)),
           make_arc(Vector3(0, 1, 0), Point3(0, 0, 1), Point3(1, 0, 0)),
       }
   );

   // Point on the first arc (between (1,0,0) and (0,1,0) along the equator)
   // This point lies on the arc's supporting circle (z=0 plane), so sign == 0
   const double sqrt2 = std::sqrt(2);
   Point3 point_on_arc(sqrt2 / 2.0, sqrt2 / 2.0, 0);
   EXPECT_TRUE(spherical_polygon.contains(point_on_arc));

   // Point on the same circle but opposite side (outside the arc)
   Point3 point_on_circle_outside_arc(-sqrt2 / 2.0, sqrt2 / 2.0, 0);
   // This should be outside since it's not within the arc angle
   EXPECT_FALSE(spherical_polygon.contains(point_on_circle_outside_arc));
}

TEST(SphericalPolygonTest, PolygonWithWrappedThetaBoundingBox) {
    double theta1 = 5.8;
    double theta2 = 0.4;
    double z_low = 0.0;
    double z_high = 0.4;
    double r_low = std::sqrt(1.0 - z_low * z_low);
    double r_high = std::sqrt(1.0 - z_high * z_high);

    Point3 p1(r_low * std::cos(theta1), r_low * std::sin(theta1), z_low);
    Point3 p2(r_low * std::cos(theta2), r_low * std::sin(theta2), z_low);
    Point3 p3(r_high * std::cos(theta2), r_high * std::sin(theta2), z_high);
    Point3 p4(r_high * std::cos(theta1), r_high * std::sin(theta1), z_high);

    Vector3 v1 = CGAL::cross_product(to_position_vector(p1), to_position_vector(p2));
    Vector3 v2 = CGAL::cross_product(to_position_vector(p2), to_position_vector(p3));
    Vector3 v3 = CGAL::cross_product(to_position_vector(p3), to_position_vector(p4));
    Vector3 v4 = CGAL::cross_product(to_position_vector(p4), to_position_vector(p1));

    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(v1, p1, p2),
        make_arc(v2, p2, p3),
        make_arc(v3, p3, p4),
        make_arc(v4, p4, p1)
    });

    SphericalBoundingBox bounding_box = polygon.bounding_box();

    EXPECT_GT(bounding_box.theta_interval().end(), TWO_PI);

    Point3 center = bounding_box.center();

    double center_theta = std::atan2(center.y(), center.x());
    if (center_theta < 0.0) center_theta += 2.0 * M_PI;

    EXPECT_TRUE(bounding_box.theta_interval().contains(center_theta));
}

TEST(SphericalPolygonTest, BoundingBoxWrappedThetaMeasure) {
    double theta1 = 5.8;
    double theta2 = 0.2;
    double z_low = 0.0;
    double z_high = 0.3;
    double r_low = std::sqrt(1.0 - z_low * z_low);
    double r_high = std::sqrt(1.0 - z_high * z_high);

    Point3 p1(r_low * std::cos(theta1), r_low * std::sin(theta1), z_low);
    Point3 p2(r_low * std::cos(theta2), r_low * std::sin(theta2), z_low);
    Point3 p3(r_high * std::cos(theta2), r_high * std::sin(theta2), z_high);
    Point3 p4(r_high * std::cos(theta1), r_high * std::sin(theta1), z_high);

    Vector3 v1 = CGAL::cross_product(to_position_vector(p1), to_position_vector(p2));
    Vector3 v2 = CGAL::cross_product(to_position_vector(p2), to_position_vector(p3));
    Vector3 v3 = CGAL::cross_product(to_position_vector(p3), to_position_vector(p4));
    Vector3 v4 = CGAL::cross_product(to_position_vector(p4), to_position_vector(p1));

    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(v1, p1, p2),
        make_arc(v2, p2, p3),
        make_arc(v3, p3, p4),
        make_arc(v4, p4, p1)
    });

    SphericalBoundingBox bounding_box = polygon.bounding_box();

    EXPECT_GT(bounding_box.theta_interval().end(), TWO_PI);

    double expected_theta_measure = (2.0 * M_PI - 5.8) + 0.2;
    EXPECT_NEAR(bounding_box.theta_interval().measure(), expected_theta_measure, 0.05);
}

TEST(SphericalPolygonTest, BoundingSphereRadiusWithWrappedTheta) {
    double theta_start = 5.5;
    double theta_measure = (TWO_PI - 5.5) + 0.8;
    ThetaInterval wrapped_theta(theta_start, theta_measure);
    Interval z_interval(0.0, 0.5);
    SphericalBoundingBox bounding_box(wrapped_theta, z_interval);

    double radius = bounding_box.bounding_sphere_radius();

    EXPECT_GT(radius, 0.0);

    double theta_span = bounding_box.theta_interval().measure();
    double z_span = bounding_box.z_interval().measure();
    double r_max = std::sqrt(1.0 - z_interval.low() * z_interval.low());
    double chord = 2.0 * r_max * std::sin(theta_span / 2.0);
    double expected_radius = std::sqrt(z_span * z_span + chord * chord);

    EXPECT_NEAR(radius, expected_radius, 1e-9);
}

TEST(SphericalPolygonTest, CentroidReturnsPointOnUnitSphere) {
    SphericalPolygon polygon = SphericalPolygon(
        std::vector<Arc>{
            make_arc(Vector3(Vector3(0, 0, 1)), Point3(Point3(1, 0, 0)), Point3(Point3(0, 1, 0))),
            make_arc(Vector3(Vector3(1, 0, 0)), Point3(Point3(0, 1, 0)), Point3(Point3(0, 0, 1))),
            make_arc(Vector3(Vector3(0, 1, 0)), Point3(Point3(0, 0, 1)), Point3(Point3(1, 0, 0))),
        }
    );

    Point3 centroid = polygon.centroid();

    double distance = std::sqrt(
        centroid.x() * centroid.x() +
        centroid.y() * centroid.y() +
        centroid.z() * centroid.z()
    );
    EXPECT_NEAR(distance, 1.0, 1e-9);
}

TEST(SphericalPolygonTest, CentroidIsOnUnitSphere) {
    SphericalPolygon polygon = SphericalPolygon(
        std::vector<Arc>{
            make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
            make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
            make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
            make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
        }
    );

    Point3 centroid = polygon.centroid();

    double distance = std::sqrt(
        centroid.x() * centroid.x() +
        centroid.y() * centroid.y() +
        centroid.z() * centroid.z()
    );
    EXPECT_NEAR(distance, 1.0, 1e-9);
}

TEST(SphericalPolygonTest, CentroidForSymmetricPolygon) {
    const double sqrt3 = std::sqrt(3);
    SphericalPolygon polygon = SphericalPolygon(
        std::vector<Arc>{
            make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0.5, sqrt3 / 2, 0)),
            make_arc(Vector3(0, 0, 1), Point3(0.5, sqrt3 / 2, 0), Point3(-0.5, sqrt3 / 2, 0)),
            make_arc(Vector3(0, 0, 1), Point3(-0.5, sqrt3 / 2, 0), Point3(-1, 0, 0)),
            make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(-0.5, -sqrt3 / 2, 0)),
            make_arc(Vector3(0, 0, 1), Point3(-0.5, -sqrt3 / 2, 0), Point3(0.5, -sqrt3 / 2, 0)),
            make_arc(Vector3(0, 0, 1), Point3(0.5, -sqrt3 / 2, 0), Point3(1, 0, 0)),
        }
    );

    Point3 centroid = polygon.centroid();

    double distance = std::sqrt(
        centroid.x() * centroid.x() +
        centroid.y() * centroid.y() +
        centroid.z() * centroid.z()
    );
    EXPECT_NEAR(distance, 1.0, 1e-9);
    EXPECT_NEAR(centroid.z(), 0.5, 1e-9);
}

TEST(SphericalPolygonTest, ArcThetaExtrema_FullCircle) {
    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0))
    });

    ThetaInterval theta_interval = polygon.bounding_box().theta_interval();

    EXPECT_TRUE(theta_interval.is_full());
}

TEST(SphericalPolygonTest, BoundingBox_IncludesNorthPoleZ) {
    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0))
    });

    EXPECT_TRUE(polygon.contains(NORTH_POLE));

    SphericalBoundingBox bounding_box = polygon.bounding_box();
    Interval z_interval = bounding_box.z_interval();

    EXPECT_LE(z_interval.low(), 0.0);
    EXPECT_GE(z_interval.high(), 1.0);
    EXPECT_TRUE(z_interval.contains(1.0));
}

TEST(SphericalPolygonTest, BoundingBox_IncludesSouthPoleZ) {
    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, -1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, -1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, -1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, -1), Point3(0, -1, 0), Point3(1, 0, 0))
    });

    EXPECT_TRUE(polygon.contains(SOUTH_POLE));

    SphericalBoundingBox bounding_box = polygon.bounding_box();
    Interval z_interval = bounding_box.z_interval();

    EXPECT_GE(z_interval.high(), 0.0);
    EXPECT_LE(z_interval.low(), -1.0);
    EXPECT_TRUE(z_interval.contains(-1.0));
}

TEST(SphericalPolygonTest, BoundingBox_PoleZEvenWhenArcsDontReach) {
    double z_arc = 0.9;
    double r_arc = std::sqrt(1.0 - z_arc * z_arc);

    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, 1), Point3(r_arc, 0, z_arc), Point3(0, r_arc, z_arc)),
        make_arc(Vector3(0, 0, 1), Point3(0, r_arc, z_arc), Point3(-r_arc, 0, z_arc)),
        make_arc(Vector3(0, 0, 1), Point3(-r_arc, 0, z_arc), Point3(0, -r_arc, z_arc)),
        make_arc(Vector3(0, 0, 1), Point3(0, -r_arc, z_arc), Point3(r_arc, 0, z_arc))
    });

    EXPECT_TRUE(polygon.contains(NORTH_POLE));

    SphericalBoundingBox bounding_box = polygon.bounding_box();
    Interval z_interval = bounding_box.z_interval();

    EXPECT_LE(z_interval.low(), z_arc);
    EXPECT_GE(z_interval.high(), 1.0);
    EXPECT_TRUE(z_interval.contains(1.0));
}

TEST(SphericalPolygonTest, BoundingSphereRadiusContainsAllVertices) {
    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1)),
        make_arc(Vector3(0, 1, 0), Point3(0, 0, 1), Point3(1, 0, 0)),
    });

    Point3 centroid = polygon.centroid();
    double radius = polygon.bounding_sphere_radius();

    for (const auto &point : polygon.points()) {
        double distance = std::sqrt(CGAL::squared_distance(point, centroid));
        EXPECT_LE(distance, radius + GEOMETRIC_EPSILON);
    }
}

TEST(SphericalPolygonTest, BoundingSphereRadiusIsMinimalForSymmetricPolygon) {
    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
    });

    double radius = polygon.bounding_sphere_radius();

    double max_distance = 0.0;
    Point3 centroid = polygon.centroid();
    for (const auto &point : polygon.points()) {
        double distance = std::sqrt(CGAL::squared_distance(point, centroid));
        max_distance = std::max(max_distance, distance);
    }

    EXPECT_NEAR(radius, max_distance, GEOMETRIC_EPSILON);
}

TEST(SphericalPolygonTest, AreaOfOctant) {
    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1)),
        make_arc(Vector3(0, 1, 0), Point3(0, 0, 1), Point3(1, 0, 0)),
    });

    double expected_area = M_PI / 2.0;
    EXPECT_NEAR(polygon.area(), expected_area, 1e-9);
}

TEST(SphericalPolygonTest, AreaOfHemisphere) {
    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, 1, 0), Point3(-1, 0, 0)),
        make_arc(Vector3(0, 0, 1), Point3(-1, 0, 0), Point3(0, -1, 0)),
        make_arc(Vector3(0, 0, 1), Point3(0, -1, 0), Point3(1, 0, 0)),
    });

    double expected_area = 2.0 * M_PI;
    EXPECT_NEAR(polygon.area(), expected_area, 1e-9);
}
