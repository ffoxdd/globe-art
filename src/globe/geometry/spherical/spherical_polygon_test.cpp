#include "gtest/gtest.h"
#include "spherical_polygon.hpp"
#include "../../testing/arc_factory.hpp"
#include <cmath>

using namespace globe;
using globe::testing::make_arc;

TEST(SphericalPolygonTest, SimplePolygon) {
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           make_arc(0, 0, 1, 1, 0, 0, 0, 1, 0),
           make_arc(1, 0, 0, 0, 1, 0, 0, 0, 1),
           make_arc(0, 1, 0, 0, 0, 1, 1, 0, 0),
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
           make_arc(0, -1, 0, 1, 0, 0, 0, 0, 1),
           make_arc(-1, 0, 0, 0, 0, 1, 0, 1, 0),
           make_arc(0, 0, -1, 0, 1, 0, 1, 0, 0),
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
           make_arc(0, 0, 1, 1, 0, 0, 0, 1, 0),
           make_arc(0, 0, 1, 0, 1, 0, -1, 0, 0),
           make_arc(0, 0, 1, -1, 0, 0, 0, -1, 0),
           make_arc(0, 0, 1, 0, -1, 0, 1, 0, 0),
       }
   );


   EXPECT_TRUE(spherical_polygon.contains(Point3(0, 0, 1)));
   EXPECT_FALSE(spherical_polygon.contains(Point3(0, 0, -1)));
}

TEST(SphericalPolygonTest, PathologicalHemisphere) {
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           make_arc(0, 0, 1, 1, 0, 0, -1, 0, 0),
           make_arc(0, 0, 1, -1, 0, 0, 1, 0, 0),
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
           make_arc(0, 0, 1, 1, 0, 0, 0, 1, 0),
           make_arc(1, 0, 0, 0, 1, 0, 0, 0, 1),
           make_arc(0, 1, 0, 0, 0, 1, 1, 0, 0),
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

    Vector3 v1 = CGAL::cross_product(position_vector(p1), position_vector(p2));
    Vector3 v2 = CGAL::cross_product(position_vector(p2), position_vector(p3));
    Vector3 v3 = CGAL::cross_product(position_vector(p3), position_vector(p4));
    Vector3 v4 = CGAL::cross_product(position_vector(p4), position_vector(p1));

    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(v1, p1, p2),
        make_arc(v2, p2, p3),
        make_arc(v3, p3, p4),
        make_arc(v4, p4, p1)
    });

    SphericalBoundingBox bbox = polygon.bounding_box();

    EXPECT_TRUE(bbox.is_theta_wrapped());

    Point3 center = bbox.center();

    double center_theta = std::atan2(center.y(), center.x());
    if (center_theta < 0.0) center_theta += 2.0 * M_PI;

    bool center_in_wrapped_interval =
        (center_theta >= bbox.theta_interval().low()) || (center_theta <= bbox.theta_interval().high());
    EXPECT_TRUE(center_in_wrapped_interval);
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

    Vector3 v1 = CGAL::cross_product(position_vector(p1), position_vector(p2));
    Vector3 v2 = CGAL::cross_product(position_vector(p2), position_vector(p3));
    Vector3 v3 = CGAL::cross_product(position_vector(p3), position_vector(p4));
    Vector3 v4 = CGAL::cross_product(position_vector(p4), position_vector(p1));

    SphericalPolygon polygon(std::vector<Arc>{
        make_arc(v1, p1, p2),
        make_arc(v2, p2, p3),
        make_arc(v3, p3, p4),
        make_arc(v4, p4, p1)
    });

    SphericalBoundingBox bbox = polygon.bounding_box();

    EXPECT_TRUE(bbox.is_theta_wrapped());

    double expected_theta_measure = (2.0 * M_PI - 5.8) + 0.2;
    EXPECT_NEAR(bbox.theta_interval().measure(), expected_theta_measure, 0.05);
}

TEST(SphericalPolygonTest, BoundingSphereRadiusWithWrappedTheta) {
    Interval wrapped_theta(5.5, 0.8 + 2.0 * M_PI);
    Interval z_interval(0.0, 0.5);
    SphericalBoundingBox bbox(wrapped_theta, z_interval);

    double radius = bbox.bounding_sphere_radius();

    EXPECT_GT(radius, 0.0);

    double theta_span = bbox.theta_interval().measure();
    double z_span = bbox.z_interval().measure();
    double r_max = std::sqrt(1.0 - z_interval.low() * z_interval.low());
    double chord = 2.0 * r_max * std::sin(theta_span / 2.0);
    double expected_radius = std::sqrt(z_span * z_span + chord * chord);

    EXPECT_NEAR(radius, expected_radius, 1e-9);
}

TEST(SphericalPolygonTest, CentroidReturnsPointOnUnitSphere) {
    SphericalPolygon polygon = SphericalPolygon(
        std::vector<Arc>{
            make_arc(Vector3(0, 0, 1), Point3(1, 0, 0), Point3(0, 1, 0)),
            make_arc(Vector3(1, 0, 0), Point3(0, 1, 0), Point3(0, 0, 1)),
            make_arc(Vector3(0, 1, 0), Point3(0, 0, 1), Point3(1, 0, 0)),
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
            make_arc(0, 0, 1, 1, 0, 0, 0, 1, 0),
            make_arc(0, 0, 1, 0, 1, 0, -1, 0, 0),
            make_arc(0, 0, 1, -1, 0, 0, 0, -1, 0),
            make_arc(0, 0, 1, 0, -1, 0, 1, 0, 0),
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
            make_arc(0, 0, 1, 1, 0, 0, 0.5, sqrt3 / 2, 0),
            make_arc(0, 0, 1, 0.5, sqrt3 / 2, 0, -0.5, sqrt3 / 2, 0),
            make_arc(0, 0, 1, -0.5, sqrt3 / 2, 0, -1, 0, 0),
            make_arc(0, 0, 1, -1, 0, 0, -0.5, -sqrt3 / 2, 0),
            make_arc(0, 0, 1, -0.5, -sqrt3 / 2, 0, 0.5, -sqrt3 / 2, 0),
            make_arc(0, 0, 1, 0.5, -sqrt3 / 2, 0, 1, 0, 0),
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
