#include "gtest/gtest.h"
#include "polygon.hpp"
#include "../arc.hpp"
#include <cmath>

using namespace globe;

TEST(PolygonTest, SimplePolygon) {
    Polygon spherical_polygon = Polygon(
        std::vector<Arc>{
            Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
            Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
        }
    );

    const double sqrt3 = std::sqrt(3);

    EXPECT_TRUE(spherical_polygon.contains(VectorS2(1 / sqrt3, 1 / sqrt3, 1 / sqrt3)));
    EXPECT_FALSE(spherical_polygon.contains(VectorS2(-1, 0, 0)));
    EXPECT_FALSE(spherical_polygon.contains(VectorS2(-1 / sqrt3, -1 / sqrt3, -1 / sqrt3)));
}

TEST(PolygonTest, InsideOutPolygon) {
    Polygon spherical_polygon = Polygon(
        std::vector<Arc>{
            Arc(VectorS2(1, 0, 0), VectorS2(0, 0, 1), VectorS2(0, -1, 0)),
            Arc(VectorS2(0, 0, 1), VectorS2(0, 1, 0), VectorS2(-1, 0, 0)),
            Arc(VectorS2(0, 1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, -1)),
        }
    );

    const double sqrt3 = std::sqrt(3);

    EXPECT_FALSE(spherical_polygon.contains(VectorS2(1 / sqrt3, 1 / sqrt3, 1 / sqrt3)));
    EXPECT_TRUE(spherical_polygon.contains(VectorS2(-1, 0, 0)));
    EXPECT_TRUE(spherical_polygon.contains(VectorS2(-1 / sqrt3, -1 / sqrt3, -1 / sqrt3)));
}

TEST(PolygonTest, Hemisphere) {
    Polygon spherical_polygon = Polygon(
        std::vector<Arc>{
            Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
        }
    );


    EXPECT_TRUE(spherical_polygon.contains(VectorS2(0, 0, 1)));
    EXPECT_FALSE(spherical_polygon.contains(VectorS2(0, 0, -1)));
}

TEST(PolygonTest, PathologicalHemisphere) {
    Polygon spherical_polygon = Polygon(
        std::vector<Arc>{
            Arc(VectorS2(1, 0, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(-1, 0, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
        }
    );

    EXPECT_TRUE(spherical_polygon.contains(VectorS2(0, 0, 1)));
    EXPECT_FALSE(spherical_polygon.contains(VectorS2(0, 0, -1)));
}

TEST(PolygonTest, PointOnArcCircumcircle) {
    Polygon spherical_polygon = Polygon(
        std::vector<Arc>{
            Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
            Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
        }
    );

    const double sqrt2 = std::sqrt(2);
    VectorS2 point_on_arc(sqrt2 / 2.0, sqrt2 / 2.0, 0);
    EXPECT_TRUE(spherical_polygon.contains(point_on_arc));

    VectorS2 point_on_circle_outside_arc(-sqrt2 / 2.0, sqrt2 / 2.0, 0);
    EXPECT_FALSE(spherical_polygon.contains(point_on_circle_outside_arc));
}

TEST(PolygonTest, PolygonWithWrappedThetaBoundingBox) {
    double theta1 = 5.8;
    double theta2 = 0.4;
    double z_low = 0.0;
    double z_high = 0.4;
    double r_low = std::sqrt(1.0 - z_low * z_low);
    double r_high = std::sqrt(1.0 - z_high * z_high);

    VectorS2 p1(r_low * std::cos(theta1), r_low * std::sin(theta1), z_low);
    VectorS2 p2(r_low * std::cos(theta2), r_low * std::sin(theta2), z_low);
    VectorS2 p3(r_high * std::cos(theta2), r_high * std::sin(theta2), z_high);
    VectorS2 p4(r_high * std::cos(theta1), r_high * std::sin(theta1), z_high);

    VectorS2 n1 = p1.cross(p2).normalized();
    VectorS2 n2 = p2.cross(p3).normalized();
    VectorS2 n3 = p3.cross(p4).normalized();
    VectorS2 n4 = p4.cross(p1).normalized();

    Polygon polygon(std::vector<Arc>{
        Arc(p1, p2, n1),
        Arc(p2, p3, n2),
        Arc(p3, p4, n3),
        Arc(p4, p1, n4)
    });

    SphericalBoundingBox bounding_box = polygon.bounding_box();

    EXPECT_GT(bounding_box.theta_interval().end(), TWO_PI);

    VectorS2 center = bounding_box.center();

    double center_theta = std::atan2(center.y(), center.x());
    if (center_theta < 0.0) center_theta += 2.0 * M_PI;

    EXPECT_TRUE(bounding_box.theta_interval().contains(center_theta));
}

TEST(PolygonTest, BoundingBoxWrappedThetaMeasure) {
    double theta1 = 5.8;
    double theta2 = 0.2;
    double z_low = 0.0;
    double z_high = 0.3;
    double r_low = std::sqrt(1.0 - z_low * z_low);
    double r_high = std::sqrt(1.0 - z_high * z_high);

    VectorS2 p1(r_low * std::cos(theta1), r_low * std::sin(theta1), z_low);
    VectorS2 p2(r_low * std::cos(theta2), r_low * std::sin(theta2), z_low);
    VectorS2 p3(r_high * std::cos(theta2), r_high * std::sin(theta2), z_high);
    VectorS2 p4(r_high * std::cos(theta1), r_high * std::sin(theta1), z_high);

    VectorS2 n1 = p1.cross(p2).normalized();
    VectorS2 n2 = p2.cross(p3).normalized();
    VectorS2 n3 = p3.cross(p4).normalized();
    VectorS2 n4 = p4.cross(p1).normalized();

    Polygon polygon(std::vector<Arc>{
        Arc(p1, p2, n1),
        Arc(p2, p3, n2),
        Arc(p3, p4, n3),
        Arc(p4, p1, n4)
    });

    SphericalBoundingBox bounding_box = polygon.bounding_box();

    EXPECT_GT(bounding_box.theta_interval().end(), TWO_PI);

    double expected_theta_measure = (2.0 * M_PI - 5.8) + 0.2;
    EXPECT_NEAR(bounding_box.theta_interval().measure(), expected_theta_measure, 0.05);
}

TEST(PolygonTest, BoundingSphereRadiusWithWrappedTheta) {
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

TEST(PolygonTest, CentroidReturnsPointOnUnitSphere) {
    Polygon polygon = Polygon(
        std::vector<Arc>{
            Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
            Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
        }
    );

    VectorS2 centroid = polygon.centroid();

    EXPECT_NEAR(centroid.norm(), 1.0, 1e-9);
}

TEST(PolygonTest, CentroidIsOnUnitSphere) {
    Polygon polygon = Polygon(
        std::vector<Arc>{
            Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
        }
    );

    VectorS2 centroid = polygon.centroid();

    EXPECT_NEAR(centroid.norm(), 1.0, 1e-9);
}

TEST(PolygonTest, CentroidForSymmetricPolygon) {
    const double sqrt3 = std::sqrt(3);
    Polygon polygon = Polygon(
        std::vector<Arc>{
            Arc(VectorS2(1, 0, 0), VectorS2(0.5, sqrt3 / 2, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0.5, sqrt3 / 2, 0), VectorS2(-0.5, sqrt3 / 2, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(-0.5, sqrt3 / 2, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(-1, 0, 0), VectorS2(-0.5, -sqrt3 / 2, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(-0.5, -sqrt3 / 2, 0), VectorS2(0.5, -sqrt3 / 2, 0), VectorS2(0, 0, 1)),
            Arc(VectorS2(0.5, -sqrt3 / 2, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
        }
    );

    VectorS2 centroid = polygon.centroid();

    EXPECT_NEAR(centroid.norm(), 1.0, 1e-9);
    EXPECT_NEAR(centroid.z(), 0.5, 1e-9);
}

TEST(PolygonTest, ArcThetaExtrema_FullCircle) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1))
    });

    ThetaInterval theta_interval = polygon.bounding_box().theta_interval();

    EXPECT_TRUE(theta_interval.is_full());
}

TEST(PolygonTest, BoundingBox_IncludesNorthPoleZ) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1))
    });

    EXPECT_TRUE(polygon.contains(VectorS2(0, 0, 1)));

    SphericalBoundingBox bounding_box = polygon.bounding_box();
    Interval z_interval = bounding_box.z_interval();

    EXPECT_LE(z_interval.low(), 0.0);
    EXPECT_GE(z_interval.high(), 1.0);
    EXPECT_TRUE(z_interval.contains(1.0));
}

TEST(PolygonTest, BoundingBox_IncludesSouthPoleZ) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, -1)),
        Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, -1)),
        Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, -1)),
        Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, -1))
    });

    EXPECT_TRUE(polygon.contains(VectorS2(0, 0, -1)));

    SphericalBoundingBox bounding_box = polygon.bounding_box();
    Interval z_interval = bounding_box.z_interval();

    EXPECT_GE(z_interval.high(), 0.0);
    EXPECT_LE(z_interval.low(), -1.0);
    EXPECT_TRUE(z_interval.contains(-1.0));
}

TEST(PolygonTest, BoundingBox_PoleZEvenWhenArcsDontReach) {
    double z_arc = 0.9;
    double r_arc = std::sqrt(1.0 - z_arc * z_arc);

    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(r_arc, 0, z_arc), VectorS2(0, r_arc, z_arc), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, r_arc, z_arc), VectorS2(-r_arc, 0, z_arc), VectorS2(0, 0, 1)),
        Arc(VectorS2(-r_arc, 0, z_arc), VectorS2(0, -r_arc, z_arc), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, -r_arc, z_arc), VectorS2(r_arc, 0, z_arc), VectorS2(0, 0, 1))
    });

    EXPECT_TRUE(polygon.contains(VectorS2(0, 0, 1)));

    SphericalBoundingBox bounding_box = polygon.bounding_box();
    Interval z_interval = bounding_box.z_interval();

    EXPECT_LE(z_interval.low(), z_arc);
    EXPECT_GE(z_interval.high(), 1.0);
    EXPECT_TRUE(z_interval.contains(1.0));
}

TEST(PolygonTest, BoundingSphereRadiusContainsAllVertices) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
        Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
    });

    VectorS2 centroid = polygon.centroid();
    double radius = polygon.bounding_sphere_radius();

    for (const auto& point : polygon.points()) {
        double distance = (point - centroid).norm();
        EXPECT_LE(distance, radius + GEOMETRIC_EPSILON);
    }
}

TEST(PolygonTest, BoundingSphereRadiusIsMinimalForSymmetricPolygon) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
    });

    double radius = polygon.bounding_sphere_radius();

    double max_distance = 0.0;
    VectorS2 centroid = polygon.centroid();
    for (const auto& point : polygon.points()) {
        double distance = (point - centroid).norm();
        max_distance = std::max(max_distance, distance);
    }

    EXPECT_NEAR(radius, max_distance, GEOMETRIC_EPSILON);
}

TEST(PolygonTest, AreaOfOctant) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
        Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
    });

    double expected_area = M_PI / 2.0;
    EXPECT_NEAR(polygon.area(), expected_area, 1e-9);
}

TEST(PolygonTest, AreaOfHemisphere) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(-1, 0, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(-1, 0, 0), VectorS2(0, -1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, -1, 0), VectorS2(1, 0, 0), VectorS2(0, 0, 1)),
    });

    double expected_area = 2.0 * M_PI;
    EXPECT_NEAR(polygon.area(), expected_area, 1e-9);
}

TEST(PolygonTest, MomentsArea) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
        Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
    });

    double expected_area = M_PI / 2.0;
    EXPECT_NEAR(polygon.area(), expected_area, 1e-6);
}

TEST(PolygonTest, MomentsFirstMomentDirection) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
        Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
    });

    Eigen::Vector3d first_moment = polygon.first_moment();

    EXPECT_GT(first_moment.x(), 0);
    EXPECT_GT(first_moment.y(), 0);
    EXPECT_GT(first_moment.z(), 0);

    EXPECT_NEAR(first_moment.x(), first_moment.y(), 1e-6);
    EXPECT_NEAR(first_moment.y(), first_moment.z(), 1e-6);
}

TEST(PolygonTest, MomentsSecondMomentSymmetric) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
        Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
    });

    Eigen::Matrix3d second_moment = polygon.second_moment();

    EXPECT_NEAR(second_moment(0, 1), second_moment(1, 0), 1e-10);
    EXPECT_NEAR(second_moment(0, 2), second_moment(2, 0), 1e-10);
    EXPECT_NEAR(second_moment(1, 2), second_moment(2, 1), 1e-10);
}

TEST(PolygonTest, MomentsSecondMomentTrace) {
    Polygon polygon(std::vector<Arc>{
        Arc(VectorS2(1, 0, 0), VectorS2(0, 1, 0), VectorS2(0, 0, 1)),
        Arc(VectorS2(0, 1, 0), VectorS2(0, 0, 1), VectorS2(1, 0, 0)),
        Arc(VectorS2(0, 0, 1), VectorS2(1, 0, 0), VectorS2(0, 1, 0)),
    });

    EXPECT_NEAR(polygon.second_moment().trace(), polygon.area(), 1e-6);
}
