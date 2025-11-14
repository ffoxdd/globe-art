#include "gtest/gtest.h"
#include "./spherical_polygon.hpp"
#include <cmath>

using namespace globe;

TEST(SphericalPolygonTest, SimplePolygon) {
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
               SphericalPoint3(1, 0, 0), SphericalPoint3(0, 1, 0)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0)),
               SphericalPoint3(0, 1, 0), SphericalPoint3(0, 0, 1)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 1, 0)),
               SphericalPoint3(0, 0, 1), SphericalPoint3(1, 0, 0)
           ),
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
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, -1, 0)),
               SphericalPoint3(1, 0, 0), SphericalPoint3(0, 0, 1)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(-1, 0, 0)),
               SphericalPoint3(0, 0, 1), SphericalPoint3(0, 1, 0)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, -1)),
               SphericalPoint3(0, 1, 0), SphericalPoint3(1, 0, 0)
           ),
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
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
               SphericalPoint3(1, 0, 0), SphericalPoint3(0, 1, 0)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
               SphericalPoint3(0, 1, 0), SphericalPoint3(-1, 0, 0)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
               SphericalPoint3(-1, 0, 0), SphericalPoint3(0, -1, 0)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
               SphericalPoint3(0, -1, 0), SphericalPoint3(1, 0, 0)
           ),
       }
   );


   EXPECT_TRUE(spherical_polygon.contains(Point3(0, 0, 1)));
   EXPECT_FALSE(spherical_polygon.contains(Point3(0, 0, -1)));
}

TEST(SphericalPolygonTest, PathologicalHemisphere) {
   SphericalPolygon spherical_polygon = SphericalPolygon(
       std::vector<Arc>{
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
               SphericalPoint3(1, 0, 0), SphericalPoint3(-1, 0, 0)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
               SphericalPoint3(-1, 0, 0), SphericalPoint3(1, 0, 0)
           ),
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
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 0, 1)),
               SphericalPoint3(1, 0, 0), SphericalPoint3(0, 1, 0)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0)),
               SphericalPoint3(0, 1, 0), SphericalPoint3(0, 0, 1)
           ),
           Arc(
               SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(0, 1, 0)),
               SphericalPoint3(0, 0, 1), SphericalPoint3(1, 0, 0)
           ),
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
