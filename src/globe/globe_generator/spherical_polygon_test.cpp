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
