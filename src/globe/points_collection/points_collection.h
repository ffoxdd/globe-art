#ifndef GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
#define GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_

#include "../types.h"
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include <vector>
#include <ranges>

namespace globe {

typedef CGAL::Search_traits_3<Kernel> KDTreeTraits;
typedef CGAL::Kd_tree<KDTreeTraits> KDTree;

typedef CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel> Traits;
typedef CGAL::Delaunay_triangulation_on_sphere_2<Traits> Triangulation;

class PointsCollection {
 public:
    void insert(Point3 point);
    bool empty();
    std::vector<Point3> nearby_points(Point3 point, double radius);

    auto points() {
        return std::ranges::subrange(_points.begin(), _points.end());
    };

 private:
    std::vector<Point3> _points;
    KDTree _kd_tree;
    Triangulation _triangulation;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
