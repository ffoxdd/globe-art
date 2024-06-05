#ifndef GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
#define GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_

#include "../types.h"
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>

namespace globe {

typedef CGAL::Search_traits_3<Kernel> KDTreeTraits;
typedef CGAL::Kd_tree<KDTreeTraits> KDTree;

class PointsCollection {
 public:
    void insert(Point3 point);
    bool empty();
    std::vector<Point3> nearby_points(Point3 point, double radius);

 private:
    std::vector<Point3> _points;
    KDTree _kd_tree;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
