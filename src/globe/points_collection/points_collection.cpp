#include "points_collection.h"

namespace globe {

typedef CGAL::Fuzzy_sphere<KDTreeTraits> FuzzySphere;

void PointsCollection::insert(Point3 point) {
    _points.push_back(point);
    _kd_tree.insert(point);
}

bool PointsCollection::empty() {
    return _points.empty();
}

std::vector<Point3> PointsCollection::nearby_points(Point3 point, double radius) {
    FuzzySphere search_sphere(point, radius);

    std::vector<Point3> nearby_points;
    _kd_tree.search(std::back_inserter(nearby_points), search_sphere);

    return nearby_points;
}

PointsCollection::iterator PointsCollection::begin() {
    return _points.begin();
}

PointsCollection::const_iterator PointsCollection::begin() const {
    return _points.begin();
}

PointsCollection::iterator PointsCollection::end() {
    return _points.end();
}

PointsCollection::const_iterator PointsCollection::end() const {
    return _points.end();
}

} // namespace globe
