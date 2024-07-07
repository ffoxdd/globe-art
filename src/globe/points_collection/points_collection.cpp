#include "points_collection.hpp"
#include <ranges>

namespace globe {

void PointsCollection::insert(Point3 point) {
    _points.push_back(point);
    _kd_tree.insert(point);
    _triangulation.insert(point);
}

bool PointsCollection::empty() const {
    return _points.empty();
}

std::vector<Point3> PointsCollection::nearby_points(Point3 point, double radius) const {
    FuzzySphere search_sphere(point, radius);

    std::vector<Point3> nearby_points;
    _kd_tree.search(std::back_inserter(nearby_points), search_sphere);

    return nearby_points;
}

} // namespace globe
