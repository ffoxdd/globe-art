#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_

#include "spherical_bounding_box.hpp"
#include "../points_collection/types.hpp" // TODO: consider moving spherical_polygon closer to points_collection
#include "../geometry/helpers.hpp"
#include "../noise_generator/interval.hpp" // TODO: move interval out of noise_generator
#include <CGAL/centroid.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <utility>
#include <ranges>
#include <limits>

#include <iostream>

namespace globe {

class SphericalPolygon {
 public:
    explicit SphericalPolygon(std::vector<Arc> arcs);
    auto arcs() -> decltype(auto);
    [[nodiscard]] auto points() const -> decltype(auto);
    [[nodiscard]] SphericalBoundingBox bounding_box() const;
    [[nodiscard]] bool contains(const Point3 &point) const;

 private:
    std::vector<Arc> _arcs;

    static double signed_interior_angle(const Arc &arc, const Point3 &point);
    static double orientation_sign(const Arc &arc, const Point3 &point);
};

auto SphericalPolygon::arcs() -> decltype(auto) { return _arcs; }

auto SphericalPolygon::points() const -> decltype(auto) {
    return _arcs | std::views::transform(
        [](const Arc &arc) { return to_point(arc.source()); }
    );
}

SphericalPolygon::SphericalPolygon(std::vector<Arc> arcs) :
    _arcs(std::move(arcs)) {
}

bool SphericalPolygon::contains(const Point3 &point) const {
    double angle_sum = std::accumulate(
        _arcs.begin(), _arcs.end(), 0.0,
        [point](double angle_sum, const Arc &arc) {
            return angle_sum + SphericalPolygon::signed_interior_angle(arc, point);
        }
    );

    // The sum of the angles should be close to 2π if the point is inside the polygon.
    // We'll use π as the threshold to account for numerical inaccuracies.

    return std::fabs(angle_sum - (2 * CGAL_PI)) < CGAL_PI;
}

double SphericalPolygon::signed_interior_angle(const Arc &arc, const Point3 &point) {
    return interior_angle(arc, point) * orientation_sign(arc, point);
}

double SphericalPolygon::orientation_sign(const Arc &arc, const Point3 &point) {
    auto normal = arc.supporting_circle().supporting_plane().orthogonal_vector();

    return (
        CGAL::scalar_product(
            position_vector(to_point(normal)),
            position_vector(point)
        ) > 0 ? 1 : -1);
}

double theta(double x, double y) { // TODO: put theta(double, double) somewhere generic
    return std::atan2(y, x);
}

SphericalBoundingBox SphericalPolygon::bounding_box() const {
    auto theta_values = points() | std::views::transform(
        [](const Point3 &point) { return theta(point.x(), point.y()); }
    );

    auto z_values = points() | std::views::transform(
        [](const Point3 &point) { return static_cast<double>(point.z()); }
    );

    // return SphericalBoundingBox(theta_values, z_values); // TODO: figure out why this doesn't work
    return {Interval(theta_values), Interval(z_values)};
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_
