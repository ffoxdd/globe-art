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
    double angle_sum = 0;

    for (const auto &arc : _arcs) {
        Vector3 a = position_vector(arc.source());
        Vector3 b = position_vector(arc.target());
        Vector3 p = position_vector(point);

        Vector3 ap = CGAL::cross_product(a, p);
        Vector3 bp = CGAL::cross_product(b, p);

        double angle = angular_distance(ap, bp);

        auto normal = arc.supporting_circle().supporting_plane().orthogonal_vector();
        double sign = CGAL::scalar_product(position_vector(normal), p);

        if (sign == 0) { // the point is on the arc's circumcircle
            SphericalPoint3 p_ = SphericalPoint3(p.x(), p.y(), p.z());
            Arc p_arc = Arc(arc.supporting_circle(), arc.source(), p_);

            std::cout << "arc normal: " << normal << std::endl;
            std::cout << "arc source: " << arc.source() << std::endl;
            std::cout << "arc target: " << arc.target() << std::endl;

            std::cout << "arc angle: " << arc.approximate_angle() << std::endl;
            std::cout << "p_arc angle: " << p_arc.approximate_angle() << std::endl;

            return p_arc.approximate_angle() < arc.approximate_angle();
        }

        std::cout << "sign: " << sign << std::endl;

        angle_sum += (sign > 0) ? angle : -angle;
    }

    return std::fabs(angle_sum - (2 * CGAL_PI)) < CGAL_PI;
}

double SphericalPolygon::signed_interior_angle(const Arc &arc, const Point3 &point) {
    Vector3 a = position_vector(arc.source());
    Vector3 b = position_vector(arc.target());
    Vector3 p = position_vector(point);

    Vector3 ap = CGAL::cross_product(a, p);
    Vector3 bp = CGAL::cross_product(b, p);

    double angle = angular_distance(ap, bp);

    auto normal = arc.supporting_circle().supporting_plane().orthogonal_vector();
    double sign = CGAL::scalar_product(position_vector(normal), p);

    if (sign == 0) {
        sign = angular_distance(a, p) < angular_distance(a, b) ? 1 : -1;
        // WE NEED TO MAKE THIS SHORT CIRCUIT TO TRUE OR FALSE
    }

    std::cout << "sign: " << sign << std::endl;

    return (sign > 0) ? angle : -angle;
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
