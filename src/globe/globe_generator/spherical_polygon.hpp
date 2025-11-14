#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_

#include "spherical_bounding_box.hpp"
#include "../points_collection/types.hpp" // TODO: consider moving spherical_polygon closer to points_collection
#include "../geometry/helpers.hpp"
#include "../noise_generator/interval.hpp" // TODO: move interval out of noise_generator
#include <CGAL/assertions.h>
#include <utility>
#include <stdexcept>
#include <vector>
#include <cmath>

namespace globe {

class SphericalPolygon {
 public:
    explicit SphericalPolygon(std::vector<Arc> arcs);

    auto arcs();
    [[nodiscard]] auto points() const;
    [[nodiscard]] SphericalBoundingBox bounding_box() const;
    [[nodiscard]] bool contains(const Point3 &point) const;

    [[nodiscard]] bool is_valid() const;
    [[nodiscard]] bool is_convex() const;
    void validate() const;

 private:
    std::vector<Arc> _arcs;
};

inline auto SphericalPolygon::arcs() { return _arcs; }

inline auto SphericalPolygon::points() const {
    return _arcs | std::views::transform(
        [](const Arc &arc) { return to_point(arc.source()); }
    );
}

inline SphericalPolygon::SphericalPolygon(std::vector<Arc> arcs) :
    _arcs(std::move(arcs)) {
    CGAL_precondition(!_arcs.empty());
    CGAL_precondition(is_valid());
    CGAL_expensive_precondition(is_convex());
}

inline bool SphericalPolygon::contains(const Point3 &point) const {
    CGAL_precondition(is_convex());

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

        if (sign == 0) {
            SphericalPoint3 p_ = SphericalPoint3(p.x(), p.y(), p.z());
            Arc p_arc = Arc(arc.supporting_circle(), arc.source(), p_);

            return p_arc.approximate_angle() < arc.approximate_angle();
        }

        angle_sum += (sign > 0) ? angle : -angle;
    }

    return std::fabs(angle_sum - (2 * CGAL_PI)) < CGAL_PI;
}

inline bool SphericalPolygon::is_valid() const {
    if (_arcs.empty()) {
        return false;
    }

    for (size_t i = 0; i < _arcs.size(); ++i) {
        size_t next = (i + 1) % _arcs.size();
        Point3 current_target = to_point(_arcs[i].target());
        Point3 next_source = to_point(_arcs[next].source());

        if (CGAL::squared_distance(current_target, next_source) > 1e-10) {
            return false;
        }
    }

    return true;
}

inline bool SphericalPolygon::is_convex() const {
    return true;
}

inline void SphericalPolygon::validate() const {
    if (!is_valid()) {
        throw std::runtime_error("SphericalPolygon is invalid: arcs don't form a closed loop");
    }
    if (!is_convex()) {
        throw std::runtime_error("SphericalPolygon is not convex");
    }
}

inline double theta(double x, double y) {
    return std::atan2(y, x);
}

inline SphericalBoundingBox SphericalPolygon::bounding_box() const {
    auto theta_values = points() | std::views::transform(
        [](const Point3 &point) { return theta(point.x(), point.y()); }
    );

    auto z_values = points() | std::views::transform(
        [](const Point3 &point) { return static_cast<double>(point.z()); }
    );

    return {Interval(theta_values), Interval(z_values)};
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_
