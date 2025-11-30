#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_

#include "../spherical_bounding_box.hpp"
#include "spherical_polygon_bounding_box_calculator.hpp"
#include "../../../types.hpp"
#include "../helpers.hpp"
#include "../../../std_ext/ranges.hpp"
#include <CGAL/assertions.h>
#include <Eigen/Core>
#include <utility>
#include <vector>
#include <cmath>
#include <ranges>
#include <algorithm>

namespace globe {

struct PolygonMoments {
    double area;
    Eigen::Vector3d first_moment;
    Eigen::Matrix3d second_moment;
};

class SphericalPolygon {
 public:
    explicit SphericalPolygon(std::vector<Arc> arcs);

    [[nodiscard]] auto points() const;
    [[nodiscard]] auto arc_source_points() const;
    [[nodiscard]] SphericalBoundingBox bounding_box() const;
    [[nodiscard]] Point3 centroid() const;
    [[nodiscard]] double bounding_sphere_radius() const;
    [[nodiscard]] double area() const;
    [[nodiscard]] PolygonMoments moments() const;
    [[nodiscard]] bool contains(const Point3 &point) const;

 private:
    std::vector<Arc> _arcs;

    [[nodiscard]] bool empty() const;
    [[nodiscard]] bool arcs_form_closed_loop() const;
    [[nodiscard]] static bool is_on_arc(const Arc &arc, const Vector3 &position_vector);
    [[nodiscard]] bool is_convex() const;
    [[nodiscard]] auto position_vectors() const;
};

inline SphericalPolygon::SphericalPolygon(std::vector<Arc> arcs) :
    _arcs(std::move(arcs)) {

    CGAL_precondition(!empty());
    CGAL_precondition(arcs_form_closed_loop());
}

inline auto SphericalPolygon::points() const {
    return _arcs | std::views::transform(
        [](const Arc &arc) { return to_point(arc.source()); }
    );
}

inline auto SphericalPolygon::arc_source_points() const {
    return _arcs | std::views::transform(
        [](const Arc &arc) { return arc.source(); }
    );
}

inline bool SphericalPolygon::empty() const {
    return _arcs.empty();
}

inline bool SphericalPolygon::arcs_form_closed_loop() const {
    return all_circular_adjacent_pairs(_arcs, [](const auto &pair) {
        const auto &[prev, next] = pair;
        return prev.target() == next.source();
    });
}

inline bool SphericalPolygon::is_on_arc(const Arc &arc, const Vector3 &position_vector) {
    return subarc(arc, position_vector).approximate_angle() < arc.approximate_angle();
}

inline bool SphericalPolygon::is_convex() const {
    return true; // TODO: implement
}

inline auto SphericalPolygon::position_vectors() const {
    return points() | std::views::transform(
        [](const Point3 &point) { return to_position_vector(point); }
    );
}

inline SphericalBoundingBox SphericalPolygon::bounding_box() const {
    return SphericalPolygonBoundingBoxCalculator(_arcs).calculate();
}

inline Point3 SphericalPolygon::centroid() const {
    if (empty()) {
        return bounding_box().center();
    }

    Vector3 average = average_vector(position_vectors());

    if (is_zero(average)) {
        return bounding_box().center();
    }

    return project_to_sphere(average);
}

inline double SphericalPolygon::bounding_sphere_radius() const {
    if (empty()) {
        return 0.0;
    }

    Point3 center = centroid();

    double max_squared_distance = std::ranges::max(
        points() | std::views::transform(
            [&center](const Point3 &point) {
                return CGAL::squared_distance(point, center);
            }
        )
    );

    return std::sqrt(max_squared_distance);
}

inline double SphericalPolygon::area() const {
    if (_arcs.size() < 3) {
        return 0.0;
    }

    auto interior_angles = circular_adjacent_pairs(_arcs) |
        std::views::transform([](const auto &pair) {
            const auto &[prev_arc, curr_arc] = pair;
            return spherical_angle(
                prev_arc.source(),
                curr_arc.source(),
                curr_arc.target()
            );
        });

    return sum(interior_angles) - static_cast<double>(_arcs.size() - 2) * M_PI;
}

inline PolygonMoments SphericalPolygon::moments() const {
    if (_arcs.size() < 3) {
        return {0.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
    }

    double total_area = 0.0;
    Eigen::Vector3d total_first_moment = Eigen::Vector3d::Zero();
    Eigen::Matrix3d total_second_moment = Eigen::Matrix3d::Zero();

    Eigen::Vector3d v0 = to_eigen(_arcs[0].source());

    for (size_t i = 1; i + 1 < _arcs.size(); ++i) {
        Eigen::Vector3d v1 = to_eigen(_arcs[i].source());
        Eigen::Vector3d v2 = to_eigen(_arcs[i + 1].source());

        double tri_area = spherical_triangle_area(
            _arcs[0].source(), _arcs[i].source(), _arcs[i + 1].source()
        );

        total_area += tri_area;
        total_first_moment += (tri_area / 3.0) * (v0 + v1 + v2);
        total_second_moment += (tri_area / 3.0) * (
            v0 * v0.transpose() + v1 * v1.transpose() + v2 * v2.transpose()
        );
    }

    return {total_area, total_first_moment, total_second_moment};
}

inline bool SphericalPolygon::contains(const Point3 &point) const {
    CGAL_precondition(is_convex());

    Vector3 p = to_position_vector(point);
    double angle_sum = 0;

    for (const auto &arc : _arcs) {
        Vector3 a = to_position_vector(arc.source());
        Vector3 b = to_position_vector(arc.target());

        Vector3 ap = CGAL::cross_product(a, p);
        Vector3 bp = CGAL::cross_product(b, p);

        double angle = angular_distance(ap, bp);
        Vector3 normal = arc_normal(arc);

        double sign = CGAL::scalar_product(normal, p);

        if (is_zero(sign)) {
            return is_on_arc(arc, p);
        }

        angle_sum += (sign > 0) ? angle : -angle;
    }

    return std::fabs(angle_sum - (2 * CGAL_PI)) < CGAL_PI;
}


} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_
