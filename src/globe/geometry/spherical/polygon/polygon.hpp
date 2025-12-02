#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_POLYGON_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_POLYGON_HPP_

#include "../bounding_box.hpp"
#include "../arc.hpp"
#include "../helpers.hpp"
#include "bounding_box_calculator.hpp"
#include "../../../types.hpp"
#include "../../../std_ext/ranges.hpp"
#include <Eigen/Core>
#include <utility>
#include <vector>
#include <cmath>
#include <ranges>
#include <algorithm>
#include <cassert>

namespace globe::geometry::spherical::polygon {

using globe::VectorS2;
using globe::std_ext::all_circular_adjacent_pairs;
using globe::std_ext::circular_adjacent_pairs;
using globe::geometry::spherical::Arc;
using globe::geometry::spherical::BoundingBox;

class Polygon {
 public:
    explicit Polygon(std::vector<Arc> arcs);

    [[nodiscard]] const std::vector<Arc>& arcs() const { return _arcs; }
    [[nodiscard]] auto points() const;
    [[nodiscard]] BoundingBox bounding_box() const;
    [[nodiscard]] VectorS2 centroid() const;
    [[nodiscard]] double bounding_sphere_radius() const;

    [[nodiscard]] double area() const;
    [[nodiscard]] VectorS2 first_moment() const;
    [[nodiscard]] Eigen::Matrix3d second_moment() const;

    [[nodiscard]] bool contains(const VectorS2& point) const;

 private:
    static constexpr double EPSILON = 1e-10;

    std::vector<Arc> _arcs;

    [[nodiscard]] bool empty() const;
    [[nodiscard]] bool arcs_form_closed_loop() const;
    [[nodiscard]] bool is_convex() const;

    [[nodiscard]] static double spherical_angle(
        const VectorS2& a, const VectorS2& b, const VectorS2& c
    );
    [[nodiscard]] static double spherical_triangle_area(
        const VectorS2& a, const VectorS2& b, const VectorS2& c
    );
};

inline Polygon::Polygon(std::vector<Arc> arcs) :
    _arcs(std::move(arcs)) {

    assert(!empty());
    assert(arcs_form_closed_loop());
}

inline auto Polygon::points() const {
    return _arcs | std::views::transform(
        [](const Arc& arc) { return arc.source(); }
    );
}

inline bool Polygon::empty() const {
    return _arcs.empty();
}

inline bool Polygon::arcs_form_closed_loop() const {
    return all_circular_adjacent_pairs(_arcs, [](const auto& pair) {
        const auto& [prev, next] = pair;
        const VectorS2& prev_target = prev.target();
        const VectorS2& next_source = next.source();
        double dist_sq = (prev_target - next_source).squaredNorm();
        return dist_sq < 1e-10;
    });
}

inline bool Polygon::is_convex() const {
    return true;
}

inline BoundingBox Polygon::bounding_box() const {
    return BoundingBoxCalculator(_arcs).calculate();
}

inline VectorS2 Polygon::centroid() const {
    if (empty()) {
        return bounding_box().center();
    }

    VectorS2 sum = VectorS2::Zero();
    size_t count = 0;

    for (const VectorS2& v : points()) {
        sum += v;
        count++;
    }

    if (count == 0) {
        return bounding_box().center();
    }

    VectorS2 average = sum / static_cast<double>(count);
    double len = average.norm();

    if (len < 1e-15) {
        return bounding_box().center();
    }

    return average / len;
}

inline double Polygon::bounding_sphere_radius() const {
    if (empty()) {
        return 0.0;
    }

    VectorS2 center = centroid();

    double max_squared_distance = 0.0;
    for (const VectorS2& point : points()) {
        double dist_sq = (point - center).squaredNorm();
        max_squared_distance = std::max(max_squared_distance, dist_sq);
    }

    return std::sqrt(max_squared_distance);
}

inline double Polygon::area() const {
    if (_arcs.size() < 3) {
        return 0.0;
    }

    std::vector<double> angles;
    angles.reserve(_arcs.size());
    auto pairs = circular_adjacent_pairs(_arcs);

    for (const auto& pair : pairs) {
        const auto& [prev_arc, curr_arc] = pair;
        angles.push_back(spherical_angle(
            prev_arc.source(),
            curr_arc.source(),
            curr_arc.target()
        ));
    }

    std::sort(angles.begin(), angles.end());

    double angle_sum = 0.0;
    for (double angle : angles) {
        angle_sum += angle;
    }

    return angle_sum - static_cast<double>(_arcs.size() - 2) * M_PI;
}

inline VectorS2 Polygon::first_moment() const {
    if (_arcs.size() < 3) {
        return VectorS2::Zero();
    }

    VectorS2 total = VectorS2::Zero();
    const VectorS2& v0 = _arcs[0].source();

    for (size_t i = 1; i + 1 < _arcs.size(); ++i) {
        const VectorS2& v1 = _arcs[i].source();
        const VectorS2& v2 = _arcs[i + 1].source();

        double tri_area = spherical_triangle_area(v0, v1, v2);

        total += (tri_area / 3.0) * (v0 + v1 + v2);
    }

    return total;
}

inline Eigen::Matrix3d Polygon::second_moment() const {
    if (_arcs.size() < 3) {
        return Eigen::Matrix3d::Zero();
    }

    Eigen::Matrix3d total = Eigen::Matrix3d::Zero();
    const VectorS2& v0 = _arcs[0].source();

    for (size_t i = 1; i + 1 < _arcs.size(); ++i) {
        const VectorS2& v1 = _arcs[i].source();
        const VectorS2& v2 = _arcs[i + 1].source();

        double tri_area = spherical_triangle_area(v0, v1, v2);

        total += (tri_area / 3.0) * (
            v0 * v0.transpose() + v1 * v1.transpose() + v2 * v2.transpose()
        );
    }

    return total;
}

inline bool Polygon::contains(const VectorS2& point) const {
    assert(is_convex());

    for (const auto& arc : _arcs) {
        const VectorS2& arc_normal = arc.normal();
        double sign = arc_normal.dot(point);

        if (sign < -EPSILON) {
            return false;
        }
    }

    return true;
}

inline double Polygon::spherical_angle(
    const VectorS2& a, const VectorS2& b, const VectorS2& c
) {
    VectorS2 ba = b.cross(a).normalized();
    VectorS2 bc = b.cross(c).normalized();
    return distance(ba, bc);
}

inline double Polygon::spherical_triangle_area(
    const VectorS2& a, const VectorS2& b, const VectorS2& c
) {
    VectorS2 va = a.normalized();
    VectorS2 vb = b.normalized();
    VectorS2 vc = c.normalized();

    double numerator = va.dot(vb.cross(vc));
    double denominator = 1.0 + va.dot(vb) + vb.dot(vc) + vc.dot(va);

    if (std::abs(denominator) < 1e-15) {
        return 0.0;
    }

    double tan_half_omega = std::abs(numerator) / denominator;
    return 2.0 * std::atan(tan_half_omega);
}

} // namespace globe::geometry::spherical::polygon

namespace globe::geometry::spherical {
using Polygon = polygon::Polygon;
}

namespace globe {
using Polygon = geometry::spherical::polygon::Polygon;
}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_POLYGON_HPP_
