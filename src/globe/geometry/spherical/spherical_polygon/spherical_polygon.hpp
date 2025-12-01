#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_

#include "../spherical_bounding_box.hpp"
#include "../spherical_arc.hpp"
#include "spherical_polygon_bounding_box_calculator.hpp"
#include "../../../types.hpp"
#include "../../../std_ext/ranges.hpp"
#include <Eigen/Core>
#include <utility>
#include <vector>
#include <cmath>
#include <ranges>
#include <algorithm>
#include <cassert>

namespace globe {

class SphericalPolygon {
 public:
    explicit SphericalPolygon(std::vector<SphericalArc> arcs);

    [[nodiscard]] const std::vector<SphericalArc>& arcs() const { return _arcs; }
    [[nodiscard]] auto points() const;
    [[nodiscard]] SphericalBoundingBox bounding_box() const;
    [[nodiscard]] VectorS2 centroid() const;
    [[nodiscard]] double bounding_sphere_radius() const;

    [[nodiscard]] double area() const;
    [[nodiscard]] VectorS2 first_moment() const;
    [[nodiscard]] Eigen::Matrix3d second_moment() const;

    [[nodiscard]] bool contains(const VectorS2& point) const;

 private:
    static constexpr double EPSILON = 1e-10;

    std::vector<SphericalArc> _arcs;

    [[nodiscard]] bool empty() const;
    [[nodiscard]] bool arcs_form_closed_loop() const;
    [[nodiscard]] bool is_convex() const;

    [[nodiscard]] static double spherical_angle(
        const VectorS2& a, const VectorS2& b, const VectorS2& c
    );
    [[nodiscard]] static double spherical_triangle_area(
        const VectorS2& a, const VectorS2& b, const VectorS2& c
    );
    [[nodiscard]] static double angular_distance(const VectorS2& a, const VectorS2& b);
};

inline SphericalPolygon::SphericalPolygon(std::vector<SphericalArc> arcs) :
    _arcs(std::move(arcs)) {

    assert(!empty());
    assert(arcs_form_closed_loop());
}

inline auto SphericalPolygon::points() const {
    return _arcs | std::views::transform(
        [](const SphericalArc& arc) { return arc.source(); }
    );
}

inline bool SphericalPolygon::empty() const {
    return _arcs.empty();
}

inline bool SphericalPolygon::arcs_form_closed_loop() const {
    return all_circular_adjacent_pairs(_arcs, [](const auto& pair) {
        const auto& [prev, next] = pair;
        const VectorS2& prev_target = prev.target();
        const VectorS2& next_source = next.source();
        double dist_sq = (prev_target - next_source).squaredNorm();
        return dist_sq < 1e-10;
    });
}

inline bool SphericalPolygon::is_convex() const {
    return true;
}

inline SphericalBoundingBox SphericalPolygon::bounding_box() const {
    return SphericalPolygonBoundingBoxCalculator(_arcs).calculate();
}

inline VectorS2 SphericalPolygon::centroid() const {
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

inline double SphericalPolygon::bounding_sphere_radius() const {
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

inline double SphericalPolygon::area() const {
    if (_arcs.size() < 3) {
        return 0.0;
    }

    double angle_sum = 0.0;
    auto pairs = circular_adjacent_pairs(_arcs);

    for (const auto& pair : pairs) {
        const auto& [prev_arc, curr_arc] = pair;
        angle_sum += spherical_angle(
            prev_arc.source(),
            curr_arc.source(),
            curr_arc.target()
        );
    }

    return angle_sum - static_cast<double>(_arcs.size() - 2) * M_PI;
}

inline VectorS2 SphericalPolygon::first_moment() const {
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

inline Eigen::Matrix3d SphericalPolygon::second_moment() const {
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

inline bool SphericalPolygon::contains(const VectorS2& point) const {
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

inline double SphericalPolygon::spherical_angle(
    const VectorS2& a, const VectorS2& b, const VectorS2& c
) {
    VectorS2 ba = b.cross(a);
    VectorS2 bc = b.cross(c);
    return angular_distance(ba, bc);
}

inline double SphericalPolygon::spherical_triangle_area(
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

inline double SphericalPolygon::angular_distance(const VectorS2& a, const VectorS2& b) {
    VectorS2 a_norm = a.normalized();
    VectorS2 b_norm = b.normalized();
    double cos_theta = std::clamp(a_norm.dot(b_norm), -1.0, 1.0);
    return std::acos(cos_theta);
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_
