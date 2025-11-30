#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_

#include "../spherical_bounding_box.hpp"
#include "../spherical_arc.hpp"
#include "spherical_polygon_bounding_box_calculator.hpp"
#include "../../../types.hpp"
#include "../../../std_ext/ranges.hpp"
#include <CGAL/Kernel/global_functions.h>
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
    [[nodiscard]] Point3 centroid() const;
    [[nodiscard]] double bounding_sphere_radius() const;

    [[nodiscard]] double area() const;
    [[nodiscard]] Eigen::Vector3d first_moment() const;
    [[nodiscard]] Eigen::Matrix3d second_moment() const;

    [[nodiscard]] bool contains(const Point3& point) const;

 private:
    std::vector<SphericalArc> _arcs;

    [[nodiscard]] bool empty() const;
    [[nodiscard]] bool arcs_form_closed_loop() const;
    [[nodiscard]] bool is_convex() const;
    [[nodiscard]] auto position_vectors() const;

    static Vector3 to_position_vector(const Point3& p);
    static Eigen::Vector3d to_eigen(const Point3& p);
    static double spherical_angle(const Point3& a, const Point3& b, const Point3& c);
    static double spherical_triangle_area(const Point3& a, const Point3& b, const Point3& c);
    static Vector3 normalize(const Vector3& v);
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
        const Point3& prev_target = prev.target();
        const Point3& next_source = next.source();
        double dist_sq = CGAL::squared_distance(prev_target, next_source);
        return dist_sq < 1e-10;
    });
}

inline bool SphericalPolygon::is_convex() const {
    return true;
}

inline auto SphericalPolygon::position_vectors() const {
    return points() | std::views::transform(
        [](const Point3& point) { return to_position_vector(point); }
    );
}

inline SphericalBoundingBox SphericalPolygon::bounding_box() const {
    return SphericalPolygonBoundingBoxCalculator(_arcs).calculate();
}

inline Point3 SphericalPolygon::centroid() const {
    if (empty()) {
        return bounding_box().center();
    }

    Vector3 sum(0, 0, 0);
    size_t count = 0;

    for (const auto& v : position_vectors()) {
        sum = sum + v;
        count++;
    }

    if (count == 0) {
        return bounding_box().center();
    }

    Vector3 average = sum / static_cast<double>(count);
    double len = std::sqrt(average.squared_length());

    if (len < 1e-15) {
        return bounding_box().center();
    }

    Vector3 normalized = average / len;
    return Point3(normalized.x(), normalized.y(), normalized.z());
}

inline double SphericalPolygon::bounding_sphere_radius() const {
    if (empty()) {
        return 0.0;
    }

    Point3 center = centroid();

    double max_squared_distance = 0.0;
    for (const Point3& point : points()) {
        double dist_sq = CGAL::squared_distance(point, center);
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

inline Eigen::Vector3d SphericalPolygon::first_moment() const {
    if (_arcs.size() < 3) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d total = Eigen::Vector3d::Zero();
    Eigen::Vector3d v0 = to_eigen(_arcs[0].source());

    for (size_t i = 1; i + 1 < _arcs.size(); ++i) {
        Eigen::Vector3d v1 = to_eigen(_arcs[i].source());
        Eigen::Vector3d v2 = to_eigen(_arcs[i + 1].source());

        double tri_area = spherical_triangle_area(
            _arcs[0].source(), _arcs[i].source(), _arcs[i + 1].source()
        );

        total += (tri_area / 3.0) * (v0 + v1 + v2);
    }

    return total;
}

inline Eigen::Matrix3d SphericalPolygon::second_moment() const {
    if (_arcs.size() < 3) {
        return Eigen::Matrix3d::Zero();
    }

    Eigen::Matrix3d total = Eigen::Matrix3d::Zero();
    Eigen::Vector3d v0 = to_eigen(_arcs[0].source());

    for (size_t i = 1; i + 1 < _arcs.size(); ++i) {
        Eigen::Vector3d v1 = to_eigen(_arcs[i].source());
        Eigen::Vector3d v2 = to_eigen(_arcs[i + 1].source());

        double tri_area = spherical_triangle_area(
            _arcs[0].source(), _arcs[i].source(), _arcs[i + 1].source()
        );

        total += (tri_area / 3.0) * (
            v0 * v0.transpose() + v1 * v1.transpose() + v2 * v2.transpose()
        );
    }

    return total;
}

inline bool SphericalPolygon::contains(const Point3& point) const {
    assert(is_convex());

    Vector3 p = to_position_vector(point);

    for (const auto& arc : _arcs) {
        Vector3 arc_normal = arc.normal();
        double sign = CGAL::scalar_product(arc_normal, p);

        if (sign < -1e-10) {
            return false;
        }
    }

    return true;
}

inline Vector3 SphericalPolygon::to_position_vector(const Point3& p) {
    return Vector3(p.x(), p.y(), p.z());
}

inline Eigen::Vector3d SphericalPolygon::to_eigen(const Point3& p) {
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}

inline double SphericalPolygon::spherical_angle(const Point3& a, const Point3& b, const Point3& c) {
    Vector3 va = to_position_vector(a);
    Vector3 vb = to_position_vector(b);
    Vector3 vc = to_position_vector(c);

    Vector3 ba = CGAL::cross_product(vb, va);
    Vector3 bc = CGAL::cross_product(vb, vc);

    double ba_len = std::sqrt(ba.squared_length());
    double bc_len = std::sqrt(bc.squared_length());

    if (ba_len < 1e-10 || bc_len < 1e-10) {
        return M_PI;
    }

    double dot = (ba.x() * bc.x() + ba.y() * bc.y() + ba.z() * bc.z()) / (ba_len * bc_len);
    dot = std::clamp(dot, -1.0, 1.0);

    return std::acos(dot);
}

inline double SphericalPolygon::spherical_triangle_area(const Point3& a, const Point3& b, const Point3& c) {
    Vector3 va = normalize(to_position_vector(a));
    Vector3 vb = normalize(to_position_vector(b));
    Vector3 vc = normalize(to_position_vector(c));

    Vector3 cross = CGAL::cross_product(vb, vc);
    double numerator = CGAL::scalar_product(va, cross);
    double denominator = 1.0 +
        CGAL::scalar_product(va, vb) +
        CGAL::scalar_product(vb, vc) +
        CGAL::scalar_product(vc, va);

    if (std::abs(denominator) < 1e-15) {
        return 0.0;
    }

    double tan_half_omega = std::abs(numerator) / denominator;
    return 2.0 * std::atan(tan_half_omega);
}

inline Vector3 SphericalPolygon::normalize(const Vector3& v) {
    double len = std::sqrt(v.squared_length());
    if (len < 1e-15) {
        return Vector3(0, 0, 1);
    }
    return v / len;
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_
