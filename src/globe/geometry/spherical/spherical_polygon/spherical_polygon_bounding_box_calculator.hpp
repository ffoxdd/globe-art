#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_BOUNDING_BOX_CALCULATOR_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_BOUNDING_BOX_CALCULATOR_HPP_

#include "../spherical_bounding_box.hpp"
#include "../../../types.hpp"
#include "../helpers.hpp"
#include "../../../math/interval.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <ranges>

namespace globe {

class SphericalPolygonBoundingBoxCalculator {
 public:
    explicit SphericalPolygonBoundingBoxCalculator(const std::vector<Arc>& arcs);

    [[nodiscard]] SphericalBoundingBox calculate() const;

 private:
    const std::vector<Arc>& _arcs;

    [[nodiscard]] ThetaInterval theta_interval() const;
    [[nodiscard]] Interval z_interval() const;
    [[nodiscard]] auto arc_z_intervals() const;
    [[nodiscard]] auto arc_theta_intervals() const;
    [[nodiscard]] bool contains_point(const Point3& point) const;

    [[nodiscard]] static ThetaInterval arc_theta_interval(const Arc& arc);
    [[nodiscard]] static Interval arc_z_interval(const Arc& arc);
    [[nodiscard]] static Interval z_interval_from_endpoints(const Arc& arc);
    [[nodiscard]] static bool is_on_arc(const Arc& arc, const Vector3& position_vector);
    [[nodiscard]] static Vector3 project_to_xy_plane(const Vector3& vector);
    [[nodiscard]] static Vector3 perpendicular_in_xy_plane(const Vector3& vector);
};

inline SphericalPolygonBoundingBoxCalculator::SphericalPolygonBoundingBoxCalculator(
    const std::vector<Arc>& arcs
) : _arcs(arcs) {}

inline auto SphericalPolygonBoundingBoxCalculator::arc_z_intervals() const {
    return _arcs | std::views::transform(
        [](const Arc& arc) { return arc_z_interval(arc); }
    );
}

inline auto SphericalPolygonBoundingBoxCalculator::arc_theta_intervals() const {
    return _arcs | std::views::transform(
        [](const Arc& arc) { return arc_theta_interval(arc); }
    );
}

inline ThetaInterval SphericalPolygonBoundingBoxCalculator::theta_interval() const {
    return ThetaInterval::hull(arc_theta_intervals());
}

inline Interval SphericalPolygonBoundingBoxCalculator::z_interval() const {
    Interval interval = Interval::hull(arc_z_intervals());

    if (contains_point(NORTH_POLE)) {
        interval = hull_interval(interval, NORTH_POLE.z());
    }

    if (contains_point(SOUTH_POLE)) {
        interval = hull_interval(interval, SOUTH_POLE.z());
    }

    return interval;
}

inline SphericalBoundingBox SphericalPolygonBoundingBoxCalculator::calculate() const {
    if (_arcs.empty()) {
        return SphericalBoundingBox::full_sphere();
    }

    return {theta_interval(), z_interval()};
}

inline bool SphericalPolygonBoundingBoxCalculator::contains_point(const Point3& point) const {
    Vector3 p = to_position_vector(point);
    double angle_sum = 0;

    for (const auto& arc : _arcs) {
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

inline Interval SphericalPolygonBoundingBoxCalculator::arc_z_interval(const Arc& arc) {
    Vector3 n = arc_normal(arc);
    Interval z_interval = z_interval_from_endpoints(arc);
    Vector3 n_cross_z = CGAL::cross_product(n, Z_AXIS);

    if (is_zero(n_cross_z)) {
        return z_interval;
    }

    Vector3 critical_dir = CGAL::cross_product(n_cross_z, n);

    if (is_zero(critical_dir)) {
        return z_interval;
    }

    Vector3 critical_point_up = normalize(critical_dir);
    Vector3 critical_point_down = -critical_point_up;

    if (is_on_arc(arc, critical_point_up)) {
        z_interval = hull_interval(z_interval, critical_point_up.z());
    }

    if (is_on_arc(arc, critical_point_down)) {
        z_interval = hull_interval(z_interval, critical_point_down.z());
    }

    return z_interval;
}

inline ThetaInterval SphericalPolygonBoundingBoxCalculator::arc_theta_interval(const Arc& arc) {
    Vector3 n = arc_normal(arc);
    Vector3 source_v = to_position_vector(arc.source());
    Vector3 target_v = to_position_vector(arc.target());
    double theta_source = theta(arc.source());
    double theta_target = theta(arc.target());
    bool is_horizontal = parallel(n, Z_AXIS);

    if (is_horizontal) {
        bool ccw = n.z() > 0;
        if (ccw) {
            return ThetaInterval::from_to(theta_source, theta_target);
        } else {
            return ThetaInterval::from_to(theta_target, theta_source);
        }
    }

    Vector3 n_xy = project_to_xy_plane(n);

    if (is_zero(n_xy)) {
        return ThetaInterval::full();
    }

    Vector3 midpoint_dir = source_v + target_v;
    if (is_zero(midpoint_dir)) {
        return ThetaInterval::full();
    }
    Vector3 midpoint = normalize(midpoint_dir);

    bool short_arc = is_on_arc(arc, midpoint);

    double min_theta = std::min(theta_source, theta_target);
    double max_theta = std::max(theta_source, theta_target);
    bool gap_small = (max_theta - min_theta) <= M_PI;

    ThetaInterval result = [&]() {
        if (short_arc) {
            if (gap_small) {
                return ThetaInterval(min_theta, max_theta - min_theta);
            } else {
                return ThetaInterval::from_to(max_theta, min_theta);
            }
        } else {
            if (gap_small) {
                return ThetaInterval::from_to(max_theta, min_theta);
            } else {
                return ThetaInterval(min_theta, max_theta - min_theta);
            }
        }
    }();

    Vector3 n_xy_normalized = normalize(n_xy);
    Vector3 radial_dir1 = perpendicular_in_xy_plane(n_xy_normalized);
    Vector3 radial_dir2 = -radial_dir1;

    Vector3 critical_dir1 = CGAL::cross_product(n, radial_dir1);
    Vector3 critical_dir2 = CGAL::cross_product(n, radial_dir2);

    if (is_zero(critical_dir1)) {
        return result;
    }

    Vector3 critical_point1 = normalize(critical_dir1);
    Vector3 critical_point2 = normalize(critical_dir2);

    if (is_on_arc(arc, critical_point1)) {
        result = ThetaInterval::hull(result, ThetaInterval(theta(critical_point1), 0));
    }

    if (is_on_arc(arc, critical_point2)) {
        result = ThetaInterval::hull(result, ThetaInterval(theta(critical_point2), 0));
    }

    return result;
}

inline Interval SphericalPolygonBoundingBoxCalculator::z_interval_from_endpoints(const Arc& arc) {
    return hull_interval(arc.source().z(), arc.target().z());
}

inline bool SphericalPolygonBoundingBoxCalculator::is_on_arc(
    const Arc& arc,
    const Vector3& position_vector
) {
    return subarc(arc, position_vector).approximate_angle() < arc.approximate_angle();
}

inline Vector3 SphericalPolygonBoundingBoxCalculator::project_to_xy_plane(const Vector3& vector) {
    return Vector3(vector.x(), vector.y(), 0.0);
}

inline Vector3 SphericalPolygonBoundingBoxCalculator::perpendicular_in_xy_plane(
    const Vector3& vector
) {
    return Vector3(vector.y(), -vector.x(), 0.0);
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_BOUNDING_BOX_CALCULATOR_HPP_
