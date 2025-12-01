#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_BOUNDING_BOX_CALCULATOR_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_BOUNDING_BOX_CALCULATOR_HPP_

#include "../spherical_bounding_box.hpp"
#include "../spherical_arc.hpp"
#include "../helpers.hpp"
#include "../../../types.hpp"
#include "../../../math/interval.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <ranges>

namespace globe {

class SphericalPolygonBoundingBoxCalculator {
 public:
    explicit SphericalPolygonBoundingBoxCalculator(const std::vector<SphericalArc>& arcs);

    [[nodiscard]] SphericalBoundingBox calculate() const;

 private:
    static constexpr double EPSILON = 1e-10;

    const std::vector<SphericalArc>& _arcs;

    [[nodiscard]] ThetaInterval theta_interval() const;
    [[nodiscard]] Interval z_interval() const;
    [[nodiscard]] auto arc_z_intervals() const;
    [[nodiscard]] auto arc_theta_intervals() const;
    [[nodiscard]] bool contains_point(const VectorS2& point) const;

    [[nodiscard]] static ThetaInterval arc_theta_interval(const SphericalArc& arc);
    [[nodiscard]] static Interval arc_z_interval(const SphericalArc& arc);
    [[nodiscard]] static Interval z_interval_from_endpoints(const SphericalArc& arc);
    [[nodiscard]] static double theta(const VectorS2& point);
    [[nodiscard]] static bool is_zero(const VectorS2& v);
    [[nodiscard]] static bool is_zero(double v);
    [[nodiscard]] static VectorS2 project_to_xy_plane(const VectorS2& v);
    [[nodiscard]] static VectorS2 perpendicular_in_xy_plane(const VectorS2& v);
    [[nodiscard]] static bool parallel(const VectorS2& v1, const VectorS2& v2);
};

inline SphericalPolygonBoundingBoxCalculator::SphericalPolygonBoundingBoxCalculator(
    const std::vector<SphericalArc>& arcs
) : _arcs(arcs) {}

inline auto SphericalPolygonBoundingBoxCalculator::arc_z_intervals() const {
    return _arcs | std::views::transform(
        [](const SphericalArc& arc) { return arc_z_interval(arc); }
    );
}

inline auto SphericalPolygonBoundingBoxCalculator::arc_theta_intervals() const {
    return _arcs | std::views::transform(
        [](const SphericalArc& arc) { return arc_theta_interval(arc); }
    );
}

inline ThetaInterval SphericalPolygonBoundingBoxCalculator::theta_interval() const {
    return ThetaInterval::hull(arc_theta_intervals());
}

inline Interval SphericalPolygonBoundingBoxCalculator::z_interval() const {
    Interval interval = Interval::hull(arc_z_intervals());

    VectorS2 north_pole(0, 0, 1);
    VectorS2 south_pole(0, 0, -1);

    if (contains_point(north_pole)) {
        interval = Interval::hull(interval, north_pole.z());
    }

    if (contains_point(south_pole)) {
        interval = Interval::hull(interval, south_pole.z());
    }

    return interval;
}

inline SphericalBoundingBox SphericalPolygonBoundingBoxCalculator::calculate() const {
    if (_arcs.empty()) {
        return SphericalBoundingBox::full_sphere();
    }

    return {theta_interval(), z_interval()};
}

inline bool SphericalPolygonBoundingBoxCalculator::contains_point(const VectorS2& point) const {
    double angle_sum = 0;

    for (const auto& arc : _arcs) {
        const VectorS2& a = arc.source();
        const VectorS2& b = arc.target();

        VectorS2 ap = a.cross(point).normalized();
        VectorS2 bp = b.cross(point).normalized();

        double angle = globe::distance(ap, bp);
        const VectorS2& normal = arc.normal();

        double sign = normal.dot(point);

        if (is_zero(sign)) {
            return arc.contains(point);
        }

        angle_sum += (sign > 0) ? angle : -angle;
    }

    return std::fabs(angle_sum - (2 * M_PI)) < M_PI;
}

inline Interval SphericalPolygonBoundingBoxCalculator::arc_z_interval(const SphericalArc& arc) {
    const VectorS2& n = arc.normal();
    Interval z_int = z_interval_from_endpoints(arc);
    VectorS2 n_cross_z = n.cross(VectorS2(0, 0, 1));

    if (is_zero(n_cross_z)) {
        return z_int;
    }

    VectorS2 critical_dir = n_cross_z.cross(n);

    if (is_zero(critical_dir)) {
        return z_int;
    }

    VectorS2 critical_point_up = critical_dir.normalized();
    VectorS2 critical_point_down = -critical_point_up;

    if (arc.contains(critical_point_up)) {
        z_int = Interval::hull(z_int, critical_point_up.z());
    }

    if (arc.contains(critical_point_down)) {
        z_int = Interval::hull(z_int, critical_point_down.z());
    }

    return z_int;
}

inline ThetaInterval SphericalPolygonBoundingBoxCalculator::arc_theta_interval(const SphericalArc& arc) {
    const VectorS2& n = arc.normal();
    const VectorS2& source_v = arc.source();
    const VectorS2& target_v = arc.target();
    double theta_source = theta(source_v);
    double theta_target = theta(target_v);
    bool is_horizontal = parallel(n, VectorS2(0, 0, 1));

    if (is_horizontal) {
        bool ccw = n.z() > 0;
        if (ccw) {
            return ThetaInterval::from_to(theta_source, theta_target);
        } else {
            return ThetaInterval::from_to(theta_target, theta_source);
        }
    }

    VectorS2 n_xy = project_to_xy_plane(n);

    if (is_zero(n_xy)) {
        return ThetaInterval::full();
    }

    VectorS2 midpoint_dir = source_v + target_v;
    if (is_zero(midpoint_dir)) {
        return ThetaInterval::full();
    }
    VectorS2 midpoint = midpoint_dir.normalized();

    bool short_arc = arc.contains(midpoint);

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

    VectorS2 n_xy_normalized = n_xy.normalized();
    VectorS2 radial_dir1 = perpendicular_in_xy_plane(n_xy_normalized);
    VectorS2 radial_dir2 = -radial_dir1;

    VectorS2 critical_dir1 = n.cross(radial_dir1);
    VectorS2 critical_dir2 = n.cross(radial_dir2);

    if (is_zero(critical_dir1)) {
        return result;
    }

    VectorS2 critical_point1 = critical_dir1.normalized();
    VectorS2 critical_point2 = critical_dir2.normalized();

    if (arc.contains(critical_point1)) {
        result = ThetaInterval::hull(result, ThetaInterval(theta(critical_point1), 0));
    }

    if (arc.contains(critical_point2)) {
        result = ThetaInterval::hull(result, ThetaInterval(theta(critical_point2), 0));
    }

    return result;
}

inline Interval SphericalPolygonBoundingBoxCalculator::z_interval_from_endpoints(const SphericalArc& arc) {
    return Interval::hull(arc.source().z(), arc.target().z());
}

inline double SphericalPolygonBoundingBoxCalculator::theta(const VectorS2& point) {
    double t = std::atan2(point.y(), point.x());
    return t < 0.0 ? t + 2.0 * M_PI : t;
}

inline bool SphericalPolygonBoundingBoxCalculator::is_zero(const VectorS2& v) {
    return v.squaredNorm() < EPSILON;
}

inline bool SphericalPolygonBoundingBoxCalculator::is_zero(double v) {
    return std::abs(v) < EPSILON;
}

inline VectorS2 SphericalPolygonBoundingBoxCalculator::project_to_xy_plane(const VectorS2& v) {
    return VectorS2(v.x(), v.y(), 0.0);
}

inline VectorS2 SphericalPolygonBoundingBoxCalculator::perpendicular_in_xy_plane(const VectorS2& v) {
    return VectorS2(v.y(), -v.x(), 0.0);
}

inline bool SphericalPolygonBoundingBoxCalculator::parallel(const VectorS2& v1, const VectorS2& v2) {
    double v1_squared_length = v1.squaredNorm();
    double v2_squared_length = v2.squaredNorm();

    if (v1_squared_length < EPSILON || v2_squared_length < EPSILON) {
        return true;
    }

    VectorS2 cross = v1.cross(v2);
    return cross.squaredNorm() / (v1_squared_length * v2_squared_length) < EPSILON;
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_BOUNDING_BOX_CALCULATOR_HPP_
