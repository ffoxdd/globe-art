#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_

#include "spherical_bounding_box.hpp"
#include "../../types.hpp"
#include "helpers.hpp"
#include "../../std_ext/ranges.hpp"
#include "../../math/interval.hpp"
#include <CGAL/assertions.h>
#include <utility>
#include <vector>
#include <cstddef>
#include <cmath>
#include <limits>
#include <algorithm>
#include <ranges>

namespace globe {

class SphericalPolygon {
 public:
    explicit SphericalPolygon(std::vector<Arc> arcs);

    [[nodiscard]] auto points() const;
    [[nodiscard]] SphericalBoundingBox bounding_box() const;
    [[nodiscard]] Point3 centroid() const;
    [[nodiscard]] bool contains(const Point3 &point) const;

 private:
    std::vector<Arc> _arcs;

    [[nodiscard]] bool arcs_form_closed_loop() const;
    [[nodiscard]] Interval theta_interval(double theta_min, double theta_max) const;
    [[nodiscard]] bool is_on_arc(const Arc &arc, const Vector3 &point_vector) const;
    [[nodiscard]] bool is_degenerate() const;
    [[nodiscard]] bool is_convex() const;
};

inline auto SphericalPolygon::points() const {
    return _arcs | std::views::transform(
        [](const Arc &arc) { return to_point(arc.source()); }
    );
}

inline SphericalPolygon::SphericalPolygon(std::vector<Arc> arcs) :
    _arcs(std::move(arcs)) {
    CGAL_precondition(!is_degenerate());
    CGAL_precondition(arcs_form_closed_loop());
}

inline bool SphericalPolygon::contains(const Point3 &point) const {
    CGAL_precondition(is_convex());

    Vector3 p = position_vector(point);
    double angle_sum = 0;

    for (const auto &arc : _arcs) {
        Vector3 a = position_vector(arc.source());
        Vector3 b = position_vector(arc.target());

        Vector3 ap = CGAL::cross_product(a, p);
        Vector3 bp = CGAL::cross_product(b, p);

        double angle = angular_distance(ap, bp);

        auto normal = arc.supporting_circle().supporting_plane().orthogonal_vector();
        double sign = CGAL::scalar_product(position_vector(normal), p);

        if (sign == 0) {
            return is_on_arc(arc, p);
        }

        angle_sum += (sign > 0) ? angle : -angle;
    }

    return std::fabs(angle_sum - (2 * CGAL_PI)) < CGAL_PI;
}

inline bool SphericalPolygon::is_convex() const {
    return true; // TODO: implement
}


inline SphericalBoundingBox SphericalPolygon::bounding_box() const {
    double theta_min = std::numeric_limits<double>::max();
    double theta_max = std::numeric_limits<double>::lowest();

    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();

    bool has_points = false;

    for (const auto &point : points()) {
        has_points = true;

        double t = theta(point.x(), point.y());
        double z = static_cast<double>(point.z());

        theta_min = std::min(theta_min, t);
        theta_max = std::max(theta_max, t);

        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
    }

    if (!has_points) {
        return SphericalBoundingBox::full_sphere();
    }

    return {
        theta_interval(theta_min, theta_max),
        Interval(z_min, z_max),
    };
}

inline Point3 SphericalPolygon::centroid() const {
    Vector3 sum(0, 0, 0);
    size_t count = 0;

    for (const auto& point : points()) {
        sum += position_vector(point);
        ++count;
    }

    if (count == 0) {
        return bounding_box().center();
    }

    Vector3 average = sum / static_cast<double>(count);

    if (average.squared_length() < 1e-20) {
        return bounding_box().center();
    }

    return ORIGIN + normalize(average);
}

inline bool SphericalPolygon::arcs_form_closed_loop() const {
    for (const auto &[prev, next] : circular_adjacent_pairs(_arcs)) {
        if (prev.target() != next.source()) {
            return false;
        }
    }

    return true;
}

inline bool SphericalPolygon::is_degenerate() const {
    return _arcs.empty();
}

inline Interval SphericalPolygon::theta_interval(double theta_min, double theta_max) const {
    double theta_span = theta_max - theta_min;

    if (theta_span > M_PI) {
        return Interval(theta_max, theta_min + 2.0 * M_PI);
    } else {
        return Interval(theta_min, theta_max);
    }
}

inline bool SphericalPolygon::is_on_arc(const Arc &arc, const Vector3 &point_vector) const {
    using SphericalKernel = CGAL::Exact_spherical_kernel_3;
    using SphericalPoint3 = SphericalKernel::Point_3;

    SphericalPoint3 p_ = SphericalPoint3(point_vector.x(), point_vector.y(), point_vector.z());
    Arc p_arc = Arc(arc.supporting_circle(), arc.source(), p_);

    return p_arc.approximate_angle() < arc.approximate_angle();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_POLYGON_HPP_
