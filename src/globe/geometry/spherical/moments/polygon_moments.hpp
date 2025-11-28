#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_MOMENTS_POLYGON_MOMENTS_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_MOMENTS_POLYGON_MOMENTS_HPP_

#include "../../../types.hpp"
#include "../helpers.hpp"
#include "../spherical_polygon/spherical_polygon.hpp"
#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace globe {

struct PolygonMoments {
    double area;
    Eigen::Vector3d first_moment;
    Eigen::Matrix3d second_moment;
};

namespace detail {

inline Eigen::Vector3d to_eigen(const Vector3& v) {
    return Eigen::Vector3d(v.x(), v.y(), v.z());
}

inline Eigen::Vector3d to_eigen(const Point3& p) {
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}

inline double spherical_triangle_area(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c
) {
    Eigen::Vector3d ab = b - a.dot(b) * a;
    Eigen::Vector3d ac = c - a.dot(c) * a;
    Eigen::Vector3d ba = a - b.dot(a) * b;
    Eigen::Vector3d bc = c - b.dot(c) * b;
    Eigen::Vector3d ca = a - c.dot(a) * c;
    Eigen::Vector3d cb = b - c.dot(b) * c;

    double ab_norm = ab.norm();
    double ac_norm = ac.norm();
    double ba_norm = ba.norm();
    double bc_norm = bc.norm();
    double ca_norm = ca.norm();
    double cb_norm = cb.norm();

    if (ab_norm < 1e-12 || ac_norm < 1e-12 ||
        ba_norm < 1e-12 || bc_norm < 1e-12 ||
        ca_norm < 1e-12 || cb_norm < 1e-12) {
        return 0.0;
    }

    double cos_A = (ab / ab_norm).dot(ac / ac_norm);
    double cos_B = (ba / ba_norm).dot(bc / bc_norm);
    double cos_C = (ca / ca_norm).dot(cb / cb_norm);

    cos_A = std::clamp(cos_A, -1.0, 1.0);
    cos_B = std::clamp(cos_B, -1.0, 1.0);
    cos_C = std::clamp(cos_C, -1.0, 1.0);

    double A = std::acos(cos_A);
    double B = std::acos(cos_B);
    double C = std::acos(cos_C);

    return A + B + C - M_PI;
}

inline Eigen::Vector3d spherical_triangle_first_moment(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    double area
) {
    return (area / 3.0) * (a + b + c);
}

inline Eigen::Matrix3d spherical_triangle_second_moment(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    double area
) {
    return (area / 3.0) * (a * a.transpose() + b * b.transpose() + c * c.transpose());
}

}

inline PolygonMoments compute_polygon_moments(const SphericalPolygon& polygon) {
    std::vector<Eigen::Vector3d> vertices;
    for (const auto& point : polygon.points()) {
        vertices.push_back(detail::to_eigen(point));
    }

    if (vertices.size() < 3) {
        return {0.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
    }

    double total_area = 0.0;
    Eigen::Vector3d total_first_moment = Eigen::Vector3d::Zero();
    Eigen::Matrix3d total_second_moment = Eigen::Matrix3d::Zero();

    const Eigen::Vector3d& v0 = vertices[0];

    for (size_t i = 1; i + 1 < vertices.size(); ++i) {
        const Eigen::Vector3d& v1 = vertices[i];
        const Eigen::Vector3d& v2 = vertices[i + 1];

        double tri_area = detail::spherical_triangle_area(v0, v1, v2);

        total_area += tri_area;
        total_first_moment += detail::spherical_triangle_first_moment(v0, v1, v2, tri_area);
        total_second_moment += detail::spherical_triangle_second_moment(v0, v1, v2, tri_area);
    }

    return {total_area, total_first_moment, total_second_moment};
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_MOMENTS_POLYGON_MOMENTS_HPP_
