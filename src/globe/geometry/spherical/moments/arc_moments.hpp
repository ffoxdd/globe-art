#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_MOMENTS_ARC_MOMENTS_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_MOMENTS_ARC_MOMENTS_HPP_

#include "../../../types.hpp"
#include "../helpers.hpp"
#include <Eigen/Core>
#include <cmath>

namespace globe {

struct ArcMoments {
    double length;
    Eigen::Vector3d first_moment;
    Eigen::Matrix3d second_moment;
};

inline Eigen::Vector3d to_eigen(const Vector3& v) {
    return Eigen::Vector3d(v.x(), v.y(), v.z());
}

inline Eigen::Vector3d find_perpendicular(const Eigen::Vector3d& u) {
    Eigen::Vector3d candidate = (std::abs(u.z()) < 0.9)
        ? Eigen::Vector3d(0, 0, 1)
        : Eigen::Vector3d(1, 0, 0);

    Eigen::Vector3d perp = candidate - u.dot(candidate) * u;
    return perp.normalized();
}

inline ArcMoments compute_arc_moments(const Point3& v1, const Point3& v2) {
    Vector3 u1_cgal = normalize(to_position_vector(v1));
    Vector3 u2_cgal = normalize(to_position_vector(v2));

    Eigen::Vector3d u1 = to_eigen(u1_cgal);
    Eigen::Vector3d u2 = to_eigen(u2_cgal);

    double cos_theta = u1.dot(u2);
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    double theta = std::acos(cos_theta);
    double sin_theta = std::sin(theta);

    if (theta < GEOMETRIC_EPSILON) {
        return {0.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
    }

    Eigen::Vector3d n_unnormalized = u2 - cos_theta * u1;
    double n_length = n_unnormalized.norm();

    Eigen::Vector3d n;
    if (n_length < GEOMETRIC_EPSILON) {
        n = find_perpendicular(u1);
    } else {
        n = n_unnormalized / n_length;
    }

    double arc_length = theta;

    Eigen::Vector3d first_moment = sin_theta * u1 + (1.0 - cos_theta) * n;

    double cos_sin = cos_theta * sin_theta;
    double integral_cos_squared = (theta + cos_sin) / 2.0;
    double integral_sin_squared = (theta - cos_sin) / 2.0;
    double integral_cos_sin = (sin_theta * sin_theta) / 2.0;

    Eigen::Matrix3d second_moment =
        integral_cos_squared * (u1 * u1.transpose()) +
        integral_sin_squared * (n * n.transpose()) +
        integral_cos_sin * (u1 * n.transpose() + n * u1.transpose());

    return {arc_length, first_moment, second_moment};
}

inline ArcMoments compute_arc_moments(const Vector3& v1, const Vector3& v2) {
    return compute_arc_moments(to_point(v1), to_point(v2));
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_MOMENTS_ARC_MOMENTS_HPP_
