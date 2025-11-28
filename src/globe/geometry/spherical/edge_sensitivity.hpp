#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_EDGE_SENSITIVITY_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_EDGE_SENSITIVITY_HPP_

#include "../../types.hpp"
#include "helpers.hpp"
#include <Eigen/Core>
#include <cmath>

namespace globe {

inline Eigen::Vector3d to_eigen_vec(const Vector3& v) {
    return Eigen::Vector3d(v.x(), v.y(), v.z());
}

inline Eigen::Vector3d to_eigen_vec(const Point3& p) {
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}

inline Eigen::Vector3d compute_edge_sensitivity(
    const Point3& site_i,
    const Point3& site_j,
    const Point3& edge_v1,
    const Point3& edge_v2
) {
    Eigen::Vector3d si = to_eigen_vec(site_i).normalized();
    Eigen::Vector3d sj = to_eigen_vec(site_j).normalized();

    Eigen::Vector3d toward_j = sj - si;

    Eigen::Vector3d tangent = toward_j - si.dot(toward_j) * si;
    double tangent_norm = tangent.norm();

    if (tangent_norm < GEOMETRIC_EPSILON) {
        return Eigen::Vector3d::Zero();
    }

    tangent /= tangent_norm;

    Eigen::Vector3d v1 = to_eigen_vec(edge_v1).normalized();
    Eigen::Vector3d v2 = to_eigen_vec(edge_v2).normalized();

    double cos_angle = v1.dot(v2);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double edge_length = std::acos(cos_angle);

    return 0.5 * edge_length * tangent;
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_EDGE_SENSITIVITY_HPP_
