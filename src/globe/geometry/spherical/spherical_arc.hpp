#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_ARC_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_ARC_HPP_

#include "helpers.hpp"
#include "../../types.hpp"
#include <CGAL/Kernel/global_functions.h>
#include <Eigen/Core>
#include <cmath>
#include <optional>

namespace globe {

class SphericalArc {
 public:
    SphericalArc(const Point3& source, const Point3& target, const Vector3& normal);
    SphericalArc(const Point3& source, const Point3& target);

    [[nodiscard]] const Point3& source() const { return _source; }
    [[nodiscard]] const Point3& target() const { return _target; }
    [[nodiscard]] const Vector3& normal() const { return _normal; }

    [[nodiscard]] double length() const;
    [[nodiscard]] SphericalArc subarc(const Point3& point) const;
    [[nodiscard]] bool contains(const Point3& point) const;

    [[nodiscard]] Eigen::Vector3d first_moment() const;
    [[nodiscard]] Eigen::Matrix3d second_moment() const;

 private:
    struct Moments {
        Eigen::Vector3d first;
        Eigen::Matrix3d second;
    };

    Point3 _source;
    Point3 _target;
    Vector3 _normal;

    mutable std::optional<Moments> _cached_moments;

    void compute_moments() const;

    static Eigen::Vector3d find_perpendicular(const Eigen::Vector3d& u);
};

inline SphericalArc::SphericalArc(
    const Point3& source,
    const Point3& target,
    const Vector3& normal
) : _source(source), _target(target), _normal(normal) {
}

inline SphericalArc::SphericalArc(const Point3& source, const Point3& target) :
    _source(source), _target(target) {
    Vector3 v1 = globe::to_position_vector(source);
    Vector3 v2 = globe::to_position_vector(target);
    Vector3 cross = CGAL::cross_product(v1, v2);
    _normal = globe::normalize(cross);
}

inline double SphericalArc::length() const {
    Eigen::Vector3d u1 = globe::to_eigen(_source).normalized();
    Eigen::Vector3d u2 = globe::to_eigen(_target).normalized();
    double cos_theta = std::clamp(u1.dot(u2), -1.0, 1.0);
    return std::acos(cos_theta);
}

inline SphericalArc SphericalArc::subarc(const Point3& point) const {
    return SphericalArc(_source, point, _normal);
}

inline bool SphericalArc::contains(const Point3& point) const {
    constexpr double EPSILON = 1e-10;

    Vector3 p = globe::to_position_vector(point);

    double dot_normal = CGAL::scalar_product(_normal, p);
    if (std::abs(dot_normal) > EPSILON) {
        return false;
    }

    return subarc(point).length() < length() + EPSILON;
}

inline Eigen::Vector3d SphericalArc::first_moment() const {
    if (!_cached_moments) {
        compute_moments();
    }
    return _cached_moments->first;
}

inline Eigen::Matrix3d SphericalArc::second_moment() const {
    if (!_cached_moments) {
        compute_moments();
    }
    return _cached_moments->second;
}

inline void SphericalArc::compute_moments() const {
    constexpr double EPSILON = 1e-10;

    Vector3 u1_cgal = globe::normalize(globe::to_position_vector(_source));
    Vector3 u2_cgal = globe::normalize(globe::to_position_vector(_target));

    Eigen::Vector3d u1 = globe::to_eigen(u1_cgal);
    Eigen::Vector3d u2 = globe::to_eigen(u2_cgal);

    double cos_theta = std::clamp(u1.dot(u2), -1.0, 1.0);
    double theta = std::acos(cos_theta);
    double sin_theta = std::sin(theta);

    if (theta < EPSILON) {
        _cached_moments = Moments{
            Eigen::Vector3d::Zero(),
            Eigen::Matrix3d::Zero()
        };
        return;
    }

    Eigen::Vector3d n_unnormalized = u2 - cos_theta * u1;
    double n_length = n_unnormalized.norm();

    Eigen::Vector3d n = (n_length < EPSILON)
        ? find_perpendicular(u1)
        : n_unnormalized / n_length;

    double cos_sin = cos_theta * sin_theta;
    double integral_cos_squared = (theta + cos_sin) / 2.0;
    double integral_sin_squared = (theta - cos_sin) / 2.0;
    double integral_cos_sin = (sin_theta * sin_theta) / 2.0;

    _cached_moments = Moments{
        sin_theta * u1 + (1.0 - cos_theta) * n,
        integral_cos_squared * (u1 * u1.transpose()) +
            integral_sin_squared * (n * n.transpose()) +
            integral_cos_sin * (u1 * n.transpose() + n * u1.transpose())
    };
}

inline Eigen::Vector3d SphericalArc::find_perpendicular(const Eigen::Vector3d& u) {
    Eigen::Vector3d candidate = (std::abs(u.z()) < 0.9)
        ? Eigen::Vector3d(0, 0, 1)
        : Eigen::Vector3d(1, 0, 0);

    Eigen::Vector3d perp = candidate - u.dot(candidate) * u;
    return perp.normalized();
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_ARC_HPP_
