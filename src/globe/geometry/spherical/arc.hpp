#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_ARC_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_ARC_HPP_

#include "helpers.hpp"
#include "../../types.hpp"
#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>

namespace globe::geometry::spherical {

using globe::VectorS2;
using globe::GEOMETRIC_EPSILON;

class Arc {
 public:
    Arc(const VectorS2& source, const VectorS2& target, const VectorS2& normal);
    Arc(const VectorS2& source, const VectorS2& target);

    [[nodiscard]] const VectorS2& source() const { return _source; }
    [[nodiscard]] const VectorS2& target() const { return _target; }
    [[nodiscard]] const VectorS2& normal() const { return _normal; }

    [[nodiscard]] double length() const;
    [[nodiscard]] VectorS2 interpolate(double t) const;
    [[nodiscard]] Arc subarc(const VectorS2& point) const;
    [[nodiscard]] bool contains(const VectorS2& point) const;

    [[nodiscard]] VectorS2 first_moment() const;
    [[nodiscard]] Eigen::Matrix3d second_moment() const;

 private:
    struct Moments {
        VectorS2 first;
        Eigen::Matrix3d second;
    };

    VectorS2 _source;
    VectorS2 _target;
    VectorS2 _normal;

    mutable std::optional<Moments> _cached_moments;

    void compute_moments() const;

    static VectorS2 find_perpendicular(const VectorS2& u);
};

inline Arc::Arc(
    const VectorS2& source,
    const VectorS2& target,
    const VectorS2& normal
) : _source(source), _target(target), _normal(normal) {
}

inline Arc::Arc(const VectorS2& source, const VectorS2& target) :
    _source(source), _target(target) {
    _normal = _source.cross(_target).normalized();
}

inline double Arc::length() const {
    return distance(_source, _target);
}

inline VectorS2 Arc::interpolate(double t) const {
    double theta = length();

    if (theta < GEOMETRIC_EPSILON) {
        return _source;
    }

    double sin_theta = std::sin(theta);
    double a = std::sin((1.0 - t) * theta) / sin_theta;
    double b = std::sin(t * theta) / sin_theta;

    return (a * _source + b * _target).normalized();
}

inline Arc Arc::subarc(const VectorS2& point) const {
    return Arc(_source, point, _normal);
}

inline bool Arc::contains(const VectorS2& point) const {
    constexpr double EPSILON = 1e-10;

    double dot_normal = _normal.dot(point);
    if (std::abs(dot_normal) > EPSILON) {
        return false;
    }

    return subarc(point).length() < length() + EPSILON;
}

inline VectorS2 Arc::first_moment() const {
    if (!_cached_moments) {
        compute_moments();
    }
    return _cached_moments->first;
}

inline Eigen::Matrix3d Arc::second_moment() const {
    if (!_cached_moments) {
        compute_moments();
    }
    return _cached_moments->second;
}

inline void Arc::compute_moments() const {
    constexpr double EPSILON = 1e-10;

    VectorS2 u1 = _source.normalized();
    VectorS2 u2 = _target.normalized();

    double theta = distance(u1, u2);
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    if (theta < EPSILON) {
        _cached_moments = Moments{
            VectorS2::Zero(),
            Eigen::Matrix3d::Zero()
        };
        return;
    }

    VectorS2 n_unnormalized = u2 - cos_theta * u1;
    double n_length = n_unnormalized.norm();

    VectorS2 n = (n_length < EPSILON)
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

inline VectorS2 Arc::find_perpendicular(const VectorS2& u) {
    VectorS2 candidate = (std::abs(u.z()) < 0.9)
        ? VectorS2(0, 0, 1)
        : VectorS2(1, 0, 0);

    VectorS2 perp = candidate - u.dot(candidate) * u;
    return perp.normalized();
}

} // namespace globe::geometry::spherical

namespace globe {
using Arc = geometry::spherical::Arc;
using SphericalArc = Arc;  // deprecated alias
}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_ARC_HPP_
