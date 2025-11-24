#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_

#include <cmath>
#include <algorithm>
#include "../../types.hpp"

namespace globe {

const Point3 ORIGIN = Point3(0, 0, 0);

template<typename ForeignPoint>
Point3 to_point(const ForeignPoint &point) {
    return {
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y()),
        CGAL::to_double(point.z()),
    };
}

template<typename PointType>
Vector3 position_vector(const PointType &point) {
    return Vector3(
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y()),
        CGAL::to_double(point.z())
    );
}

inline Vector3 normalize(const Vector3 &vector) {
    double length = std::sqrt(vector.squared_length());

    if (length == 0) {
        return {0, 0, 0};
    }

    return {vector.x() / length, vector.y() / length, vector.z() / length};
}

inline Point3 spherical_interpolate(const Point3 &point1, const Point3 &point2, double t) {
    Vector3 v1 = position_vector(point1);
    Vector3 v2 = position_vector(point2);

    Vector3 v1_normalized = normalize(v1);
    Vector3 v2_normalized = normalize(v2);

    double dot_product = v1_normalized * v2_normalized;
    double theta = std::acos(dot_product);

    double sin_theta = std::sin(theta);
    double alpha = std::sin((1 - t) * theta) / sin_theta;
    double beta = std::sin(t * theta) / sin_theta;

    Vector3 interpolated_vector = (alpha * v1) + (beta * v2);

    return Point3(interpolated_vector.x(), interpolated_vector.y(), interpolated_vector.z());
}

inline double theta(double x, double y) {
    double t = std::atan2(y, x);
    return t < 0.0 ? t + 2.0 * M_PI : t;
}

inline double theta(const Point3 &point) {
    return theta(point.x(), point.y());
}

inline double angular_distance(const Vector3 &a, const Vector3 &b) {
    Vector3 a_ = normalize(a);
    Vector3 b_ = normalize(b);

    double cos_theta = CGAL::scalar_product(a_, b_);
    return std::acos(std::clamp(cos_theta, -1.0, 1.0));
}

inline double spherical_angle(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
    return angular_distance(
        CGAL::cross_product(b, a),
        CGAL::cross_product(b, c)
    );
}

inline Point3 project_to_sphere(const Point3 &point) {
    Vector3 v = position_vector(point);
    Vector3 normalized = normalize(v);
    return Point3(normalized.x(), normalized.y(), normalized.z());
}

inline Point3 antipodal(const Point3 &point) {
    return Point3(-point.x(), -point.y(), -point.z());
}
struct TangentBasis {
    Vector3 tangent_u;
    Vector3 tangent_v;
};

inline TangentBasis build_tangent_basis(const Vector3 &normal) {
    Vector3 tangent_u;

    if (std::abs(normal.z()) < 0.9) {
        tangent_u = Vector3(normal.y(), -normal.x(), 0.0);
    } else {
        tangent_u = Vector3(0.0, normal.z(), -normal.y());
    }

    tangent_u = normalize(tangent_u);

    Vector3 tangent_v = CGAL::cross_product(normal, tangent_u);

    return {tangent_u, tangent_v};
}

inline Point3 stereographic_plane_to_sphere(
    double u,
    double v,
    const Point3 &south_pole,
    const Vector3 &tangent_u,
    const Vector3 &tangent_v
) {
    double r_squared = u * u + v * v;
    double scale = 4.0 / (4.0 + r_squared);

    Point3 result(
        south_pole.x() + scale * (u * tangent_u.x() + v * tangent_v.x() - south_pole.x() * r_squared / 4.0),
        south_pole.y() + scale * (u * tangent_u.y() + v * tangent_v.y() - south_pole.y() * r_squared / 4.0),
        south_pole.z() + scale * (u * tangent_u.z() + v * tangent_v.z() - south_pole.z() * r_squared / 4.0)
    );

    double len = std::sqrt(
        result.x() * result.x() +
        result.y() * result.y() +
        result.z() * result.z()
    );

    return Point3(result.x() / len, result.y() / len, result.z() / len);
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_

