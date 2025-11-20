#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_
#define GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_

#include <cmath>
#include <algorithm>
#include "../types.hpp"

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

template<typename P>
Vector3 position_vector(const P &point) {
    return to_point(point) - ORIGIN;
}

inline Vector3 normalize(const Vector3 &vector) {
    double length = std::sqrt(vector.squared_length());

    if (length == 0) {
        return {0, 0, 0};
    }

    return {vector.x() / length, vector.y() / length, vector.z() / length};
}

inline Point3 normalize(const Point3 &point) {
    Vector3 vec = point - ORIGIN;
    Vector3 normalized = normalize(vec);
    return Point3(normalized.x(), normalized.y(), normalized.z());
}

inline Point3 spherical_interpolate(const Point3 &point1, const Point3 &point2, double t, const Point3 &center = ORIGIN) {
    globe::Vector3 v1 = point1 - center;
    globe::Vector3 v2 = point2 - center;

    globe::Vector3 v1_ = normalize(v1);
    globe::Vector3 v2_ = normalize(v2);

    double dot_product = v1_ * v2_;
    double theta = acos(dot_product);

    double sin_theta = sin(theta);
    double alpha = sin((1 - t) * theta) / sin_theta;
    double beta = sin(t * theta) / sin_theta;

    Vector3 interpolated_vector = (alpha * v1) + (beta * v2);

    return center + interpolated_vector;
}

inline double distance(const Point3 &a, const Point3 &b) {
    return std::sqrt(CGAL::squared_distance(a, b));
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

inline Point3 project_to_sphere(Point3 &point, const Point3 &center = ORIGIN, double radius = 1.0) {
    auto vector = point - ORIGIN;
    double scale = radius / std::sqrt(vector.squared_length());
    return center + (vector * scale);
}

inline Point3 antipodal(const Point3 &point) {
    return Point3(-point.x(), -point.y(), -point.z());
}

struct TangentBasis {
    Point3 tangent_u;
    Point3 tangent_v;
};

inline TangentBasis build_tangent_basis(const Point3 &normal) {
    Point3 tangent_u;

    if (std::abs(normal.z()) < 0.9) {
        tangent_u = Point3(normal.y(), -normal.x(), 0.0);
    } else {
        tangent_u = Point3(0.0, normal.z(), -normal.y());
    }

    Vector3 tangent_u_vec = normalize(tangent_u - ORIGIN);
    tangent_u = Point3(tangent_u_vec.x(), tangent_u_vec.y(), tangent_u_vec.z());

    Point3 tangent_v(
        normal.y() * tangent_u.z() - normal.z() * tangent_u.y(),
        normal.z() * tangent_u.x() - normal.x() * tangent_u.z(),
        normal.x() * tangent_u.y() - normal.y() * tangent_u.x()
    );

    return {tangent_u, tangent_v};
}

inline Point3 stereographic_plane_to_sphere(
    double u,
    double v,
    const Point3 &south_pole,
    const Point3 &tangent_u,
    const Point3 &tangent_v
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

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_

