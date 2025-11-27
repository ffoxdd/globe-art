#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_

#include <cmath>
#include <algorithm>
#include <ranges>
#include <CGAL/Exact_spherical_kernel_3.h>
#include "../../types.hpp"
#include "../../math/interval.hpp"

namespace globe {

template<typename T>
concept HasXYZ = requires(const T &value) { value.x(); value.y(); value.z(); };

template<typename T>
concept HasXY = requires(const T &value) { value.x(); value.y(); };

template<typename T>
concept SquaredLength = requires(const T &squared_length) { std::sqrt(squared_length); };

template<typename T>
concept Vector3Range = std::ranges::range<T> && std::same_as<std::ranges::range_value_t<T>, Vector3>;

template<typename T>
concept ConvertibleToDouble = requires(const T &value) { CGAL::to_double(value); };

const Point3 ORIGIN = Point3(0, 0, 0);
const Point3 NORTH_POLE = Point3(0, 0, 1);
const Point3 SOUTH_POLE = Point3(0, 0, -1);
const Vector3 Z_AXIS = Vector3(0, 0, 1);

constexpr double GEOMETRIC_EPSILON = 1e-10;
constexpr double TWO_PI = 2.0 * M_PI;

inline bool is_zero(const Vector3 &vector) {
    return vector.squared_length() < GEOMETRIC_EPSILON;
}

inline bool is_zero(double value) {
    return std::abs(value) < GEOMETRIC_EPSILON;
}

inline bool is_equal(double a, double b) {
    return is_zero(a - b);
}

inline bool is_greater_equal(double a, double b) {
    return a >= b - GEOMETRIC_EPSILON;
}

inline bool is_pole(const Vector3 &position_vector) {
    return is_equal(std::abs(position_vector.z()), 1.0);
}

template<HasXYZ PointType>
Point3 to_point(const PointType &point) {
    return {
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y()),
        CGAL::to_double(point.z()),
    };
}

template<HasXYZ PointType>
inline SphericalPoint3 to_spherical_point(const PointType &point) {
    return SphericalPoint3(
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y()),
        CGAL::to_double(point.z())
    );
}

template<HasXYZ PointType>
Vector3 to_position_vector(const PointType &point) {
    return Vector3(
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y()),
        CGAL::to_double(point.z())
    );
}

template<HasXYZ VectorType>
inline SphericalVector3 to_spherical_vector(const VectorType &vector) {
    return SphericalVector3(
        CGAL::to_double(vector.x()),
        CGAL::to_double(vector.y()),
        CGAL::to_double(vector.z())
    );
}

inline double length(const Vector3 &vector) {
    return std::sqrt(vector.squared_length());
}

template<SquaredLength SquaredLengthType>
inline Vector3 normalize(const Vector3 &vector, const SquaredLengthType &squared_length) {
    if (is_zero(squared_length)) {
        return {0, 0, 0};
    }

    double length = std::sqrt(squared_length);

    return {
        vector.x() / length,
        vector.y() / length,
        vector.z() / length,
    };
}

inline Vector3 normalize(const Vector3 &vector) {
    return normalize(vector, vector.squared_length());
}

inline Point3 spherical_interpolate(const Vector3 &v1, const Vector3 &v2, double t) {
    double cos_theta = normalize(v1) * normalize(v2);
    double theta = std::acos(cos_theta);

    double sin_theta = std::sin(theta);
    double alpha = std::sin((1 - t) * theta) / sin_theta;
    double beta = std::sin(t * theta) / sin_theta;

    return to_point((alpha * v1) + (beta * v2));
}

template<HasXYZ PointType>
Point3 spherical_interpolate(const PointType &point1, const PointType &point2, double t) {
    return spherical_interpolate(
        to_position_vector(point1),
        to_position_vector(point2),
        t
    );
}

inline double theta(double x, double y) {
    double t = std::atan2(y, x);
    return t < 0.0 ? t + TWO_PI : t;
}

template<HasXY XYType>
inline double theta(const XYType &point) {
    return theta(
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y())
    );
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

template<HasXYZ PointType>
inline double spherical_angle(const PointType &a, const PointType &b, const PointType &c) {
    return spherical_angle(
        to_position_vector(a),
        to_position_vector(b),
        to_position_vector(c)
    );
}

inline Point3 project_to_sphere(const Vector3 &position_vector) {
    return to_point(normalize(position_vector));
}

template<SquaredLength SquaredLengthType>
inline Point3 project_to_sphere(const Vector3 &vector, const SquaredLengthType &squared_length) {
    Vector3 normalized = normalize(vector, squared_length);
    return to_point(normalized);
}

inline Point3 project_to_sphere(const Point3 &point) {
    return project_to_sphere(to_position_vector(point));
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

    if (is_pole(normal)) {
        tangent_u = Vector3(0.0, normal.z(), -normal.y());
    } else {
        tangent_u = Vector3(normal.y(), -normal.x(), 0.0);
    }

    tangent_u = normalize(tangent_u);
    Vector3 tangent_v = CGAL::cross_product(normal, tangent_u);

    return {tangent_u, tangent_v};
}

inline Vector3 arc_normal(const Vector3 &v1, const Vector3 &v2) {
    return normalize(CGAL::cross_product(v1, v2));
}

template<HasXYZ PointType>
inline Vector3 arc_normal(const PointType &p1, const PointType &p2) {
    return arc_normal(to_position_vector(p1), to_position_vector(p2));
}

inline Vector3 arc_normal(const Arc &arc) {
    auto normal = arc.supporting_circle().supporting_plane().orthogonal_vector();
    return to_position_vector(normal);
}

inline bool parallel(const Vector3 &v1, const Vector3 &v2) {
    auto v1_squared_length = v1.squared_length();
    auto v2_squared_length = v2.squared_length();

    if (v1_squared_length < GEOMETRIC_EPSILON || v2_squared_length < GEOMETRIC_EPSILON) {
        return true;
    }

    Vector3 cross = CGAL::cross_product(v1, v2);
    return cross.squared_length() / (v1_squared_length * v2_squared_length) < GEOMETRIC_EPSILON;
}

inline Point3 stereographic_plane_to_sphere(
    double u,
    double v,
    const Vector3 &south_pole,
    const Vector3 &tangent_u,
    const Vector3 &tangent_v
) {
    double r_squared = u * u + v * v;
    double scale = 4.0 / (4.0 + r_squared);

    Vector3 result = south_pole + scale * (
        u * tangent_u + v * tangent_v - south_pole * (r_squared / 4.0)
    );

    return project_to_sphere(result);
}

template<ConvertibleToDouble T>
inline Interval hull_interval(const Interval &interval, const T &value) {
    return Interval::hull(interval, CGAL::to_double(value));
}

template<ConvertibleToDouble T1, ConvertibleToDouble T2>
inline Interval hull_interval(const T1 &a, const T2 &b) {
    return Interval::hull(CGAL::to_double(a), CGAL::to_double(b));
}

inline Interval hull_interval(double a, double b) {
    return Interval::hull(a, b);
}

inline Arc subarc(const Arc &arc, const Vector3 &point) {
    return Arc(arc.supporting_circle(), arc.source(), to_spherical_point(point));
}

template<Vector3Range RangeType>
Vector3 average_vector(const RangeType &vectors) {
    Vector3 sum(0, 0, 0);
    size_t count = 0;

    for (const auto &vector : vectors) {
        sum += vector;
        ++count;
    }

    if (count == 0) {
        return Vector3(0, 0, 0);
    }

    return sum / static_cast<double>(count);
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_HELPERS_H_

