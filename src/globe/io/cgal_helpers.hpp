#ifndef GLOBEART_SRC_GLOBE_IO_CGAL_HELPERS_HPP_
#define GLOBEART_SRC_GLOBE_IO_CGAL_HELPERS_HPP_

#include <cmath>
#include <CGAL/Exact_spherical_kernel_3.h>
#include "../cgal_types.hpp"
#include "../geometry/spherical/helpers.hpp"

namespace globe::io {

inline cgal::Vector3 normalize(const cgal::Vector3 &vector) {
    auto squared_length = vector.squared_length();
    if (squared_length < GEOMETRIC_EPSILON) {
        return {0, 0, 0};
    }

    double length = std::sqrt(squared_length);
    return {
        vector.x() / length,
        vector.y() / length,
        vector.z() / length,
    };
}

inline cgal::Vector3 normal(const cgal::Vector3 &v1, const cgal::Vector3 &v2) {
    return normalize(::CGAL::cross_product(v1, v2));
}

template<VectorXYZ PointType>
inline cgal::Vector3 normal(const PointType &p1, const PointType &p2) {
    return normal(cgal::to_vector(p1), cgal::to_vector(p2));
}

inline cgal::Point3 interpolate(const cgal::Vector3 &v1, const cgal::Vector3 &v2, double t) {
    VectorS2 result = globe::interpolate(to_vector_s2(v1), to_vector_s2(v2), t);
    return cgal::to_point(result);
}

template<VectorXYZ PointType>
inline cgal::Point3 interpolate(const PointType &point1, const PointType &point2, double t) {
    return interpolate(cgal::to_vector(point1), cgal::to_vector(point2), t);
}

}

#endif //GLOBEART_SRC_GLOBE_IO_CGAL_HELPERS_HPP_
