#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_
#define GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_

#include <iostream>
#include <ranges>
#include <utility>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/centroid.h>
#include "../noise_generator/interval.hpp" // TODO: move interval out of noise_generator
#include "../geometry/helpers.hpp"
#include "../points_collection/types.hpp" // TODO: consider moving spherical_polygon closer to points_collection
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
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

Vector3 normalize(const Vector3 &vector) {
    double length = std::sqrt(vector.squared_length());

    if (length == 0) {
        return {0, 0, 0};
    }

    return {vector.x() / length, vector.y() / length, vector.z() / length};
}

Point3 spherical_interpolate(const Point3 &point1, const Point3 &point2, double t, const Point3 &center = ORIGIN) {
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

double spherical_angle(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
    Vector3 u = CGAL::cross_product(a, b);
    Vector3 v = CGAL::cross_product(c, b);

    std::cout << "u: " << u << std::endl;
    std::cout << "v: " << v << std::endl;

    u = normalize(u);
    v = normalize(v);

    double cos_abc = CGAL::scalar_product(u, v);
    return std::acos(std::clamp(cos_abc, -1.0, 1.0));
}

Point3 project_to_sphere(Point3 &point, const Point3 &center = ORIGIN, double radius = 1.0) {
    auto vector = point - ORIGIN;
    double scale = radius / std::sqrt(vector.squared_length());
    return center + (vector * scale);
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_

