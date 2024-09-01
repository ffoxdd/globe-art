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

double angular_distance(const Vector3 &a, const Vector3 &b) {
    Vector3 a_ = normalize(a);
    Vector3 b_ = normalize(b);

    double cos_theta = CGAL::scalar_product(a_, b_);
    return std::acos(std::clamp(cos_theta, -1.0, 1.0));
}

double spherical_angle(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
    return angular_distance(
        CGAL::cross_product(a, b),
        CGAL::cross_product(c, b)
    );
}

double interior_angle(const Arc &arc, const Point3 &point) {
    Vector3 a = position_vector(arc.source());
    Vector3 b = position_vector(point);
    Vector3 c = position_vector(arc.target());

    return spherical_angle(a, b, c);
}

Vector3 plane_normal(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
    return CGAL::cross_product(b - a, c - a);
}

Vector2 project_onto_plane(const Vector3 &normal, const Vector3 &point) {
    Vector3 projection = point - (CGAL::scalar_product(point, normal) / normal.squared_length()) * normal;

    Vector3 basis_1 = CGAL::cross_product(normal, Vector3(1, 0, 0));

    if (basis_1.squared_length() == 0) {
        basis_1 = CGAL::cross_product(normal, Vector3(0, 1, 0));
    }

    Vector3 basis_2 = CGAL::cross_product(normal, basis_1);

    basis_1 = normalize(basis_1);
    basis_2 = normalize(basis_2);

    return {
        CGAL::scalar_product(projection, basis_1),
        CGAL::scalar_product(projection, basis_2),
    };
}

CGAL::Orientation orientation_2d(const Vector3& a, const Vector3& b, const Vector3& c) {
    Vector3 normal = plane_normal(a, b, c);

    Vector2 a_ = project_onto_plane(normal, a);
    Vector2 b_ = project_onto_plane(normal, b);
    Vector2 c_ = project_onto_plane(normal, c);

    Vector2 u = b_ - a_;
    Vector2 v = c_ - a_;

    return CGAL::orientation(u, v);
}

CGAL::Orientation orientation(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
    CGAL::Orientation orientation = CGAL::orientation(a, b, c);

    if (orientation == CGAL::COLLINEAR) {
        return orientation_2d(a, b, c);
    }

    return orientation;
}

CGAL::Orientation orientation(const Arc &arc, const Point3 &point) {
    return orientation(
        position_vector(arc.source()),
        position_vector(arc.target()),
        position_vector(point)
    );
}

Point3 project_to_sphere(Point3 &point, const Point3 &center = ORIGIN, double radius = 1.0) {
    auto vector = point - ORIGIN;
    double scale = radius / std::sqrt(vector.squared_length());
    return center + (vector * scale);
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_

