#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_
#define GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_

#include "../types.hpp"

namespace globe {

Vector3 normalize(const Vector3 &vector) {
    double length = std::sqrt(vector.squared_length());
    return {vector.x() / length, vector.y() / length, vector.z() / length};
}

Point3 spherical_interpolate(const Point3 &point1,
    const globe::Point3 &point2,
    double t,
    const globe::Point3 &center
) {
    globe::Vector3 v1 = point1 - center;
    globe::Vector3 v2 = point2 - center;

    globe::Vector3 v1_ = normalize(v1);
    globe::Vector3 v2_ = normalize(v2);

    double dot_product = v1_ * v2_;
    double theta = acos(dot_product);

    double sin_theta = sin(theta);
    double alpha = sin((1 - t) * theta) / sin_theta;
    double beta = sin(t * theta) / sin_theta;

    globe::Vector3 interpolated_vector = (alpha * v1) + (beta * v2);

    return center + interpolated_vector;
}

template<typename ForeignPoint>
Point3 to_point(const ForeignPoint &point) {
    return {
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y()),
        CGAL::to_double(point.z()),
    };
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_HELPERS_H_

