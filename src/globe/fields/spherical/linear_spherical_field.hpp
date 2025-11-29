#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_SPHERICAL_FIELD_HPP_

#include "spherical_field.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"

namespace globe {

class LinearSphericalField {
 public:
    explicit LinearSphericalField(double slope = 1.0, double offset = 0.0);

    [[nodiscard]] double value(const Point3& point) const;
    [[nodiscard]] double mass(const PolygonMoments& moments) const;
    [[nodiscard]] double edge_integral(const ArcMoments& moments) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const ArcMoments& moments) const;

    [[nodiscard]] double total_mass() const;

 private:
    double _slope;
    double _offset;
};

inline LinearSphericalField::LinearSphericalField(double slope, double offset) :
    _slope(slope),
    _offset(offset) {
}

inline double LinearSphericalField::value(const Point3& point) const {
    return _slope * point.z() + _offset;
}

inline double LinearSphericalField::mass(const PolygonMoments& moments) const {
    return _slope * moments.first_moment.z() + _offset * moments.area;
}

inline double LinearSphericalField::edge_integral(const ArcMoments& moments) const {
    return _slope * moments.first_moment.z() + _offset * moments.length;
}

inline Eigen::Vector3d LinearSphericalField::edge_gradient_integral(
    const ArcMoments& moments
) const {
    return _slope * moments.second_moment.col(2) + _offset * moments.first_moment;
}

inline double LinearSphericalField::total_mass() const {
    return _offset * UNIT_SPHERE_AREA;
}

static_assert(SphericalField<LinearSphericalField>);

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_SPHERICAL_FIELD_HPP_
