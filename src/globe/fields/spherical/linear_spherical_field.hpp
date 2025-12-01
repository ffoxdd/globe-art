#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_SPHERICAL_FIELD_HPP_

#include "spherical_field.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"

namespace globe {

class LinearSphericalField {
 public:
    explicit LinearSphericalField(double slope = 1.0, double offset = 0.0);

    [[nodiscard]] double value(const VectorS2& point) const;
    [[nodiscard]] double mass(const SphericalPolygon& polygon) const;
    [[nodiscard]] double edge_integral(const SphericalArc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const SphericalArc& arc) const;

    [[nodiscard]] double total_mass() const;

 private:
    double _slope;
    double _offset;
};

inline LinearSphericalField::LinearSphericalField(double slope, double offset) :
    _slope(slope),
    _offset(offset) {
}

inline double LinearSphericalField::value(const VectorS2& point) const {
    return _slope * point.z() + _offset;
}

inline double LinearSphericalField::mass(const SphericalPolygon& polygon) const {
    return _slope * polygon.first_moment().z() + _offset * polygon.area();
}

inline double LinearSphericalField::edge_integral(const SphericalArc& arc) const {
    return _slope * arc.first_moment().z() + _offset * arc.length();
}

inline Eigen::Vector3d LinearSphericalField::edge_gradient_integral(
    const SphericalArc& arc
) const {
    return _slope * arc.second_moment().col(2) + _offset * arc.first_moment();
}

inline double LinearSphericalField::total_mass() const {
    return _offset * UNIT_SPHERE_AREA;
}

static_assert(SphericalField<LinearSphericalField>);

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_SPHERICAL_FIELD_HPP_
