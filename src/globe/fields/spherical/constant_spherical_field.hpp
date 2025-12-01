#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_CONSTANT_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_CONSTANT_SPHERICAL_FIELD_HPP_

#include "spherical_field.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"

namespace globe {

class ConstantSphericalField {
 public:
    explicit ConstantSphericalField(double value = 1.0);

    [[nodiscard]] double value(const VectorS2& point) const;
    [[nodiscard]] double mass(const SphericalPolygon& polygon) const;
    [[nodiscard]] double edge_integral(const SphericalArc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const SphericalArc& arc) const;

    [[nodiscard]] double total_mass() const;

 private:
    double _value;
};

inline ConstantSphericalField::ConstantSphericalField(double value) :
    _value(value) {
}

inline double ConstantSphericalField::value([[maybe_unused]] const VectorS2& point) const {
    return _value;
}

inline double ConstantSphericalField::mass(const SphericalPolygon& polygon) const {
    return _value * polygon.area();
}

inline double ConstantSphericalField::edge_integral(const SphericalArc& arc) const {
    return _value * arc.length();
}

inline Eigen::Vector3d ConstantSphericalField::edge_gradient_integral(
    const SphericalArc& arc
) const {
    return _value * arc.first_moment();
}

inline double ConstantSphericalField::total_mass() const {
    return _value * UNIT_SPHERE_AREA;
}

static_assert(SphericalField<ConstantSphericalField>);

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_CONSTANT_SPHERICAL_FIELD_HPP_
