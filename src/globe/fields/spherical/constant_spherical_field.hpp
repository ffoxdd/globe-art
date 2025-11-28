#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_CONSTANT_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_CONSTANT_SPHERICAL_FIELD_HPP_

#include "spherical_field.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"

namespace globe {

class ConstantSphericalField {
 public:
    explicit ConstantSphericalField(double value = 1.0);

    [[nodiscard]] double value(const Point3& point) const;
    [[nodiscard]] double mass(const PolygonMoments& moments) const;
    [[nodiscard]] double edge_integral(const ArcMoments& moments) const;

    [[nodiscard]] double total_mass() const;

 private:
    double _value;
};

inline ConstantSphericalField::ConstantSphericalField(double value) :
    _value(value) {
}

inline double ConstantSphericalField::value([[maybe_unused]] const Point3& point) const {
    return _value;
}

inline double ConstantSphericalField::mass(const PolygonMoments& moments) const {
    return _value * moments.area;
}

inline double ConstantSphericalField::edge_integral(const ArcMoments& moments) const {
    return _value * moments.length;
}

inline double ConstantSphericalField::total_mass() const {
    return _value * UNIT_SPHERE_AREA;
}

static_assert(SphericalField<ConstantSphericalField>);

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_CONSTANT_SPHERICAL_FIELD_HPP_
