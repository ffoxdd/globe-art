#ifndef GLOBEART_SRC_GLOBE_FIELDS_INTEGRABLE_CONSTANT_INTEGRABLE_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_INTEGRABLE_CONSTANT_INTEGRABLE_FIELD_HPP_

#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../types.hpp"

namespace globe {

class ConstantIntegrableField {
 public:
    explicit ConstantIntegrableField(double value = 1.0);

    [[nodiscard]] double integrate(const SphericalPolygon &polygon) const;
    [[nodiscard]] double integrate() const;
    [[nodiscard]] double max_frequency() const;

 private:
    double _value;
};

inline ConstantIntegrableField::ConstantIntegrableField(double value) :
    _value(value) {
}

inline double ConstantIntegrableField::integrate(const SphericalPolygon &polygon) const {
    return _value * polygon.area();
}

inline double ConstantIntegrableField::integrate() const {
    return _value * UNIT_SPHERE_AREA;
}

inline double ConstantIntegrableField::max_frequency() const {
    return 0.0;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_FIELDS_INTEGRABLE_CONSTANT_INTEGRABLE_FIELD_HPP_
