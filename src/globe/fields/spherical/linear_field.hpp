#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_FIELD_HPP_

#include "field.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/arc.hpp"
#include "../../geometry/spherical/polygon/polygon.hpp"

namespace globe::fields::spherical {

using geometry::spherical::UNIT_SPHERE_AREA;

class LinearField {
 public:
    explicit LinearField(double slope = 1.0, double offset = 0.0);

    [[nodiscard]] double value(const VectorS2& point) const;
    [[nodiscard]] double mass(const Polygon& polygon) const;
    [[nodiscard]] double edge_integral(const Arc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const Arc& arc) const;

    [[nodiscard]] double total_mass() const;

 private:
    double _slope;
    double _offset;
};

inline LinearField::LinearField(double slope, double offset) :
    _slope(slope),
    _offset(offset) {
}

inline double LinearField::value(const VectorS2& point) const {
    return _slope * point.z() + _offset;
}

inline double LinearField::mass(const Polygon& polygon) const {
    return _slope * polygon.first_moment().z() + _offset * polygon.area();
}

inline double LinearField::edge_integral(const Arc& arc) const {
    return _slope * arc.first_moment().z() + _offset * arc.length();
}

inline Eigen::Vector3d LinearField::edge_gradient_integral(
    const Arc& arc
) const {
    return _slope * arc.second_moment().col(2) + _offset * arc.first_moment();
}

inline double LinearField::total_mass() const {
    return _offset * UNIT_SPHERE_AREA;
}

static_assert(Field<LinearField>);

} // namespace globe::fields::spherical

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_LINEAR_FIELD_HPP_
