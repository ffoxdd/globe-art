#ifndef GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_

#include "../types.hpp"
#include "../geometry/spherical/spherical_arc.hpp"

namespace globe::testing {

inline SphericalArc make_arc(const Vector3& normal, const Point3& source, const Point3& target) {
    return SphericalArc(source, target, normal);
}

} // namespace globe::testing

#endif //GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_
