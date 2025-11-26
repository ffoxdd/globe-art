#ifndef GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_

#include "../types.hpp"
#include "../geometry/spherical/helpers.hpp"
#include <CGAL/Exact_spherical_kernel_3.h>

namespace globe::testing {

inline Arc make_arc(const Vector3 &normal, const Point3 &source, const Point3 &target) {
    using SphericalKernel = CGAL::Exact_spherical_kernel_3;
    using SphericalCircle3 = SphericalKernel::Circle_3;

    return Arc(
        SphericalCircle3(
            SphericalPoint3(0, 0, 0),
            1.0,
            to_spherical_vector(normal)
        ),
        to_spherical_point(source),
        to_spherical_point(target)
    );
}

} // namespace globe::testing

#endif //GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_
