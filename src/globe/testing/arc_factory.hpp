#ifndef GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_

#include "../types.hpp"
#include <CGAL/Exact_spherical_kernel_3.h>

namespace globe::testing {

inline Arc make_arc(double nx, double ny, double nz, double sx, double sy, double sz, double tx, double ty, double tz) {
    using SphericalKernel = CGAL::Exact_spherical_kernel_3;
    using SphericalCircle3 = SphericalKernel::Circle_3;
    using SphericalPoint3 = SphericalKernel::Point_3;
    using SphericalVector3 = SphericalKernel::Vector_3;

    return Arc(
        SphericalCircle3(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(nx, ny, nz)),
        SphericalPoint3(sx, sy, sz),
        SphericalPoint3(tx, ty, tz)
    );
}

inline Arc make_arc(const Vector3 &normal, const Point3 &source, const Point3 &target) {
    return make_arc(
        normal.x(), normal.y(), normal.z(),
        source.x(), source.y(), source.z(),
        target.x(), target.y(), target.z()
    );
}

} // namespace globe::testing

#endif //GLOBEART_SRC_GLOBE_TESTING_ARC_FACTORY_HPP_
