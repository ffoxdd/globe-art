#ifndef GLOBEART_SRC_GLOBE_TYPES_H_
#define GLOBEART_SRC_GLOBE_TYPES_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>

namespace globe {

using Vector3 = CGAL::Exact_predicates_inexact_constructions_kernel::Vector_3;
using Point3 = CGAL::Exact_predicates_inexact_constructions_kernel::Point_3;
using SurfaceMesh = CGAL::Surface_mesh<Point3>;

using Arc = CGAL::Delaunay_triangulation_on_sphere_2<
    CGAL::Delaunay_triangulation_on_sphere_traits_2<
        CGAL::Exact_predicates_inexact_constructions_kernel,
        CGAL::Exact_spherical_kernel_3
    >
>::Arc_on_sphere_2;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_TYPES_H_


