#ifndef GLOBEART_SRC_GLOBE_TYPES_H_
#define GLOBEART_SRC_GLOBE_TYPES_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>

namespace globe {

namespace detail {
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using SphericalKernel = CGAL::Exact_spherical_kernel_3;
    using DelaunayTraits = CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel, SphericalKernel>;
    using DelaunayTriangulation = CGAL::Delaunay_triangulation_on_sphere_2<DelaunayTraits>;
}

using Vector3 = detail::Kernel::Vector_3;
using Point3 = detail::Kernel::Point_3;
using Arc = detail::DelaunayTriangulation::Arc_on_sphere_2;
using SphericalPoint3 = detail::SphericalKernel::Point_3;
using SphericalVector3 = detail::SphericalKernel::Vector_3;
using SearchTraits = CGAL::Search_traits_3<detail::Kernel>;
using KDTree = CGAL::Kd_tree<SearchTraits>;
using FuzzySphere = CGAL::Fuzzy_sphere<SearchTraits>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_TYPES_H_


