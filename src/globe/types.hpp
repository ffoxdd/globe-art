#ifndef GLOBEART_SRC_GLOBE_TYPES_H_
#define GLOBEART_SRC_GLOBE_TYPES_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/Polygon_2.h>

namespace globe {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point2 = Kernel::Point_2;
using Polygon2 = CGAL::Polygon_2<Kernel>;
using Vector3 = Kernel::Vector_3;
using Direction3 = Kernel::Direction_3;
using Point3 = Kernel::Point_3;
using Plane3 = Kernel::Plane_3;
using SurfaceMesh = CGAL::Surface_mesh<Point3>;
using VertexIterator = SurfaceMesh::Vertex_iterator;
using SphericalKernel = CGAL::Exact_spherical_kernel_3;
using SphericalPoint3 = SphericalKernel::Point_3;

template<typename T>
concept Point3Range = std::ranges::range<T> && std::same_as<std::ranges::range_value_t<T>, Point3>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_TYPES_H_


