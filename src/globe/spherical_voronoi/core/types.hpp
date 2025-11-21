#ifndef GLOBEART_SRC_GLOBE_VORONOI_CORE_TYPES_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_CORE_TYPES_HPP_

#include "../../types.hpp"
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>

namespace globe {

using VertexHandle = CGAL::Delaunay_triangulation_on_sphere_2<
    CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel, SphericalKernel>
>::Vertex_handle;

using Arc = CGAL::Delaunay_triangulation_on_sphere_2<
    CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel, SphericalKernel>
>::Arc_on_sphere_2;

struct DualNeighborhood {
    Point3 &point;
    std::vector<Arc> dual_cell_arcs;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_VORONOI_CORE_TYPES_HPP_

