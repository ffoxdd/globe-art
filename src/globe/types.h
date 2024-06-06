#ifndef GLOBEART_SRC_GLOBE_TYPES_H_
#define GLOBEART_SRC_GLOBE_TYPES_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace globe {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point3 = Kernel::Point_3;
using SurfaceMesh = CGAL::Surface_mesh<Point3>;
using VertexIterator = SurfaceMesh::Vertex_iterator;
//using VertexColorMap = SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_TYPES_H_


