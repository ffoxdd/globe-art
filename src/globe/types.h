#ifndef GLOBEART_SRC_GLOBE_TYPES_H_
#define GLOBEART_SRC_GLOBE_TYPES_H_

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

namespace globe {

using Kernel = CGAL::Simple_cartesian<double>;
using Point3 = Kernel::Point_3;
using SurfaceMesh = CGAL::Surface_mesh<Point3>;
//using VertexColorMap = SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_TYPES_H_


