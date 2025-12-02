#ifndef GLOBEART_SRC_GLOBE_IO_PLY_MESH_TYPES_HPP_
#define GLOBEART_SRC_GLOBE_IO_PLY_MESH_TYPES_HPP_

#include "../../../cgal/types.hpp"
#include <CGAL/Surface_mesh.h>

namespace globe::io::ply::mesh {

using SurfaceMesh = ::CGAL::Surface_mesh<cgal::Point3>;
using VertexIndex = SurfaceMesh::Vertex_index;

} // namespace globe::io::ply::mesh

#endif //GLOBEART_SRC_GLOBE_IO_PLY_MESH_TYPES_HPP_
