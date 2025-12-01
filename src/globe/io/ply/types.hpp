#ifndef GLOBEART_SRC_GLOBE_IO_PLY_TYPES_HPP_
#define GLOBEART_SRC_GLOBE_IO_PLY_TYPES_HPP_

#include "../../cgal_types.hpp"
#include <CGAL/Surface_mesh.h>

namespace globe {

using SurfaceMesh = ::CGAL::Surface_mesh<cgal::Point3>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_IO_PLY_TYPES_HPP_
