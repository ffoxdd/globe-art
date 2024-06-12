#include "surface_mesh_point_range.h"

namespace globe {

PointIterator SurfaceMeshPointRange::begin() const {
    return PointIterator::begin(_mesh);
}

PointIterator SurfaceMeshPointRange::end() const {
    return PointIterator::end(_mesh);
}

} // namespace globe
