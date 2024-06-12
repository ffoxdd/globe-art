#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SURFACE_MESH_POINT_RANGE_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SURFACE_MESH_POINT_RANGE_H_

#include "../types.h"
#include "point_iterator.h"

namespace globe {

class SurfaceMeshPointRange {
 public:
    explicit SurfaceMeshPointRange(const SurfaceMesh &mesh) : _mesh(mesh) { };

    [[nodiscard]] PointIterator begin() const;
    [[nodiscard]] PointIterator end() const;

 private:
    const SurfaceMesh &_mesh;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_SURFACE_MESH_POINT_RANGE_H_
