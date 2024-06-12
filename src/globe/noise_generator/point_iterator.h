#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_POINT_ITERATOR_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_POINT_ITERATOR_H_

#include "../types.h"

namespace globe {

class PointIterator {
 public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = Point3;
    using difference_type = std::ptrdiff_t;
    using pointer = Point3 *;
    using reference = Point3 &;

    PointIterator(VertexIterator vertex_iterator, const SurfaceMesh &mesh) :
        _vertex_iterator(vertex_iterator), _mesh(mesh) { };

    static PointIterator begin(const SurfaceMesh &mesh);
    static PointIterator end(const SurfaceMesh &mesh);

    Point3 operator*() const;
    PointIterator &operator++();
    PointIterator &operator++(int);
    bool operator==(const PointIterator &other);
    bool operator!=(const PointIterator &other);

 private:
    VertexIterator _vertex_iterator;
    const SurfaceMesh &_mesh;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_POINT_ITERATOR_H_
