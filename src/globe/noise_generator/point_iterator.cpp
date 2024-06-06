#include "point_iterator.h"

namespace globe {

PointIterator SurfaceMeshPointRange::begin() const {
    return PointIterator::begin(_mesh);
}

PointIterator SurfaceMeshPointRange::end() const {
    return PointIterator::end(_mesh);
}

PointIterator PointIterator::begin(const SurfaceMesh& mesh) {
    return {mesh.vertices_begin(), mesh};
}

PointIterator PointIterator::end(const SurfaceMesh& mesh) {
    return {mesh.vertices_end(), mesh};
}

Point3 PointIterator::operator*() const {
    return _mesh.point(*_vertex_iterator);
}

PointIterator &PointIterator::operator++() {
    ++_vertex_iterator;
    return *this;
}

PointIterator &PointIterator::operator++(int) {
    PointIterator tmp(*this);
    ++(*this);
    return tmp;
}

bool PointIterator::operator==(const PointIterator &other) {
    return _vertex_iterator == other._vertex_iterator;
}

bool PointIterator::operator!=(const PointIterator &other) {
    return _vertex_iterator != other._vertex_iterator;
}

}
