#ifndef GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
#define GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_

#include "../types.hpp"
#include "types.hpp"
#include "handle_iterator.hpp"
#include <ranges>

namespace globe {

using VertexHandleValue = typename std::iterator_traits<FiniteVerticesIterator>::value_type::Vertex_handle;
using FaceHandleValue = typename std::iterator_traits<FiniteFacesIterator>::value_type::Face_handle;

using VertexHandleIterator = HandleIterator<FiniteVerticesIterator, VertexHandleValue>;
using FaceHandleIterator = HandleIterator<FiniteFacesIterator, FaceHandleValue>;

class PointsCollection {
 public:
    void insert(Point3 point);
    bool empty() const;
    std::vector<Point3> nearby_points(Point3 point, double radius) const;

    auto points() {
        return std::ranges::subrange(_points.begin(), _points.end());
    };

    auto edges() {
        return std::ranges::subrange(
            _triangulation.finite_edges_begin(),
            _triangulation.finite_edges_end()
        );
    }

    auto all_edges() {
        return std::ranges::subrange(
            _triangulation.all_edges_begin(),
            _triangulation.all_edges_end()
        );
    }

    auto faces() {
        return std::ranges::subrange(
            FaceHandleIterator(_triangulation.finite_faces_begin()),
            FaceHandleIterator(_triangulation.finite_faces_end())
        );
    }

    auto dual_arcs() {
        return all_edges() | std::views::transform(
            [this](Edge edge) {
                return this->_triangulation.dual_on_sphere(edge);
            }
        );
    }

    auto segments() {
        return edges() | std::views::transform(
            [this](Edge edge) {
                return this->_triangulation.segment(edge);
            }
        );
    }

 private:
    std::vector<Point3> _points;
    KDTree _kd_tree;
    Triangulation _triangulation;
};

void PointsCollection::insert(Point3 point) {
    _points.push_back(point);
    _kd_tree.insert(point);
    _triangulation.insert(point);
}

bool PointsCollection::empty() const {
    return _points.empty();
}

std::vector<Point3> PointsCollection::nearby_points(Point3 point, double radius) const {
    FuzzySphere search_sphere(point, radius);

    std::vector<Point3> nearby_points;
    _kd_tree.search(std::back_inserter(nearby_points), search_sphere);

    return nearby_points;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
