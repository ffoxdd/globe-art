#ifndef GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
#define GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_

#include "../types.hpp"
#include "types.hpp"
#include "handle_iterator.hpp"
#include "handle_circulator_iterator.hpp"
#include "circulator_iterator.hpp"
#include "../geometry/helpers.hpp"
#include <ranges>

namespace globe {

// TODO: consider what should go here and what should go in ./types.hpp
using VertexHandleValue = typename std::iterator_traits<FiniteVerticesIterator>::value_type::Vertex_handle;
using VertexHandleIterator = HandleIterator<FiniteVerticesIterator, VertexHandleValue>;
using FaceHandleValue = typename std::iterator_traits<FiniteFacesIterator>::value_type::Face_handle;
using FaceHandleIterator = HandleIterator<FiniteFacesIterator, FaceHandleValue>;
using VertexCirculatorIterator = CirculatorIterator<VertexCirculator, VertexHandle>;
using EdgeCirculatorIterator = CirculatorIterator<EdgeCirculator, Edge>;
using FaceHandleCirculatorValue = typename std::iterator_traits<FaceCirculator>::value_type::Face_handle;
using FaceHandleCirculatorIterator = HandleCirculatorIterator<FaceCirculator, FaceHandleCirculatorValue>;

class PointsCollection {
 public:
    explicit PointsCollection(
        std::function<void(const DualNeighborhood &)> dual_neighborhood_callback = [](const DualNeighborhood &) {}
    );

    void insert(Point3 point);
    template<Point3Range PR>
    void reset(PR new_points);
    VertexHandle move_vertex(VertexHandle vertex_handle, Point3 new_point);

    auto vertices() const;
    std::size_t size() const;
    auto points() const;
    auto dual_arcs() const;
    auto dual_neighborhoods();
    auto faces() const;
    auto dual_face_neighborhood(VertexHandle vertex_handle);
    std::vector<Arc> dual_cell_arcs(VertexHandle vertex_handle);

 private:
    Triangulation _triangulation;

    std::function<void(const DualNeighborhood &)> _dual_neighborhood_callback;

    auto all_edges() const;
    auto static vertex_circulator_range(VertexCirculator vertex_circulator);
    auto static edge_circulator_range(EdgeCirculator edge_circulator);
    auto static face_circulator_range(FaceCirculator face_circulator);
    auto incident_vertices_range(VertexHandleValue vertex_handle) const;
    auto incident_edges_range(VertexHandleValue vertex_handle) const;
    void clear();
};

inline PointsCollection::PointsCollection(
    std::function<void(const DualNeighborhood &)> dual_neighborhood_callback
) :
    _dual_neighborhood_callback(dual_neighborhood_callback) {
}

inline void PointsCollection::insert(Point3 point) {
    _triangulation.insert(point);
}

template<Point3Range PR>
inline void PointsCollection::reset(PR new_points) {
    clear();

    for (auto point : new_points) {
        insert(point);
    }
}

inline VertexHandle PointsCollection::move_vertex(VertexHandle vertex_handle, Point3 new_point) {
    _triangulation.remove(vertex_handle);
    return _triangulation.insert(new_point);
}

inline auto PointsCollection::vertex_circulator_range(VertexCirculator vertex_circulator) {
    auto begin = VertexCirculatorIterator(vertex_circulator);
    auto end = VertexCirculatorIterator(vertex_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto PointsCollection::edge_circulator_range(EdgeCirculator edge_circulator) {
    auto begin = EdgeCirculatorIterator(edge_circulator);
    auto end = EdgeCirculatorIterator(edge_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto PointsCollection::face_circulator_range(FaceCirculator face_circulator) {
    auto begin = FaceHandleCirculatorIterator(face_circulator);
    auto end = FaceHandleCirculatorIterator(face_circulator);

    return std::ranges::subrange(begin, end);
}

inline auto PointsCollection::incident_vertices_range(VertexHandle vertex_handle) const {
    return vertex_circulator_range(_triangulation.incident_vertices(vertex_handle));
}

inline auto PointsCollection::incident_edges_range(VertexHandle vertex_handle) const {
    return edge_circulator_range(_triangulation.incident_edges(vertex_handle));
}

inline auto PointsCollection::vertices() const {
    return std::ranges::subrange(
        VertexHandleIterator(_triangulation.finite_vertices_begin()),
        VertexHandleIterator(_triangulation.finite_vertices_end())
    );
}

inline std::size_t PointsCollection::size() const {
    return _triangulation.number_of_vertices();
}

inline auto PointsCollection::points() const {
    return vertices() | std::views::transform(
        [](VertexHandleValue vertex_handle) {
            return vertex_handle->point();
        }
    );
}

inline auto PointsCollection::all_edges() const {
    return std::ranges::subrange(
        _triangulation.all_edges_begin(),
        _triangulation.all_edges_end()
    );
}

inline auto PointsCollection::faces() const {
    return std::ranges::subrange(
        FaceHandleIterator(_triangulation.finite_faces_begin()),
        FaceHandleIterator(_triangulation.finite_faces_end())
    );
}

inline auto PointsCollection::dual_face_neighborhood(VertexHandle vertex_handle) {
    return incident_vertices_range(vertex_handle) | std::views::transform(
        [this](VertexHandleValue vertex_handle) {
            return dual_cell_arcs(vertex_handle);
        }
    );
}

inline void PointsCollection::clear() {
    _triangulation.clear();
}

inline auto PointsCollection::dual_arcs() const {
    return all_edges() | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );
}

inline auto PointsCollection::dual_neighborhoods() {
    return vertices() | std::views::transform(
        [this](VertexHandleValue vertex_handle) {
            auto dual_neighborhood = DualNeighborhood{
                .point = vertex_handle->point(),
                .dual_cell_arcs = dual_cell_arcs(vertex_handle)
            };

            _dual_neighborhood_callback(dual_neighborhood);

            return dual_neighborhood;
        }
    );
}

inline std::vector<Arc> PointsCollection::dual_cell_arcs(VertexHandle vertex_handle) {
    auto range = incident_edges_range(vertex_handle) | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );

    return {range.begin(), range.end()};
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_