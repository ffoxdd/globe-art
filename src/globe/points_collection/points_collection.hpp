#ifndef GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
#define GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_

#include "../types.hpp"
#include "types.hpp"
#include "handle_iterator.hpp"
#include "handle_circulator_iterator.hpp"
#include "circulator_iterator.hpp"
#include "../geometry/helpers.hpp"
#include <ranges>
#include <algorithm>

namespace globe {

// TODO: consider what should go here and what should go in ./types.hpp
using VertexHandleValue = typename std::iterator_traits<FiniteVerticesIterator>::value_type::Vertex_handle;
using VertexHandleIterator = HandleIterator<FiniteVerticesIterator, VertexHandleValue>;
using FaceHandleValue = typename std::iterator_traits<FiniteFacesIterator>::value_type::Face_handle;
using FaceHandleIterator = HandleIterator<FiniteFacesIterator, FaceHandleValue>;
using EdgeCirculatorIterator = CirculatorIterator<EdgeCirculator, Edge>;
using FaceHandleCirculatorValue = typename std::iterator_traits<FaceCirculator>::value_type::Face_handle;
using FaceHandleCirculatorIterator = HandleCirculatorIterator<FaceCirculator, FaceHandleCirculatorValue>;

class PointsCollection {
 public:
    struct Config;

    PointsCollection();
    explicit PointsCollection(Config &&config);

    void insert(Point3 point);
    template<Point3Range PR> void reset(PR new_points);
    void move_vertex(VertexHandle vertex_handle, Point3 new_point);

    auto vertices() -> decltype(auto);
    auto dual_arcs() -> decltype(auto);
    auto dual_neighborhoods() -> decltype(auto);
    auto faces() -> decltype(auto);

 private:
    Triangulation _triangulation;

    std::function<void(const DualNeighborhood &)> _dual_neighborhood_callback;

    auto all_edges() -> decltype(auto);
    auto static edge_circulator_range(EdgeCirculator edge_circulator) -> decltype(auto);
    auto static face_circulator_range(FaceCirculator face_circulator) -> decltype(auto);
    auto incident_edges_range(VertexHandleValue vertex_handle) -> decltype(auto);
    std::vector<Arc> dual_cell_arcs(VertexHandle vertex_handle);
    void clear();

};

struct PointsCollection::Config {
    std::function<void(const DualNeighborhood &)> dual_neighborhood_callback = [](const DualNeighborhood &) { };
};

PointsCollection::PointsCollection() :
    PointsCollection(Config()) {
}

PointsCollection::PointsCollection(Config &&config) :
    _dual_neighborhood_callback(config.dual_neighborhood_callback) {
}

void PointsCollection::insert(Point3 point) {
    _triangulation.insert(point);
}

template<Point3Range PR> void PointsCollection::reset(PR new_points) {
    clear();

    for (auto point : new_points) {
        insert(point);
    }
}

void PointsCollection::move_vertex(VertexHandle vertex_handle, Point3 new_point) {
    _triangulation.remove(vertex_handle);
    _triangulation.insert(new_point);
}

auto PointsCollection::vertices() -> decltype(auto) {
    return std::ranges::subrange(
        VertexHandleIterator(_triangulation.finite_vertices_begin()),
        VertexHandleIterator(_triangulation.finite_vertices_end())
    );
}

auto PointsCollection::all_edges() -> decltype(auto) {
    return std::ranges::subrange(
        _triangulation.all_edges_begin(),
        _triangulation.all_edges_end()
    );
}

auto PointsCollection::faces() -> decltype(auto) {
    return std::ranges::subrange(
        FaceHandleIterator(_triangulation.finite_faces_begin()),
        FaceHandleIterator(_triangulation.finite_faces_end())
    );
}

void PointsCollection::clear() {
    _triangulation.clear();
}

auto PointsCollection::dual_arcs() -> decltype(auto) {
    return all_edges() | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );
}

auto PointsCollection::dual_neighborhoods() -> decltype(auto) {
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

auto PointsCollection::edge_circulator_range(EdgeCirculator edge_circulator) -> decltype(auto) {
    auto begin = EdgeCirculatorIterator(edge_circulator);
    auto end = EdgeCirculatorIterator(edge_circulator);

    return std::ranges::subrange(begin, end);
}

auto PointsCollection::face_circulator_range(FaceCirculator face_circulator) -> decltype(auto) {
    auto begin = FaceHandleCirculatorIterator(face_circulator);
    auto end = FaceHandleCirculatorIterator(face_circulator);

    return std::ranges::subrange(begin, end);
}

auto PointsCollection::incident_edges_range(VertexHandle vertex_handle) -> decltype(auto) {
    return edge_circulator_range(_triangulation.incident_edges(vertex_handle));
}

std::vector<Arc> PointsCollection::dual_cell_arcs(VertexHandle vertex_handle) {
    auto range = incident_edges_range(vertex_handle) | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );

    return {range.begin(), range.end()};
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_