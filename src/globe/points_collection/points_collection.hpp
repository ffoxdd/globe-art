#ifndef GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_
#define GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_

#include "../types.hpp"
#include "types.hpp"
#include "handle_iterator.hpp"
#include "../geometry/helpers.hpp"
#include <ranges>
#include <algorithm>

namespace globe {

// TODO: consider what should go here and what should go in ./types.hpp
using VertexHandleValue = typename std::iterator_traits<FiniteVerticesIterator>::value_type::Vertex_handle;
using VertexHandleIterator = HandleIterator<FiniteVerticesIterator, VertexHandleValue>;
using FaceHandleValue = typename std::iterator_traits<FiniteFacesIterator>::value_type::Face_handle;
using FaceHandleIterator = HandleIterator<FiniteFacesIterator, FaceHandleValue>;
using FaceHandleCirculatorValue = typename std::iterator_traits<FaceCirculator>::value_type::Face_handle;
using FaceHandleCirculatorIterator = HandleIterator<FaceCirculator, FaceHandleCirculatorValue>;

struct DualNeighborhood {
    Point3 &point;
    std::vector<Point3> dual_cell_points;
};

class PointsCollection {
 public:
    struct Config;

    PointsCollection();
    explicit PointsCollection(Config &&config);

    PointsCollection(PointsCollection&&) = default;
    PointsCollection& operator=(PointsCollection&&) = default;

    void insert(Point3 point);
    bool empty() const;
    std::vector<Point3> nearby_points(Point3 point, double radius) const;

    auto dual_arcs() -> decltype(auto);
    auto dual_neighborhoods() -> decltype(auto);
    auto points() -> decltype(auto);
    auto faces() -> decltype(auto);
    std::vector<Point3> dual_cell_points(VertexHandle vertex_handle);

 private:
    std::vector<Point3> _points;
    KDTree _kd_tree;
    Triangulation _triangulation;

    std::function<void(const DualNeighborhood&)> _dual_neighborhood_callback;

    auto vertices() -> decltype(auto);
    auto edges() -> decltype(auto);
    auto all_edges() -> decltype(auto);
    auto segments() -> decltype(auto);
    auto static face_circulator_range(FaceCirculator face_circulator) -> decltype(auto);
    auto incident_faces_range(VertexHandleValue vertex_handle) -> decltype(auto);
};

struct PointsCollection::Config {
    std::function<void(const DualNeighborhood&)> dual_neighborhood_callback = [](const DualNeighborhood &) { };
};

PointsCollection::PointsCollection() : PointsCollection(Config()) {
}

PointsCollection::PointsCollection(Config &&config) :
    _dual_neighborhood_callback(config.dual_neighborhood_callback) {
}

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

auto PointsCollection::points() -> decltype(auto) {
    return std::ranges::subrange(_points.begin(), _points.end());
}

auto PointsCollection::vertices() -> decltype(auto) {
    return std::ranges::subrange(
        VertexHandleIterator(_triangulation.finite_vertices_begin()),
        VertexHandleIterator(_triangulation.finite_vertices_end())
    );
}

auto PointsCollection::edges() -> decltype(auto) {
    return std::ranges::subrange(
        _triangulation.finite_edges_begin(),
        _triangulation.finite_edges_end()
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

auto PointsCollection::dual_arcs() -> decltype(auto) {
    return all_edges() | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.dual_on_sphere(edge);
        }
    );
}

auto PointsCollection::segments() -> decltype(auto) {
    return edges() | std::views::transform(
        [this](Edge edge) {
            return this->_triangulation.segment(edge);
        }
    );
}

auto PointsCollection::dual_neighborhoods() -> decltype(auto) {
    return vertices() | std::views::transform(
        [this](VertexHandleValue vertex_handle) {
            auto dual_neighborhood = DualNeighborhood{
                .point = vertex_handle->point(),
                .dual_cell_points = dual_cell_points(vertex_handle)
            };

            _dual_neighborhood_callback(dual_neighborhood);

            return dual_neighborhood;
        }
    );
}

auto PointsCollection::face_circulator_range(FaceCirculator face_circulator) -> decltype(auto) {
    auto end = FaceHandleCirculatorIterator(face_circulator);
    auto begin = FaceHandleCirculatorIterator(++face_circulator);

    return std::ranges::subrange(begin, end);
}

auto PointsCollection::incident_faces_range(VertexHandle vertex_handle) -> decltype(auto) {
    return face_circulator_range(_triangulation.incident_faces(vertex_handle));
}

std::vector<Point3> PointsCollection::dual_cell_points(VertexHandle vertex_handle) {
    auto range = incident_faces_range(vertex_handle) | std::views::transform(
        [this](FaceHandleCirculatorValue face_handle) {
            return to_point(this->_triangulation.dual_on_sphere(face_handle));
        }
    );

    return {range.begin(), range.end()};
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_POINTS_COLLECTION_POINTS_COLLECTION_H_