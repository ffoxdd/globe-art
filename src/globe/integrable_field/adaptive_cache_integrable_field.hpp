#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_ADAPTIVE_CACHE_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_ADAPTIVE_CACHE_INTEGRABLE_FIELD_H_

#include "integrable_field.hpp"
#include "quadtree.hpp"
#include "../scalar_field/scalar_field.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../globe_generator/spherical_bounding_box.hpp"
#include "../globe_generator/mass_calculator.hpp"
#include "../globe_generator/sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "../scalar_field/interval.hpp"
#include "../types.hpp"
#include <iostream>
#include <cmath>

namespace globe {

template<ScalarField SF>
class AdaptiveCacheIntegrableField {
 public:
    explicit AdaptiveCacheIntegrableField(SF scalar_field, size_t max_depth = 8);
    double integrate(const SphericalPolygon &polygon);
    void print_stats() const;

 private:
    struct CacheData {
        double cached_mass;
    };

    using CacheQuadtree = Quadtree<CacheData>;

    SF _scalar_field;
    size_t _max_depth;
    mutable size_t _leaves_computed;
    mutable size_t _total_leaves;
    CacheQuadtree _quadtree;

    mutable size_t _cache_hits;
    mutable size_t _subdivisions;
    mutable size_t _boundary_computations;

    CacheData compute_node_data(const Interval &theta_range, const Interval &z_range);
    void cache_parent_masses(CacheQuadtree &node);
    bool is_fully_contained(const CacheQuadtree &node, const SphericalPolygon &polygon) const;
    bool intersects_polygon(const CacheQuadtree &node, const SphericalPolygon &polygon) const;
    double integrate_node(const CacheQuadtree &node, const SphericalPolygon &polygon);
};

template<ScalarField SF>
AdaptiveCacheIntegrableField<SF>::AdaptiveCacheIntegrableField(SF scalar_field, size_t max_depth) :
    _scalar_field(scalar_field),
    _max_depth(max_depth),
    _leaves_computed(0),
    _total_leaves(std::pow(4, max_depth)),
    _quadtree(
        Interval(0, 2 * M_PI),
        Interval(-1, 1),
        max_depth,
        [this](const Interval &theta_range, const Interval &z_range) {
            return this->compute_node_data(theta_range, z_range);
        }
    ),
    _cache_hits(0),
    _subdivisions(0),
    _boundary_computations(0) {
    std::cout << std::endl;
    std::cout << "Caching parent node masses..." << std::flush;
    cache_parent_masses(_quadtree);
    std::cout << " done" << std::endl;
    std::cout << "Initialization complete: " << _leaves_computed << " / " << _total_leaves << " leaf nodes cached" << std::endl;
}

template<ScalarField SF>
typename AdaptiveCacheIntegrableField<SF>::CacheData AdaptiveCacheIntegrableField<SF>::compute_node_data(
    const Interval &theta_range,
    const Interval &z_range
) {
    _leaves_computed++;
    double progress = 100.0 * _leaves_computed / _total_leaves;

    if (_leaves_computed % 1000 == 0 || _leaves_computed == _total_leaves) {
        std::cout << "\rCaching integrable field: " << progress << "% (" << _leaves_computed << " / " << _total_leaves << ")" << std::flush;
    }

    SphericalBoundingBox bbox(theta_range, z_range);

    MassCalculator<SF, BoundingBoxSamplePointGenerator> calculator(
        std::nullopt,
        _scalar_field,
        BoundingBoxSamplePointGenerator(bbox),
        1e-6,
        10,
        bbox
    );

    return CacheData{calculator.mass()};
}

template<ScalarField SF>
void AdaptiveCacheIntegrableField<SF>::cache_parent_masses(CacheQuadtree &node) {
    if (node.is_leaf()) {
        return;
    }

    for (auto &child : node.children()) {
        cache_parent_masses(*child);
    }

    double total_mass = 0.0;
    for (const auto &child : node.children()) {
        total_mass += child->data().cached_mass;
    }

    node.data().cached_mass = total_mass;
}

template<ScalarField SF>
bool AdaptiveCacheIntegrableField<SF>::is_fully_contained(
    const CacheQuadtree &node,
    const SphericalPolygon &polygon
) const {
    const auto &theta_range = node.x_range();
    const auto &z_range = node.y_range();

    std::vector<double> theta_values = {theta_range.low(), theta_range.high()};
    std::vector<double> z_values = {z_range.low(), z_range.high()};

    for (double theta : theta_values) {
        for (double z : z_values) {
            double r = std::sqrt(1 - z * z);
            Point3 corner(r * std::cos(theta), r * std::sin(theta), z);
            if (!polygon.contains(corner)) {
                return false;
            }
        }
    }

    return true;
}

template<ScalarField SF>
bool AdaptiveCacheIntegrableField<SF>::intersects_polygon(
    const CacheQuadtree &node,
    const SphericalPolygon &polygon
) const {
    const auto &theta_range = node.x_range();
    const auto &z_range = node.y_range();

    std::vector<double> theta_values = {theta_range.low(), theta_range.high()};
    std::vector<double> z_values = {z_range.low(), z_range.high()};

    for (double theta : theta_values) {
        for (double z : z_values) {
            double r = std::sqrt(1 - z * z);
            Point3 corner(r * std::cos(theta), r * std::sin(theta), z);
            if (polygon.contains(corner)) {
                return true;
            }
        }
    }

    SphericalBoundingBox poly_bbox = polygon.bounding_box();
    return theta_range.overlaps(poly_bbox.theta_interval()) &&
           z_range.overlaps(poly_bbox.z_interval());
}

template<ScalarField SF>
double AdaptiveCacheIntegrableField<SF>::integrate_node(
    const CacheQuadtree &node,
    const SphericalPolygon &polygon
) {
    if (is_fully_contained(node, polygon)) {
        _cache_hits++;
        return node.data().cached_mass;
    }

    if (!intersects_polygon(node, polygon)) {
        return 0.0;
    }

    if (!node.is_leaf()) {
        _subdivisions++;
        double sum = 0.0;
        for (const auto &child : node.children()) {
            sum += integrate_node(*child, polygon);
        }
        return sum;
    }

    _boundary_computations++;
    SphericalBoundingBox bbox(node.x_range(), node.y_range());

    MassCalculator<SF, BoundingBoxSamplePointGenerator> calculator(
        std::ref(polygon),
        _scalar_field,
        BoundingBoxSamplePointGenerator(bbox),
        1e-3,
        2,
        bbox,
        50
    );

    return calculator.mass();
}

template<ScalarField SF>
double AdaptiveCacheIntegrableField<SF>::integrate(const SphericalPolygon &polygon) {
    _cache_hits = 0;
    _subdivisions = 0;
    _boundary_computations = 0;
    return integrate_node(_quadtree, polygon);
}

template<ScalarField SF>
void AdaptiveCacheIntegrableField<SF>::print_stats() const {
    size_t total = _cache_hits + _subdivisions + _boundary_computations;
    std::cout << "\nAdaptive cache statistics:" << std::endl;
    std::cout << "  Cache hits (fully contained): " << _cache_hits
              << " (" << (100.0 * _cache_hits / total) << "%)" << std::endl;
    std::cout << "  Subdivisions (non-leaf boundaries): " << _subdivisions
              << " (" << (100.0 * _subdivisions / total) << "%)" << std::endl;
    std::cout << "  Boundary computations (leaf boundaries): " << _boundary_computations
              << " (" << (100.0 * _boundary_computations / total) << "%)" << std::endl;
    std::cout << "  Total queries: " << total << std::endl;
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_ADAPTIVE_CACHE_INTEGRABLE_FIELD_H_
