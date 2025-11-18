#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_VARIANCE_ADAPTIVE_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_VARIANCE_ADAPTIVE_INTEGRABLE_FIELD_H_

#include "integrable_field.hpp"
#include "../scalar_field/scalar_field.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../globe_generator/spherical_bounding_box.hpp"
#include "../globe_generator/mass_calculator.hpp"
#include "../globe_generator/monte_carlo_result.hpp"
#include "../globe_generator/sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "../scalar_field/interval.hpp"
#include "../types.hpp"
#include <iostream>
#include <cmath>

namespace globe {

template<ScalarField SF>
class VarianceAdaptiveIntegrableField {
 public:
    explicit VarianceAdaptiveIntegrableField(
        SF scalar_field,
        double variance_threshold = 0.005,
        double min_mass_threshold = 0.001,
        size_t max_depth = 14
    );
    double integrate(const SphericalPolygon &polygon);
    void print_stats() const;

 private:
    struct Node {
        Interval theta_range;
        Interval z_range;
        double cached_mass;
        double variance;
        bool is_leaf;
        std::array<std::unique_ptr<Node>, 4> children;

        Node(const Interval &theta, const Interval &z, double mass, double var, bool leaf)
            : theta_range(theta), z_range(z), cached_mass(mass), variance(var), is_leaf(leaf) {}
    };

    SF _scalar_field;
    double _variance_threshold;
    double _min_mass_threshold;
    size_t _max_depth;
    mutable size_t _nodes_created;
    std::unique_ptr<Node> _root;

    mutable size_t _cache_hits;
    mutable size_t _subdivisions;
    mutable size_t _boundary_computations;

    std::unique_ptr<Node> build_tree(const Interval &theta_range, const Interval &z_range, size_t depth);
    bool should_subdivide(const MonteCarloResult &result, size_t depth) const;
    bool is_fully_contained(const Node &node, const SphericalPolygon &polygon) const;
    bool intersects_polygon(const Node &node, const SphericalPolygon &polygon) const;
    double integrate_node(const Node &node, const SphericalPolygon &polygon);
};

template<ScalarField SF>
VarianceAdaptiveIntegrableField<SF>::VarianceAdaptiveIntegrableField(
    SF scalar_field,
    double variance_threshold,
    double min_mass_threshold,
    size_t max_depth
) :
    _scalar_field(scalar_field),
    _variance_threshold(variance_threshold),
    _min_mass_threshold(min_mass_threshold),
    _max_depth(max_depth),
    _nodes_created(0),
    _cache_hits(0),
    _subdivisions(0),
    _boundary_computations(0) {
    std::cout << std::endl;
    std::cout << "Building variance-adaptive quadtree..." << std::flush;
    _root = build_tree(Interval(0, 2 * M_PI), Interval(-1, 1), 0);
    std::cout << " done" << std::endl;
    std::cout << "Created " << _nodes_created << " nodes" << std::endl;
}

template<ScalarField SF>
std::unique_ptr<typename VarianceAdaptiveIntegrableField<SF>::Node>
VarianceAdaptiveIntegrableField<SF>::build_tree(
    const Interval &theta_range,
    const Interval &z_range,
    size_t depth
) {
    _nodes_created++;

    if (_nodes_created % 1000 == 0) {
        std::cout << "\rCreated " << _nodes_created << " nodes" << std::flush;
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

    MonteCarloResult result = calculator.compute();
    bool is_leaf = !should_subdivide(result, depth);

    auto node = std::make_unique<Node>(
        theta_range,
        z_range,
        result.mass,
        result.variance,
        is_leaf
    );

    if (!is_leaf) {
        Interval theta_lower = theta_range.lower_half();
        Interval theta_upper = theta_range.upper_half();
        Interval z_lower = z_range.lower_half();
        Interval z_upper = z_range.upper_half();

        node->children[0] = build_tree(theta_lower, z_lower, depth + 1);
        node->children[1] = build_tree(theta_upper, z_lower, depth + 1);
        node->children[2] = build_tree(theta_lower, z_upper, depth + 1);
        node->children[3] = build_tree(theta_upper, z_upper, depth + 1);

        double total_mass = 0.0;
        for (const auto &child : node->children) {
            total_mass += child->cached_mass;
        }
        node->cached_mass = total_mass;
    }

    return node;
}

template<ScalarField SF>
bool VarianceAdaptiveIntegrableField<SF>::should_subdivide(
    const MonteCarloResult &result,
    size_t depth
) const {
    if (depth >= _max_depth) return false;
    if (result.mass < _min_mass_threshold) return false;
    if (result.variance < _variance_threshold) return false;
    return true;
}

template<ScalarField SF>
bool VarianceAdaptiveIntegrableField<SF>::is_fully_contained(
    const Node &node,
    const SphericalPolygon &polygon
) const {
    const auto &theta_range = node.theta_range;
    const auto &z_range = node.z_range;

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
bool VarianceAdaptiveIntegrableField<SF>::intersects_polygon(
    const Node &node,
    const SphericalPolygon &polygon
) const {
    const auto &theta_range = node.theta_range;
    const auto &z_range = node.z_range;

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
double VarianceAdaptiveIntegrableField<SF>::integrate_node(
    const Node &node,
    const SphericalPolygon &polygon
) {
    if (is_fully_contained(node, polygon)) {
        _cache_hits++;
        return node.cached_mass;
    }

    if (!intersects_polygon(node, polygon)) {
        return 0.0;
    }

    if (!node.is_leaf) {
        _subdivisions++;
        double sum = 0.0;
        for (const auto &child : node.children) {
            sum += integrate_node(*child, polygon);
        }
        return sum;
    }

    _boundary_computations++;
    SphericalBoundingBox bbox(node.theta_range, node.z_range);

    MassCalculator<SF, BoundingBoxSamplePointGenerator> calculator(
        std::ref(polygon),
        _scalar_field,
        BoundingBoxSamplePointGenerator(bbox),
        1e-4,
        3,
        bbox,
        200
    );

    return calculator.mass();
}

template<ScalarField SF>
double VarianceAdaptiveIntegrableField<SF>::integrate(const SphericalPolygon &polygon) {
    _cache_hits = 0;
    _subdivisions = 0;
    _boundary_computations = 0;
    return integrate_node(*_root, polygon);
}

template<ScalarField SF>
void VarianceAdaptiveIntegrableField<SF>::print_stats() const {
    size_t total = _cache_hits + _subdivisions + _boundary_computations;
    std::cout << "\nVariance-adaptive cache statistics:" << std::endl;
    std::cout << "  Total nodes created: " << _nodes_created << std::endl;
    std::cout << "  Cache hits (fully contained): " << _cache_hits
              << " (" << (100.0 * _cache_hits / total) << "%)" << std::endl;
    std::cout << "  Subdivisions (non-leaf boundaries): " << _subdivisions
              << " (" << (100.0 * _subdivisions / total) << "%)" << std::endl;
    std::cout << "  Boundary computations (leaf boundaries): " << _boundary_computations
              << " (" << (100.0 * _boundary_computations / total) << "%)" << std::endl;
    std::cout << "  Total queries: " << total << std::endl;
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_VARIANCE_ADAPTIVE_INTEGRABLE_FIELD_H_
