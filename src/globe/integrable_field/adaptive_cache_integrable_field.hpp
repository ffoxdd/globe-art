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

    CacheData compute_node_data(const Interval &theta_range, const Interval &z_range);
    bool is_fully_contained(const CacheQuadtree &node, const SphericalPolygon &polygon) const;
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
    ) {
    std::cout << std::endl;
    std::cout << "Initialization complete: " << _leaves_computed << " / " << _total_leaves << " leaf nodes cached" << std::endl;
}

template<ScalarField SF>
typename AdaptiveCacheIntegrableField<SF>::CacheData AdaptiveCacheIntegrableField<SF>::compute_node_data(
    const Interval &theta_range,
    const Interval &z_range
) {
    SphericalBoundingBox bbox(theta_range, z_range);

    MassCalculator<SF, BoundingBoxSamplePointGenerator> calculator(
        std::nullopt,
        _scalar_field,
        BoundingBoxSamplePointGenerator(bbox),
        1e-6,
        10,
        bbox
    );

    double cached_mass = calculator.mass();

    _leaves_computed++;
    double progress = 100.0 * _leaves_computed / _total_leaves;

    if (_leaves_computed % 1000 == 0 || _leaves_computed == _total_leaves) {
        std::cout << "\rCaching integrable field: " << progress << "% (" << _leaves_computed << " / " << _total_leaves << ")" << std::flush;
    }

    return CacheData{cached_mass};
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
double AdaptiveCacheIntegrableField<SF>::integrate_node(
    const CacheQuadtree &node,
    const SphericalPolygon &polygon
) {
    if (is_fully_contained(node, polygon)) {
        return node.data().cached_mass;
    }

    if (!node.is_leaf()) {
        double sum = 0.0;
        for (const auto &child : node.children()) {
            sum += integrate_node(*child, polygon);
        }
        return sum;
    }

    return 0.0;
}

template<ScalarField SF>
double AdaptiveCacheIntegrableField<SF>::integrate(const SphericalPolygon &polygon) {
    return integrate_node(_quadtree, polygon);
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_ADAPTIVE_CACHE_INTEGRABLE_FIELD_H_
