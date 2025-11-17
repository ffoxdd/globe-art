#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_ADAPTIVE_CACHE_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_ADAPTIVE_CACHE_INTEGRABLE_FIELD_H_

#include "integrable_field.hpp"
#include "quadtree.hpp"
#include "../scalar_field/scalar_field.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../scalar_field/interval.hpp"
#include <optional>

namespace globe {

template<ScalarField SF>
class AdaptiveCacheIntegrableField {
 public:
    explicit AdaptiveCacheIntegrableField(SF scalar_field, size_t max_depth = 8);
    double integrate(const SphericalPolygon &polygon);

 private:
    struct CacheData {
        std::optional<double> cached_mass;
    };

    using CacheQuadtree = Quadtree<CacheData>;

    SF _scalar_field;
    CacheQuadtree _quadtree;

    CacheData compute_node_data(const Interval &theta_range, const Interval &z_range);
    bool is_fully_contained(const CacheQuadtree &node, const SphericalPolygon &polygon) const;
    double integrate_node(const CacheQuadtree &node, const SphericalPolygon &polygon);
};

template<ScalarField SF>
AdaptiveCacheIntegrableField<SF>::AdaptiveCacheIntegrableField(SF scalar_field, size_t max_depth) :
    _scalar_field(scalar_field),
    _quadtree(
        Interval(0, 2 * M_PI),
        Interval(-1, 1),
        max_depth,
        [this](const Interval &theta_range, const Interval &z_range) {
            return this->compute_node_data(theta_range, z_range);
        }
    ) {
}

template<ScalarField SF>
typename AdaptiveCacheIntegrableField<SF>::CacheData AdaptiveCacheIntegrableField<SF>::compute_node_data(
    const Interval &theta_range,
    const Interval &z_range
) {
    return CacheData{std::nullopt};
}

template<ScalarField SF>
bool AdaptiveCacheIntegrableField<SF>::is_fully_contained(
    const CacheQuadtree &node,
    const SphericalPolygon &polygon
) const {
    return false;
}

template<ScalarField SF>
double AdaptiveCacheIntegrableField<SF>::integrate_node(
    const CacheQuadtree &node,
    const SphericalPolygon &polygon
) {
    if (node.data().cached_mass.has_value() && is_fully_contained(node, polygon)) {
        return node.data().cached_mass.value();
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
