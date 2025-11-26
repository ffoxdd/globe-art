#ifndef GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_POISSON_SPHERE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_POISSON_SPHERE_POINT_GENERATOR_HPP_

#include "indexed_kd_tree.hpp"
#include "../../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../../geometry/spherical/helpers.hpp"
#include "../sphere_point_generator.hpp"
#include "../random_sphere_point_generator.hpp"
#include <boost/iterator/counting_iterator.hpp>
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>

namespace globe {

template<SpherePointGenerator SpherePointGeneratorType = RandomSpherePointGenerator<>>
class PoissonSpherePointGenerator {
 public:
    PoissonSpherePointGenerator() = default;

    explicit PoissonSpherePointGenerator(
        SpherePointGeneratorType generator,
        double oversample_factor = 2.0
    ) : _generator(std::move(generator)),
        _oversample_factor(oversample_factor) {
    }

    std::vector<Point3> generate(size_t count);
    std::vector<Point3> generate(size_t count, const SphericalBoundingBox &bounding_box);

    [[nodiscard]] size_t last_attempt_count() const { return _last_attempt_count; }

 private:
    SpherePointGeneratorType _generator;
    double _oversample_factor = 2.0;
    size_t _last_attempt_count = 0;

    static constexpr double WEIGHT_ALPHA = 8.0;

    double compute_r_max(size_t target_count, double area) const;
    double weight_contribution(double geodesic_distance, double r_max) const;

    std::vector<Point3> eliminate_to_count(
        std::vector<Point3> &candidates,
        size_t target_count,
        double r_max
    );
};

template<SpherePointGenerator SpherePointGeneratorType>
std::vector<Point3> PoissonSpherePointGenerator<SpherePointGeneratorType>::generate(size_t count) {
    return generate(count, SphericalBoundingBox::full_sphere());
}

template<SpherePointGenerator SpherePointGeneratorType>
std::vector<Point3> PoissonSpherePointGenerator<SpherePointGeneratorType>::generate(
    size_t count,
    const SphericalBoundingBox &bounding_box
) {
    if (count == 0) {
        _last_attempt_count = 0;
        return {};
    }

    size_t oversample_count = static_cast<size_t>(count * _oversample_factor);
    oversample_count = std::max(oversample_count, count + 1);

    auto candidates = _generator.generate(oversample_count, bounding_box);
    _last_attempt_count = _generator.last_attempt_count();

    double area = bounding_box.area();
    double r_max = compute_r_max(count, area);

    return eliminate_to_count(candidates, count, r_max);
}

template<SpherePointGenerator SpherePointGeneratorType>
double PoissonSpherePointGenerator<SpherePointGeneratorType>::compute_r_max(
    size_t target_count,
    double area
) const {
    // For hexagonal packing on a sphere, area per point ≈ (sqrt(3)/2) * r^2
    // Solving for r: r = sqrt(2 * area / (sqrt(3) * n))
    // We use 2x this as r_max for the neighborhood radius
    double area_per_point = area / static_cast<double>(target_count);
    double r_optimal = std::sqrt(2.0 * area_per_point / std::sqrt(3.0));
    return 2.0 * r_optimal;
}

template<SpherePointGenerator SpherePointGeneratorType>
double PoissonSpherePointGenerator<SpherePointGeneratorType>::weight_contribution(
    double geodesic_distance,
    double r_max
) const {
    if (geodesic_distance >= r_max) {
        return 0.0;
    }
    double ratio = 1.0 - geodesic_distance / r_max;
    return std::pow(ratio, WEIGHT_ALPHA);
}

template<SpherePointGenerator SpherePointGeneratorType>
std::vector<Point3> PoissonSpherePointGenerator<SpherePointGeneratorType>::eliminate_to_count(
    std::vector<Point3> &candidates,
    size_t target_count,
    double r_max
) {
    size_t n = candidates.size();
    if (n <= target_count) {
        return candidates;
    }

    // Chord distance corresponding to geodesic r_max: 2*sin(r_max/2)
    double chord_r_max = 2.0 * std::sin(r_max / 2.0);

    // Build indexed KD-tree - stores indices, maps to points via property map
    IndexedPointMap<std::vector<Point3>> point_map(candidates);
    IndexedSearchTraits search_traits(point_map);

    IndexedKDTree tree(
        boost::counting_iterator<std::size_t>(0),
        boost::counting_iterator<std::size_t>(n),
        IndexedKDTree::Splitter(),
        search_traits
    );
    tree.build();

    // Track which points are still active
    std::vector<bool> active(n, true);
    size_t active_count = n;

    // Precompute neighbor lists - queries now return indices directly
    std::vector<std::vector<size_t>> neighbor_indices(n);

    for (size_t i = 0; i < n; ++i) {
        IndexedFuzzySphere query(candidates[i], chord_r_max, 0.0, search_traits);
        std::vector<size_t> neighbors;
        tree.search(std::back_inserter(neighbors), query);

        for (size_t j : neighbors) {
            if (j != i) {
                neighbor_indices[i].push_back(j);
            }
        }
    }

    // Calculate initial weights using geodesic distance
    std::vector<double> weights(n, 0.0);

    for (size_t i = 0; i < n; ++i) {
        Vector3 vi = to_position_vector(candidates[i]);
        for (size_t j : neighbor_indices[i]) {
            Vector3 vj = to_position_vector(candidates[j]);
            double geodesic = angular_distance(vi, vj);
            weights[i] += weight_contribution(geodesic, r_max);
        }
    }

    // Max-heap: (weight, index)
    using HeapEntry = std::pair<double, size_t>;
    std::priority_queue<HeapEntry> heap;

    for (size_t i = 0; i < n; ++i) {
        heap.emplace(weights[i], i);
    }

    // Eliminate points until we reach target count
    while (active_count > target_count && !heap.empty()) {
        auto [weight, idx] = heap.top();
        heap.pop();

        if (!active[idx]) {
            continue;
        }

        // Check if weight is stale (was updated after being pushed)
        if (std::abs(weight - weights[idx]) > 1e-10) {
            // Re-push with current weight
            heap.emplace(weights[idx], idx);
            continue;
        }

        // Eliminate this point
        active[idx] = false;
        active_count--;

        // Update weights of neighbors
        Vector3 vi = to_position_vector(candidates[idx]);

        for (size_t j : neighbor_indices[idx]) {
            if (!active[j]) continue;

            Vector3 vj = to_position_vector(candidates[j]);
            double geodesic = angular_distance(vi, vj);
            double contribution = weight_contribution(geodesic, r_max);
            weights[j] -= contribution;
            // Push updated weight (lazy update - old entry will be ignored)
            heap.emplace(weights[j], j);
        }
    }

    // Collect remaining points
    std::vector<Point3> result;
    result.reserve(target_count);

    for (size_t i = 0; i < n && result.size() < target_count; ++i) {
        if (active[i]) {
            result.push_back(candidates[i]);
        }
    }

    return result;
}

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_POISSON_SPHERE_POINT_GENERATOR_HPP_
