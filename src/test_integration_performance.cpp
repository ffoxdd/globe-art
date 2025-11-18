#include "globe/integrable_field/monte_carlo_integrable_field.hpp"
#include "globe/integrable_field/variance_adaptive_integrable_field.hpp"
#include "globe/integrable_field/adaptive_cache_integrable_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include "globe/point_generator/random_sphere_point_generator.hpp"
#include "globe/points_collection/points_collection.hpp"
#include <iostream>
#include <chrono>
#include <vector>

using namespace globe;

int main() {
    std::cout << "Integration Performance Comparison Test\n" << std::endl;

    NoiseField noise_field;
    RandomSpherePointGenerator point_gen(1.0);

    std::cout << "Generating 10 test points for Voronoi diagram..." << std::endl;
    PointsCollection points;
    for (int i = 0; i < 10; i++) {
        points.insert(point_gen.generate());
    }

    std::cout << "Extracting test polygons..." << std::endl;
    std::vector<SphericalPolygon> test_polygons;
    for (const auto &vertex : points.vertices()) {
        auto arcs = points.dual_cell_arcs(vertex);
        if (!arcs.empty()) {
            test_polygons.emplace_back(arcs);
        }
    }
    std::cout << "Created " << test_polygons.size() << " test polygons\n" << std::endl;

    std::cout << "=== Monte Carlo (No Cache) ===" << std::endl;
    MonteCarloIntegrableField<NoiseField&> mc_field(noise_field);

    auto mc_start = std::chrono::high_resolution_clock::now();
    std::vector<double> mc_masses;
    for (const auto &polygon : test_polygons) {
        mc_masses.push_back(mc_field.integrate(polygon));
    }
    auto mc_end = std::chrono::high_resolution_clock::now();
    auto mc_duration = std::chrono::duration_cast<std::chrono::milliseconds>(mc_end - mc_start);

    std::cout << "Integrated " << test_polygons.size() << " polygons" << std::endl;
    std::cout << "Total time: " << mc_duration.count() << " ms" << std::endl;
    std::cout << "Average time per polygon: " << (mc_duration.count() / test_polygons.size()) << " ms\n" << std::endl;

    std::cout << "=== Adaptive Cache (Fixed Depth) ===" << std::endl;
    AdaptiveCacheIntegrableField<NoiseField&> cache_field(noise_field, 8);

    auto cache_start = std::chrono::high_resolution_clock::now();
    std::vector<double> cache_masses;
    for (const auto &polygon : test_polygons) {
        cache_masses.push_back(cache_field.integrate(polygon));
    }
    auto cache_end = std::chrono::high_resolution_clock::now();
    auto cache_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cache_end - cache_start);

    std::cout << "Integrated " << test_polygons.size() << " polygons" << std::endl;
    std::cout << "Total time: " << cache_duration.count() << " ms" << std::endl;
    std::cout << "Average time per polygon: " << (cache_duration.count() / test_polygons.size()) << " ms" << std::endl;
    cache_field.print_stats();
    std::cout << std::endl;

    std::cout << "=== Variance-Adaptive Quadtree ===" << std::endl;
    VarianceAdaptiveIntegrableField<NoiseField&> variance_field(noise_field);

    auto variance_start = std::chrono::high_resolution_clock::now();
    std::vector<double> variance_masses;
    for (const auto &polygon : test_polygons) {
        variance_masses.push_back(variance_field.integrate(polygon));
    }
    auto variance_end = std::chrono::high_resolution_clock::now();
    auto variance_duration = std::chrono::duration_cast<std::chrono::milliseconds>(variance_end - variance_start);

    std::cout << "Integrated " << test_polygons.size() << " polygons" << std::endl;
    std::cout << "Total time: " << variance_duration.count() << " ms" << std::endl;
    std::cout << "Average time per polygon: " << (variance_duration.count() / test_polygons.size()) << " ms" << std::endl;
    variance_field.print_stats();
    std::cout << std::endl;

    std::cout << "=== Comparison ===" << std::endl;
    std::cout << "Monte Carlo total: " << mc_duration.count() << " ms" << std::endl;
    std::cout << "Adaptive Cache total: " << cache_duration.count() << " ms" << std::endl;
    std::cout << "Variance-Adaptive total: " << variance_duration.count() << " ms" << std::endl;
    std::cout << "\nSpeedup vs Monte Carlo:" << std::endl;
    std::cout << "  Adaptive Cache: " << (static_cast<double>(mc_duration.count()) / cache_duration.count()) << "x" << std::endl;
    std::cout << "  Variance-Adaptive: " << (static_cast<double>(mc_duration.count()) / variance_duration.count()) << "x" << std::endl;

    std::cout << "\nMass comparison (vs Monte Carlo baseline):" << std::endl;
    double max_cache_error = 0.0;
    double max_variance_error = 0.0;
    for (size_t i = 0; i < test_polygons.size(); i++) {
        double cache_error = std::abs(mc_masses[i] - cache_masses[i]);
        double variance_error = std::abs(mc_masses[i] - variance_masses[i]);
        max_cache_error = std::max(max_cache_error, cache_error);
        max_variance_error = std::max(max_variance_error, variance_error);
        std::cout << "  Polygon " << i << ": MC=" << mc_masses[i]
                  << ", Cache=" << cache_masses[i] << " (err=" << cache_error << ")"
                  << ", Variance=" << variance_masses[i] << " (err=" << variance_error << ")"
                  << std::endl;
    }
    std::cout << "Max cache error: " << max_cache_error << std::endl;
    std::cout << "Max variance error: " << max_variance_error << std::endl;

    return 0;
}
