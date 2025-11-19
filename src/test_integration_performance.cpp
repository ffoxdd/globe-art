#include "globe/integrable_field/monte_carlo_integrable_field.hpp"
#include "globe/integrable_field/grid_sample_integrable_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include "globe/point_generator/random_sphere_point_generator.hpp"
#include "globe/voronoi_sphere/voronoi_sphere.hpp"
#include <iostream>
#include <chrono>
#include <vector>

using namespace globe;

int main() {
    std::cout << "Integration Performance Comparison Test\n" << std::endl;

    NoiseField noise_field;
    RandomSpherePointGenerator point_gen(1.0);

    std::cout << "Generating 10 test points for Voronoi diagram..." << std::endl;
    VoronoiSphere points;
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
    std::cout << "Average time per polygon: " <<
        (mc_duration.count() / test_polygons.size()) << " ms\n" << std::endl;

    std::cout << "=== Grid Sample (Fibonacci Sphere, 10k samples) ===" << std::endl;
    GridSampleIntegrableField<NoiseField&> grid_field(noise_field, 10000);

    auto grid_start = std::chrono::high_resolution_clock::now();
    std::vector<double> grid_masses;
    for (const auto &polygon : test_polygons) {
        grid_masses.push_back(grid_field.integrate(polygon));
    }
    auto grid_end = std::chrono::high_resolution_clock::now();
    auto grid_duration = std::chrono::duration_cast<std::chrono::milliseconds>(grid_end - grid_start);

    std::cout << "Integrated " << test_polygons.size() << " polygons" << std::endl;
    std::cout << "Total time: " << grid_duration.count() << " ms" << std::endl;
    std::cout << "Average time per polygon: " <<
        (grid_duration.count() / test_polygons.size()) << " ms" << std::endl;
    grid_field.print_stats();
    std::cout << std::endl;

    std::cout << "=== Comparison ===" << std::endl;
    std::cout << "Monte Carlo total: " << mc_duration.count() << " ms" << std::endl;
    std::cout << "Grid Sample total: " << grid_duration.count() << " ms" << std::endl;
    std::cout << "\nSpeedup vs Monte Carlo:" << std::endl;
    std::cout << "  Grid Sample: " <<
        (static_cast<double>(mc_duration.count()) / grid_duration.count()) << "x" << std::endl;

    std::cout << "\nMass comparison (vs Monte Carlo baseline):" << std::endl;
    double max_grid_error = 0.0;
    for (size_t i = 0; i < test_polygons.size(); i++) {
        double grid_error = std::abs(mc_masses[i] - grid_masses[i]);
        max_grid_error = std::max(max_grid_error, grid_error);
        std::cout << "  Polygon " << i << ": MC=" << mc_masses[i] <<
            ", Grid=" << grid_masses[i] << " (err=" << grid_error << ")" <<
            std::endl;
    }
    std::cout << "Max grid error: " << max_grid_error << std::endl;

    return 0;
}
