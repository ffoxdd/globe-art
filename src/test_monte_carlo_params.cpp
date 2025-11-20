#include "globe/integrable_field/monte_carlo_integrable_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include "globe/scalar_field/constant_scalar_field.hpp"
#include "globe/point_generator/random_sphere_point_generator.hpp"
#include "globe/voronoi_sphere/voronoi_sphere.hpp"
#include "globe/globe_generator/monte_carlo_integrator.hpp"
#include "globe/globe_generator/monte_carlo_params.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip>

using namespace globe;

struct TestResult {
    double avg_time_ms;
    double avg_samples;
    double avg_variance;
    double max_error_vs_baseline;
};

TestResult test_params(
    const std::vector<SphericalPolygon> &polygons,
    const std::vector<double> &baseline_masses,
    const MonteCarloParams &params
) {
    ConstantScalarField field(1.0);

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> masses;
    std::vector<double> variances;
    std::vector<size_t> sample_counts;

    for (const auto &polygon : polygons) {
        SphericalBoundingBox bbox = polygon.bounding_box();
        MonteCarloIntegrator<ConstantScalarField, BoundingBoxSamplePointGenerator> calculator(
            std::ref(polygon),
            field,
            BoundingBoxSamplePointGenerator(),
            params,
            bbox
        );
        auto result = calculator.result();
        masses.push_back(result.mass);
        variances.push_back(result.variance);
        sample_counts.push_back(result.sample_count);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    double total_samples = 0;
    double total_variance = 0;
    double max_error = 0;

    for (size_t i = 0; i < polygons.size(); i++) {
        total_samples += sample_counts[i];
        total_variance += variances[i];
        double error = std::abs(masses[i] - baseline_masses[i]);
        max_error = std::max(max_error, error);
    }

    return TestResult{
        duration.count() / 1000.0 / polygons.size(),
        total_samples / polygons.size(),
        total_variance / polygons.size(),
        max_error
    };
}

int main() {
    std::cout << "Generating test Voronoi diagram with 10 points..." << std::endl;
    ConstantScalarField field(1.0);
    RandomSpherePointGenerator point_gen(1.0);
    VoronoiSphere points;

    for (int i = 0; i < 10; i++) {
        points.insert(point_gen.generate());
    }

    std::vector<SphericalPolygon> test_polygons;
    for (const auto &vertex : points.vertices()) {
        auto arcs = points.dual_cell_arcs(vertex);
        if (!arcs.empty()) {
            test_polygons.emplace_back(arcs);
        }
    }

    std::cout << "Computing baseline with very tight parameters..." << std::endl;
    MonteCarloParams baseline_params{20, 2000};
    std::vector<double> baseline_masses;

    {
        for (const auto &polygon : test_polygons) {
            SphericalBoundingBox bbox = polygon.bounding_box();
            MonteCarloIntegrator<ConstantScalarField, BoundingBoxSamplePointGenerator> calculator(
                std::ref(polygon),
                field,
                BoundingBoxSamplePointGenerator(),
                baseline_params,
                bbox
            );
            baseline_masses.push_back(calculator.result().mass);
        }
    }

    std::cout << std::endl;
    std::cout << "Testing different parameter combinations..." << std::endl;
    std::cout << std::endl;

    std::vector<MonteCarloParams> param_sets = {
        {10, 1000},
        {8, 500},
        {6, 400},
        {5, 250},
        {4, 200},
        {3, 150},
        {3, 100},
    };

    std::cout << std::setw(12) << "Stable Its"
        << std::setw(12) << "Min Hits"
        << std::setw(12) << "Time (ms)"
        << std::setw(12) << "Samples"
        << std::setw(12) << "Max Error"
        << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    for (const auto &params : param_sets) {
        auto result = test_params(test_polygons, baseline_masses, params);

        std::cout << std::setw(12) << params.consecutive_stable_iterations
            << std::setw(12) << params.min_hits
            << std::setw(12) << std::fixed << std::setprecision(2) << result.avg_time_ms
            << std::setw(12) << std::fixed << std::setprecision(0) << result.avg_samples
            << std::setw(12) << std::scientific << std::setprecision(2) << result.max_error_vs_baseline
            << std::endl;
    }

    return 0;
}
