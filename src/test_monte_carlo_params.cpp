#include "globe/integrable_field/monte_carlo_integrable_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include "globe/scalar_field/constant_scalar_field.hpp"
#include "globe/point_generator/random_sphere_point_generator.hpp"
#include "globe/voronoi_sphere/voronoi_sphere.hpp"
#include "globe/globe_generator/monte_carlo_integrator.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip>

using namespace globe;

struct TestParams {
    double error_threshold;
    int consecutive_stable_iterations;
    size_t max_samples;
};

struct TestResult {
    double avg_time_ms;
    double avg_samples;
    double avg_variance;
    double max_error_vs_baseline;
};

TestResult test_params(
    const std::vector<SphericalPolygon> &polygons,
    const std::vector<double> &baseline_masses,
    const TestParams &params
) {
    ConstantScalarField field(1.0);

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> masses;
    std::vector<double> variances;
    std::vector<size_t> sample_counts;

    for (const auto &polygon : polygons) {
        SphericalBoundingBox bbox = polygon.bounding_box();

        MonteCarloIntegrator<ConstantScalarField, BoundingBoxSamplePointGenerator> calc(
            std::ref(polygon),
            field,
            BoundingBoxSamplePointGenerator(bbox),
            params.error_threshold,
            params.consecutive_stable_iterations,
            bbox,
            params.max_samples
        );

        auto result = calc.result();
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
    TestParams baseline_params{1e-9, 20, 10000000};
    std::vector<double> baseline_masses;

    for (const auto &polygon : test_polygons) {
        SphericalBoundingBox bbox = polygon.bounding_box();
        MonteCarloIntegrator<ConstantScalarField, BoundingBoxSamplePointGenerator> calc(
            std::ref(polygon),
            field,
            BoundingBoxSamplePointGenerator(bbox),
            baseline_params.error_threshold,
            baseline_params.consecutive_stable_iterations,
            bbox,
            baseline_params.max_samples
        );
        baseline_masses.push_back(calc.result().mass);
    }

    std::cout << std::endl;
    std::cout << "Testing different parameter combinations..." << std::endl;
    std::cout << std::endl;

    std::vector<TestParams> param_sets = {
        {1e-6, 10, 1000000},
        {1e-5, 10, 1000000},
        {1e-4, 10, 1000000},
        {1e-3, 10, 1000000},
        {1e-6, 5, 1000000},
        {1e-5, 5, 1000000},
        {1e-4, 5, 1000000},
        {1e-3, 5, 1000000},
        {1e-6, 3, 1000000},
        {1e-5, 3, 1000000},
        {1e-4, 3, 1000000},
        {1e-3, 3, 1000000},
    };

    std::cout << std::setw(12) << "Error Thr"
        << std::setw(12) << "Stable Its"
        << std::setw(12) << "Time (ms)"
        << std::setw(12) << "Samples"
        << std::setw(12) << "Max Error"
        << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    for (const auto &params : param_sets) {
        auto result = test_params(test_polygons, baseline_masses, params);

        std::cout << std::setw(12) << params.error_threshold
            << std::setw(12) << params.consecutive_stable_iterations
            << std::setw(12) << std::fixed << std::setprecision(2) << result.avg_time_ms
            << std::setw(12) << std::fixed << std::setprecision(0) << result.avg_samples
            << std::setw(12) << std::scientific << std::setprecision(2) << result.max_error_vs_baseline
            << std::endl;
    }

    return 0;
}
