#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "../types.hpp"
#include "../geometry/helpers.hpp"
#include "../point_generator/point_generator.hpp"
#include "../point_generator/random_sphere_point_generator.hpp"
#include "../voronoi_sphere/voronoi_sphere.hpp"
#include "../scalar_field/noise_field.hpp"
#include "spherical_polygon.hpp"
#include "../integrable_field/density_sampled_integrable_field.hpp"
#include "../scalar_field/interval.hpp"
#include <algorithm>
#include <memory>
#include <queue>
#include <vector>
#include <utility>
#include <cstddef>
#include <iostream>
#include <atomic>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <dlib/optimization.h>
#include <optional>

namespace globe {

const Interval DENSITY_FIELD_INTERVAL = Interval(1, 100);
const int DEFAULT_POINT_COUNT = 10;
const size_t DEFAULT_OPTIMIZATION_PASSES = 10;
const double DISPLACEMENT_PENALTY_SCALE = 0.05;
const double MOVEMENT_EPSILON = 1e-4;
const double ZERO_ERROR_TOLERANCE = 1e-3;
const double MAX_PERTURBATION_STEP = 0.15;
const size_t MIN_SAMPLE_COUNT = 60'000;
const size_t SAMPLES_PER_POINT = 3'000;

struct VoronoiCell {
    size_t index;
    double mass{};
};

struct MinMassComparator {
    bool operator()(const VoronoiCell &a, const VoronoiCell &b) const {
        return a.mass > b.mass;
    }
};

template<
    PointGenerator PG = RandomSpherePointGenerator,
    typename IntegrableFieldType = DensitySampledIntegrableField<NoiseField>
>
class GlobeGenerator {
 public:
    GlobeGenerator(
        PG point_generator = PG(RandomSpherePointGenerator(1.0)),
        VoronoiSphere points_collection = VoronoiSphere(),
        std::unique_ptr<IntegrableFieldType> integrable_field = nullptr
    );

    VoronoiSphere generate(
        int point_count = DEFAULT_POINT_COUNT,
        size_t optimization_passes = DEFAULT_OPTIMIZATION_PASSES
    );

 private:
    PG _point_generator;
    VoronoiSphere _points_collection;
    std::unique_ptr<IntegrableFieldType> _integrable_field;

    void initialize();
    void add_points(int count);
    void add_point();
    std::vector<Point3> sample_points(size_t n);
    double mass(const SphericalPolygon &spherical_polygon);
    double total_mass();
    double average_mass();
    struct OptimizationResult {
        double error;
        bool moved;
    };

    OptimizationResult optimize_vertex_position(size_t index, double target_mass, double previous_error);
    void adjust_mass(size_t max_passes);
    std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator> build_cell_mass_heap();
    double compute_total_error(double target_mass);
    bool perturb_most_undersized_vertex(double target_mass);
    std::optional<std::pair<size_t, double>> find_most_undersized_vertex_with_deficit(double target_mass);
    bool perturb_vertex_toward_random_point(size_t index, double deficit, double target_mass);
};

template<PointGenerator PG, typename IntegrableFieldType>
GlobeGenerator<PG, IntegrableFieldType>::GlobeGenerator(
    PG point_generator,
    VoronoiSphere points_collection,
    std::unique_ptr<IntegrableFieldType> integrable_field
) :
    _point_generator(std::move(point_generator)),
    _points_collection(std::move(points_collection)),
    _integrable_field(std::move(integrable_field)) {
}

template<PointGenerator PG, typename IntegrableFieldType>
VoronoiSphere GlobeGenerator<PG, IntegrableFieldType>::generate(
    int point_count,
    size_t optimization_passes
) {
    initialize();
    add_points(point_count);
    adjust_mass(optimization_passes);
    return std::move(_points_collection);
}

template<PointGenerator PG, typename IntegrableFieldType>
void GlobeGenerator<PG, IntegrableFieldType>::initialize() {
}

template<PointGenerator PG, typename IntegrableFieldType>
void GlobeGenerator<PG, IntegrableFieldType>::add_points(int count) {
    for (int i = 0; i < count; i++) {
        add_point();
    }
}

template<PointGenerator PG, typename IntegrableFieldType>
void GlobeGenerator<PG, IntegrableFieldType>::adjust_mass(size_t max_passes) {
    std::cout <<
        "Using integrable field (" <<
        _integrable_field->sample_count() <<
        " samples)" << std::endl;

    double target_mass = average_mass();
    std::cout << "Target mass per cell: " << target_mass << std::endl;

    for (size_t pass = 0; pass < max_passes; pass++) {
        std::cout << std::endl;
        std::cout << "=== Optimization pass " << pass + 1 << " / " << max_passes << " ===" << std::endl;

        auto heap = build_cell_mass_heap();
        size_t vertex_count = 0;
        double current_error = compute_total_error(target_mass);
        bool pass_made_progress = false;

        while (!heap.empty()) {
            size_t i = heap.top().index;
            heap.pop();

            OptimizationResult result = optimize_vertex_position(i, target_mass, current_error);
            current_error = result.error;
            pass_made_progress = pass_made_progress || result.moved;
            vertex_count++;
        }

        std::cout << "Optimized " << vertex_count << " vertices" << std::endl;

        if (!pass_made_progress) {
            double rms_error = std::sqrt(current_error / _points_collection.size());
            if (rms_error <= ZERO_ERROR_TOLERANCE * target_mass) {
                std::cout <<
                    "No vertex movement and RMS error " << rms_error <<
                    " below threshold; stopping optimization." <<
                    std::endl;
                break;
            }

            std::cout <<
                "No vertex movement detected; perturbing undersized vertex to escape local minimum."
                << std::endl;

            if (!perturb_most_undersized_vertex(target_mass)) {
                std::cout << "No suitable vertex found for perturbation." << std::endl;
            }
        }
    }

    std::cout << std::endl;
    std::cout << "Final cell masses after optimization:" << std::endl;

    double total_error = 0.0;
    double max_error = 0.0;

    size_t i = 0;
    for (const auto &cell : _points_collection.dual_cells()) {
        double cell_mass = mass(cell);
        double error = std::abs(cell_mass - target_mass);

        total_error += error;
        max_error = std::max(max_error, error);

        std::cout << "  Cell " << i << " mass: " << cell_mass << ", error: " << error << std::endl;
        i++;
    }

    std::cout << "Average error: " << (total_error / _points_collection.size()) << std::endl;
    std::cout << "Max error: " << max_error << std::endl;
}

template<PointGenerator PG, typename IntegrableFieldType>
double GlobeGenerator<PG, IntegrableFieldType>::mass(const SphericalPolygon &spherical_polygon) {
    return _integrable_field->integrate(spherical_polygon);
}

template<PointGenerator PG, typename IntegrableFieldType>
double GlobeGenerator<PG, IntegrableFieldType>::total_mass() {
    return _integrable_field->integrate_entire_sphere();
}

template<PointGenerator PG, typename IntegrableFieldType>
double GlobeGenerator<PG, IntegrableFieldType>::average_mass() {
    return total_mass() / _points_collection.size();
}

template<PointGenerator PG, typename IntegrableFieldType>
typename GlobeGenerator<PG, IntegrableFieldType>::OptimizationResult GlobeGenerator<PG, IntegrableFieldType>::optimize_vertex_position(
    size_t index,
    double target_mass,
    double previous_error
) {
    Point3 original_position = _points_collection.site(index);
    Point3 north = antipodal(original_position);
    TangentBasis basis = build_tangent_basis(north);
    Point3 tangent_u = basis.tangent_u;
    Point3 tangent_v = basis.tangent_v;

    using column_vector = dlib::matrix<double, 2, 1>;

    double initial_error = previous_error;
    Vector3 original_vector = original_position - ORIGIN;

    auto run_bobyqa = [&](const column_vector& initial_point, int max_function_calls) {
        struct RunResult {
            double mass_error;
            double cost;
            Point3 best_position;
        };

        Point3 run_best_position = original_position;
        double run_best_mass_error = initial_error;
        double run_best_cost = initial_error;
        column_vector starting_point = initial_point;
        _points_collection.update_site(index, original_position);

        auto objective = [&](const column_vector& params) -> double {
            Point3 candidate = stereographic_plane_to_sphere(
                params(0),
                params(1),
                original_position,
                tangent_u,
                tangent_v
            );

            _points_collection.update_site(index, candidate);

            double total_error = compute_total_error(target_mass);
            double displacement_angle = angular_distance(original_vector, candidate - ORIGIN);
            double penalty = DISPLACEMENT_PENALTY_SCALE * target_mass * target_mass * displacement_angle * displacement_angle;
            double cost = total_error + penalty;

            if (cost < run_best_cost) {
                run_best_cost = cost;
                run_best_mass_error = total_error;
                run_best_position = candidate;
            }

            return cost;
        };

        try {
            dlib::find_min_bobyqa(
                objective,
                starting_point,
                5,
                dlib::uniform_matrix<double>(2, 1, -10.0),
                dlib::uniform_matrix<double>(2, 1, 10.0),
                0.5,
                1e-5,
                max_function_calls
            );
        } catch (...) {
        }

        _points_collection.update_site(index, run_best_position);
        return RunResult{run_best_mass_error, run_best_cost, run_best_position};
    };

    column_vector origin;
    origin = 0.0, 0.0;

    auto initial_result = run_bobyqa(origin, 6);
    double final_error = initial_result.mass_error;
    Point3 best_position = initial_result.best_position;

    double initial_norm = std::sqrt(initial_error);
    double final_norm = std::sqrt(final_error);

    _points_collection.update_site(index, best_position);
    double delta = final_norm - initial_norm;
    bool moved = angular_distance(original_vector, best_position - ORIGIN) > MOVEMENT_EPSILON;

    std::cout <<
        "  Vertex " << index <<
        ": error = " << final_norm <<
        " (Δ = " << delta << ")" <<
        std::endl;

    return {final_error, moved};
}

template<PointGenerator PG, typename IntegrableFieldType>
std::vector<Point3> GlobeGenerator<PG, IntegrableFieldType>::sample_points(size_t n) {
    std::vector<Point3> points;

    for (size_t i = 0; i < n; i++) {
        points.push_back(_point_generator.generate());
    }

    return points;
}

template<PointGenerator PG, typename IntegrableFieldType>
void GlobeGenerator<PG, IntegrableFieldType>::add_point() {
    Point3 point = _point_generator.generate();
    _points_collection.insert(point);
}

template<PointGenerator PG, typename IntegrableFieldType>
std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator> GlobeGenerator<PG, IntegrableFieldType>::build_cell_mass_heap() {
    std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator> heap;

    size_t i = 0;
    for (const auto &cell : _points_collection.dual_cells()) {
        double cell_mass = mass(cell);
        heap.push({i, cell_mass});
        i++;
    }

    return heap;
}

template<PointGenerator PG, typename IntegrableFieldType>
double GlobeGenerator<PG, IntegrableFieldType>::compute_total_error(double target_mass) {
    std::vector<SphericalPolygon> cells;
    cells.reserve(_points_collection.size());

    for (const auto &cell : _points_collection.dual_cells()) {
        cells.push_back(cell);
    }

    std::atomic<double> total_error{0.0};

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, cells.size()),
        [&](const tbb::blocked_range<size_t> &range) {
            double local_error = 0.0;

            for (size_t i = range.begin(); i != range.end(); ++i) {
                double cell_mass = mass(cells[i]);
                double error = cell_mass - target_mass;
                local_error += error * error;
            }

            double current = total_error.load();
            while (!total_error.compare_exchange_weak(current, current + local_error));
        }
    );

    return total_error.load();
}

template<PointGenerator PG, typename IntegrableFieldType>
bool GlobeGenerator<PG, IntegrableFieldType>::perturb_most_undersized_vertex(double target_mass) {
    auto candidate = find_most_undersized_vertex_with_deficit(target_mass);

    if (!candidate.has_value()) {
        return false;
    }

    auto [vertex_index, deficit] = candidate.value();

    if (!perturb_vertex_toward_random_point(vertex_index, deficit, target_mass)) {
        return false;
    }

    std::cout <<
        "  Perturbed vertex " << vertex_index <<
        " toward random direction to escape local minimum." <<
        std::endl;

    return true;
}

template<PointGenerator PG, typename IntegrableFieldType>
std::optional<std::pair<size_t, double>> GlobeGenerator<PG, IntegrableFieldType>::find_most_undersized_vertex_with_deficit(double target_mass) {
    size_t best_index = _points_collection.size();
    double largest_deficit = 0.0;
    size_t i = 0;

    for (const auto &cell : _points_collection.dual_cells()) {
        double cell_mass = mass(cell);
        double deficit = target_mass - cell_mass;

        if (deficit > largest_deficit) {
            largest_deficit = deficit;
            best_index = i;
        }

        i++;
    }

    if (largest_deficit <= 0.0) {
        return std::nullopt;
    }

    return std::make_pair(best_index, largest_deficit);
}

template<PointGenerator PG, typename IntegrableFieldType>
bool GlobeGenerator<PG, IntegrableFieldType>::perturb_vertex_toward_random_point(size_t index, double deficit, double target_mass) {
    if (index >= _points_collection.size()) {
        return false;
    }

    double deficit_ratio = std::clamp(deficit / target_mass, 0.0, 1.0);
    double step = MAX_PERTURBATION_STEP * deficit_ratio;

    Point3 current_site = _points_collection.site(index);
    Point3 random_point = _point_generator.generate();
    Point3 perturbed = spherical_interpolate(current_site, random_point, step);
    Point3 normalized_point = project_to_sphere(perturbed);
    _points_collection.update_site(index, normalized_point);

    return true;
}

GlobeGenerator() -> GlobeGenerator<RandomSpherePointGenerator, DensitySampledIntegrableField<NoiseField>>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_