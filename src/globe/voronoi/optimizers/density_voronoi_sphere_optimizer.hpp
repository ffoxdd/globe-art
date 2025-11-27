#ifndef GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_DENSITY_VORONOI_SPHERE_OPTIMIZER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_DENSITY_VORONOI_SPHERE_OPTIMIZER_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../core/voronoi_sphere.hpp"
#include "../../generators/sphere_point_generator/random_sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/rejection_sampling_sphere_point_generator.hpp"
#include "../../fields/scalar/noise_field.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../fields/integrable/sampled_integrable_field.hpp"
#include "../../fields/integrable/integrable_field.hpp"
#include <algorithm>
#include <memory>
#include <queue>
#include <vector>
#include <utility>
#include <cstddef>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <dlib/optimization.h>
#include <optional>

namespace globe {

const int DEFAULT_POINT_COUNT = 10;
const size_t DEFAULT_OPTIMIZATION_PASSES = 10;
const double DISPLACEMENT_PENALTY_SCALE = 0.0;
const double ZERO_ERROR_TOLERANCE = 1e-4;
const double MIN_PERTURBATION_RADIANS = 0.025;
const size_t MAX_PERTURBATION_ATTEMPTS = 100;
const double CENTROID_DEVIATION_PENALTY = 2e-8;

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
    IntegrableField IntegrableFieldType = SampledIntegrableField<RejectionSamplingSpherePointGenerator<NoiseField>>,
    SpherePointGenerator GeneratorType = RandomSpherePointGenerator<>
>
class DensityVoronoiSphereOptimizer {
 public:
    DensityVoronoiSphereOptimizer(
        std::unique_ptr<VoronoiSphere> voronoi_sphere,
        std::unique_ptr<IntegrableFieldType> integrable_field,
        size_t optimization_passes = DEFAULT_OPTIMIZATION_PASSES,
        GeneratorType point_generator = GeneratorType()
    );

    std::unique_ptr<VoronoiSphere> optimize();

 private:
    std::unique_ptr<VoronoiSphere> _voronoi_sphere;
    std::unique_ptr<IntegrableFieldType> _integrable_field;
    size_t _optimization_passes;
    GeneratorType _point_generator;


    struct PassResult {
        double start_error;
        double end_error;
        size_t vertices_processed;
    };

    struct OptimizationState {
        size_t consecutive_stuck_passes = 0;
        double best_error = std::numeric_limits<double>::max();
        std::vector<Point3> best_checkpoint;
    };

    using CellMassHeap = std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator>;
    using Checkpoint = std::vector<Point3>;

    double mass(const SphericalPolygon &spherical_polygon);
    double total_mass();
    double average_mass();
    double optimize_vertex_position(size_t index, double target_mass, double previous_error);
    void adjust_mass(size_t max_passes);
    PassResult run_single_pass(double target_mass);
    bool check_progress_and_maybe_perturb(
        double target_mass,
        const PassResult &pass_result,
        OptimizationState &state
    );
    void print_final_results(double target_mass);
    CellMassHeap build_cell_mass_heap();
    double compute_convergence_error(double target_mass);
    double compute_objective_error(double target_mass);
    double compute_mass_error(double target_mass);
    double compute_centroid_deviation_error(double target_mass);
    bool perturb_most_undersized_vertex(double target_mass);
    std::optional<std::pair<size_t, double>> find_most_undersized_vertex_with_deficit(double target_mass);
    bool perturb_vertex_toward_random_point(size_t index);
    double perturbation_scale() const;
    Checkpoint save_checkpoint() const;
    void restore_checkpoint(const Checkpoint &checkpoint);
};

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::DensityVoronoiSphereOptimizer(
    std::unique_ptr<VoronoiSphere> voronoi_sphere,
    std::unique_ptr<IntegrableFieldType> integrable_field,
    size_t optimization_passes,
    GeneratorType point_generator
) :
    _voronoi_sphere(std::move(voronoi_sphere)),
    _integrable_field(std::move(integrable_field)),
    _optimization_passes(optimization_passes),
    _point_generator(std::move(point_generator)) {
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
std::unique_ptr<VoronoiSphere> DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::optimize() {
    adjust_mass(_optimization_passes);
    return std::move(_voronoi_sphere);
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
void DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::adjust_mass(size_t max_passes) {
    double target_mass = average_mass();
    std::cout << "Target mass per cell: " << target_mass << std::endl;

    OptimizationState state;
    state.best_checkpoint = save_checkpoint();

    for (size_t pass = 0; pass < max_passes; pass++) {
        std::cout << std::endl;
        std::cout << "=== Optimization pass " << pass + 1 << " / " << max_passes << " ===" << std::endl;

        PassResult result = run_single_pass(target_mass);

        if (!check_progress_and_maybe_perturb(target_mass, result, state)) {
            break;
        }
    }

    print_final_results(target_mass);
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
typename DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::PassResult
DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::run_single_pass(double target_mass) {
    auto heap = build_cell_mass_heap();
    double start_error = compute_convergence_error(target_mass);
    double current_error = start_error;
    size_t vertex_count = 0;

    while (!heap.empty()) {
        size_t index = heap.top().index;
        heap.pop();

        current_error = optimize_vertex_position(index, target_mass, current_error);
        vertex_count++;
    }

    std::cout << "Optimized " << vertex_count << " vertices" << std::endl;

    return {start_error, current_error, vertex_count};
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
bool DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::check_progress_and_maybe_perturb(
    double target_mass,
    const PassResult &pass_result,
    OptimizationState &state
) {
    double pass_improvement = pass_result.start_error - pass_result.end_error;
    bool made_meaningful_progress = pass_improvement > ZERO_ERROR_TOLERANCE * pass_result.start_error;

    if (made_meaningful_progress) {
        state.consecutive_stuck_passes = 0;

        if (pass_result.end_error < state.best_error) {
            state.best_error = pass_result.end_error;
            state.best_checkpoint = save_checkpoint();
        }

        return true;
    }

    state.consecutive_stuck_passes++;

    double rms_error = std::sqrt(pass_result.end_error / _voronoi_sphere->size());
    if (rms_error <= ZERO_ERROR_TOLERANCE * target_mass) {
        std::cout <<
            "RMS error " << rms_error <<
            " below threshold; stopping optimization." <<
            std::endl;
        return false;
    }

    if (state.consecutive_stuck_passes > MAX_PERTURBATION_ATTEMPTS) {
        std::cout <<
            "Exceeded " << MAX_PERTURBATION_ATTEMPTS <<
            " perturbation attempts; reverting to best checkpoint." <<
            std::endl;
        restore_checkpoint(state.best_checkpoint);
        return false;
    }

    std::cout <<
        "No meaningful error improvement (stuck " << state.consecutive_stuck_passes <<
        "x); perturbing to escape local minimum." << std::endl;

    if (!perturb_most_undersized_vertex(target_mass)) {
        std::cout << "No suitable vertex found for perturbation." << std::endl;
    }

    return true;
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
void DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::print_final_results(double target_mass) {
    std::cout << std::endl;
    std::cout << "Final cell masses after optimization:" << std::endl;

    double total_error = 0.0;
    double max_error = 0.0;

    size_t i = 0;
    for (const auto &cell : _voronoi_sphere->dual_cells()) {
        double cell_mass = mass(cell);
        double error = std::abs(cell_mass - target_mass);

        total_error += error;
        max_error = std::max(max_error, error);

        std::cout << "  Cell " << std::setw(4) << i << " mass: " << cell_mass << ", error: " << error << std::endl;
        i++;
    }

    std::cout << "Average error: " << (total_error / _voronoi_sphere->size()) << std::endl;
    std::cout << "Max error: " << max_error << std::endl;
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::mass(const SphericalPolygon &spherical_polygon) {
    return _integrable_field->integrate(spherical_polygon);
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::total_mass() {
    return _integrable_field->integrate();
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::average_mass() {
    return total_mass() / _voronoi_sphere->size();
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::optimize_vertex_position(
    size_t index,
    double target_mass,
    double previous_error
) {
    Point3 original_position = _voronoi_sphere->site(index);
    Point3 north = antipodal(original_position);
    TangentBasis basis = build_tangent_basis(to_position_vector(north));
    Vector3 tangent_u = basis.tangent_u;
    Vector3 tangent_v = basis.tangent_v;

    using column_vector = dlib::matrix<double, 2, 1>;

    double initial_error = previous_error;
    Vector3 original_vector = to_position_vector(original_position);

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
        _voronoi_sphere->update_site(index, original_position);

        auto objective = [&](const column_vector& params) -> double {
            Point3 candidate = stereographic_plane_to_sphere(
                params(0),
                params(1),
                original_vector,
                tangent_u,
                tangent_v
            );

            _voronoi_sphere->update_site(index, candidate);

            double optimization_error = compute_objective_error(target_mass);
            double displacement_angle = angular_distance(original_vector, to_position_vector(candidate));
            double penalty = DISPLACEMENT_PENALTY_SCALE * target_mass * target_mass * displacement_angle * displacement_angle;
            double cost = optimization_error + penalty;

            if (cost < run_best_cost) {
                run_best_cost = cost;
                run_best_mass_error = compute_convergence_error(target_mass);
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

        _voronoi_sphere->update_site(index, run_best_position);
        return RunResult{run_best_mass_error, run_best_cost, run_best_position};
    };

    column_vector origin;
    origin = 0.0, 0.0;

    auto initial_result = run_bobyqa(origin, 15);
    double final_error = initial_result.mass_error;
    Point3 best_position = initial_result.best_position;

    double initial_norm = std::sqrt(initial_error);
    double final_norm = std::sqrt(final_error);

    _voronoi_sphere->update_site(index, best_position);
    double delta = final_norm - initial_norm;
    double movement = angular_distance(original_vector, to_position_vector(best_position));

    std::cout <<
        "  Vertex " << std::setw(4) << index <<
        ": error = " << final_norm <<
        " (Δ = " << delta <<
        ", movement = " << movement <<
        ")" <<
        std::endl;

    return final_error;
}



template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
typename DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::CellMassHeap
DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::build_cell_mass_heap() {
    CellMassHeap heap;

    size_t i = 0;
    for (const auto &cell : _voronoi_sphere->dual_cells()) {
        double cell_mass = mass(cell);
        heap.push({i, cell_mass});
        i++;
    }

    return heap;
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::compute_convergence_error(double target_mass) {
    return compute_mass_error(target_mass);
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::compute_objective_error(double target_mass) {
    return compute_mass_error(target_mass) +
        CENTROID_DEVIATION_PENALTY * target_mass * target_mass * compute_centroid_deviation_error(target_mass);
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::compute_mass_error(double target_mass) {
    std::vector<SphericalPolygon> cells;
    cells.reserve(_voronoi_sphere->size());

    for (const auto &cell : _voronoi_sphere->dual_cells()) {
        cells.push_back(cell);
    }

    std::atomic<double> total_error{0.0};

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, cells.size()),
        [&](const tbb::blocked_range<size_t> &range) {
            double local_error = 0.0;

            for (size_t i = range.begin(); i != range.end(); ++i) {
                double cell_mass = mass(cells[i]);
                double mass_error = cell_mass - target_mass;
                local_error += mass_error * mass_error;
            }

            double current = total_error.load();
            while (!total_error.compare_exchange_weak(current, current + local_error));
        }
    );

    return total_error.load();
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::compute_centroid_deviation_error(double target_mass) {
    double total_error = 0.0;

    size_t i = 0;
    for (const auto &cell : _voronoi_sphere->dual_cells()) {
        Point3 site = _voronoi_sphere->site(i);
        Point3 centroid = cell.centroid();
        double deviation = angular_distance(
            to_position_vector(site),
            to_position_vector(centroid)
        );
        total_error += deviation * deviation;
        i++;
    }

    return total_error;
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
bool DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::perturb_most_undersized_vertex(
    double target_mass
) {
    auto candidate = find_most_undersized_vertex_with_deficit(target_mass);

    if (!candidate.has_value()) {
        return false;
    }

    auto [vertex_index, deficit] = candidate.value();

    if (!perturb_vertex_toward_random_point(vertex_index)) {
        return false;
    }

    std::cout << "  Perturbed vertex " << std::setw(4) << vertex_index << std::endl;

    return true;
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
std::optional<std::pair<size_t, double>>
DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::find_most_undersized_vertex_with_deficit(
    double target_mass
) {
    size_t best_index = _voronoi_sphere->size();
    double largest_deficit = 0.0;
    size_t i = 0;

    for (const auto &cell : _voronoi_sphere->dual_cells()) {
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

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
bool DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::perturb_vertex_toward_random_point(
    size_t index
) {
    if (index >= _voronoi_sphere->size()) {
        return false;
    }

    double angular_step = perturbation_scale();

    Point3 current_site = _voronoi_sphere->site(index);
    Vector3 current_vector = to_position_vector(current_site);

    Point3 random_point = _point_generator.generate(1)[0];
    Vector3 random_vector = to_position_vector(random_point);

    double dot = current_vector * random_vector;
    Vector3 tangent_component = random_vector - dot * current_vector;
    double tangent_length = std::sqrt(tangent_component.squared_length());

    if (tangent_length < 1e-10) {
        return false;
    }

    Vector3 tangent_direction = tangent_component / tangent_length;
    Vector3 new_vector =
        std::cos(angular_step) * current_vector +
        std::sin(angular_step) * tangent_direction;

    Point3 new_site(new_vector.x(), new_vector.y(), new_vector.z());
    _voronoi_sphere->update_site(index, new_site);

    return true;
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
double DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::perturbation_scale() const {
    double max_freq = _integrable_field->max_frequency();
    if (max_freq <= 0.0) {
        return MIN_PERTURBATION_RADIANS;
    }
    return 1.0 / (2.0 * max_freq);
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
typename DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::Checkpoint
DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::save_checkpoint() const {
    Checkpoint checkpoint;
    checkpoint.reserve(_voronoi_sphere->size());

    for (size_t i = 0; i < _voronoi_sphere->size(); i++) {
        checkpoint.push_back(_voronoi_sphere->site(i));
    }

    return checkpoint;
}

template<IntegrableField IntegrableFieldType, SpherePointGenerator GeneratorType>
void DensityVoronoiSphereOptimizer<IntegrableFieldType, GeneratorType>::restore_checkpoint(
    const Checkpoint &checkpoint
) {
    for (size_t i = 0; i < checkpoint.size(); i++) {
        _voronoi_sphere->update_site(i, checkpoint[i]);
    }
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_DENSITY_VORONOI_SPHERE_OPTIMIZER_HPP_