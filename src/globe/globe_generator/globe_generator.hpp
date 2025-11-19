#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "../types.hpp"
#include "../geometry/helpers.hpp"
#include "../point_generator/point_generator.hpp"
#include "../point_generator/random_sphere_point_generator.hpp"
#include "../voronoi_sphere/voronoi_sphere.hpp"
#include "../scalar_field/scalar_field.hpp"
#include "../scalar_field/noise_field.hpp"
#include "spherical_polygon.hpp"
#include "sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "monte_carlo_integrator.hpp"
#include "../integrable_field/monte_carlo_integrable_field.hpp"
#include "../scalar_field/interval.hpp"
#include <queue>
#include <vector>
#include <utility>
#include <cstddef>
#include <iostream>
#include <atomic>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <dlib/optimization.h>

namespace globe {

const Interval DENSITY_FIELD_INTERVAL = Interval(1, 100);
const int POINT_COUNT = 10;

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
    ScalarField DF = NoiseField
>
class GlobeGenerator {
 public:
    GlobeGenerator(
        PG point_generator = PG(RandomSpherePointGenerator(1.0)),
        VoronoiSphere points_collection = VoronoiSphere(),
        DF density_field = DF(NoiseField())
    );

    VoronoiSphere generate(int point_count = POINT_COUNT);

 private:
    PG _point_generator;
    VoronoiSphere _points_collection;
    DF _density_field;
    MonteCarloIntegrableField<DF&> _integrable_field;

    void initialize();
    void add_points(int count);
    void normalize_density_field();
    void add_point();
    std::vector<Point3> sample_points(size_t n);
    double mass(const SphericalPolygon &spherical_polygon);
    double total_mass();
    double average_mass();
    double optimize_vertex_position(size_t index, double target_mass, double previous_error);
    void adjust_mass();
    std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator> build_cell_mass_heap();
    double compute_total_error(double target_mass);
};

template<PointGenerator PG, ScalarField DF>
GlobeGenerator<PG, DF>::GlobeGenerator(
    PG point_generator,
    VoronoiSphere points_collection,
    DF density_field
) :
    _point_generator(std::move(point_generator)),
    _points_collection(std::move(points_collection)),
    _density_field(std::move(density_field)),
    _integrable_field(_density_field) {
}

template<PointGenerator PG, ScalarField DF>
VoronoiSphere GlobeGenerator<PG, DF>::generate(int point_count) {
    initialize();
    add_points(point_count);
    adjust_mass();
    return std::move(_points_collection);
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::initialize() {
    normalize_density_field();
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::normalize_density_field() {
    _density_field.normalize(sample_points(1000), DENSITY_FIELD_INTERVAL);
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::add_points(int count) {
    for (int i = 0; i < count; i++) {
        add_point();
    }
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::adjust_mass() {
    std::cout << "Calculating total mass..." << std::endl;

    double target_mass = average_mass();
    std::cout << "Target mass per cell: " << target_mass << std::endl;

    const size_t max_passes = 10;

    for (size_t pass = 0; pass < max_passes; pass++) {
        std::cout << std::endl;
        std::cout << "=== Optimization pass " << pass + 1 << " / " << max_passes << " ===" << std::endl;

        auto heap = build_cell_mass_heap();
        size_t vertex_count = 0;
        double current_error = compute_total_error(target_mass);

        while (!heap.empty()) {
            size_t i = heap.top().index;
            double current_mass = heap.top().mass;
            heap.pop();

            current_error = optimize_vertex_position(i, target_mass, current_error);
            vertex_count++;
        }

        std::cout << "Optimized " << vertex_count << " vertices" << std::endl;
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

template<PointGenerator PG, ScalarField DF>
double GlobeGenerator<PG, DF>::mass(const SphericalPolygon &spherical_polygon) {
    return _integrable_field.integrate(spherical_polygon);
}

template<PointGenerator PG, ScalarField DF>
double GlobeGenerator<PG, DF>::total_mass() {
    auto bounding_box = SphericalBoundingBox(Interval(0, 2 * M_PI), Interval(-1, 1));
    auto generator = BoundingBoxSamplePointGenerator(bounding_box);

    return MonteCarloIntegrator<DF, BoundingBoxSamplePointGenerator>(
        std::nullopt,
        _density_field,
        std::move(generator)
    ).result().mass;
}

template<PointGenerator PG, ScalarField DF>
double GlobeGenerator<PG, DF>::average_mass() {
    return total_mass() / _points_collection.size();
}

template<PointGenerator PG, ScalarField DF>
double GlobeGenerator<PG, DF>::optimize_vertex_position(
    size_t index,
    double target_mass,
    double previous_error
) {
    Point3 current_position = _points_collection.site(index);
    Point3 north = antipodal(current_position);
    TangentBasis basis = build_tangent_basis(north);
    Point3 tangent_u = basis.tangent_u;
    Point3 tangent_v = basis.tangent_v;

    using column_vector = dlib::matrix<double, 2, 1>;
    int objective_call_count = 0;

    double initial_error = previous_error;

    auto objective = [&](const column_vector& params) -> double {
        objective_call_count++;
        Point3 candidate = stereographic_plane_to_sphere(params(0), params(1), current_position, tangent_u, tangent_v);
        _points_collection.update_site(index, candidate);
        double total_error = compute_total_error(target_mass);

        if (objective_call_count % 10 == 0) {
            std::cout <<
                "      " <<
                "Objective call " << objective_call_count <<
                ": total error = " << std::sqrt(total_error) <<
                std::endl;
        }

        return total_error;
    };

    column_vector starting_point;
    starting_point = 0.0, 0.0;

    std::cout <<
        "  Vertex " << index <<
        ": initial error = " << std::sqrt(initial_error) <<
        std::endl;

    try {
        dlib::find_min_bobyqa(
            objective,
            starting_point,
            5,
            dlib::uniform_matrix<double>(2, 1, -10.0),
            dlib::uniform_matrix<double>(2, 1, 10.0),
            0.5,
            1e-5,
            20
        );
    } catch (...) {
    }

    Point3 optimal_position = stereographic_plane_to_sphere(starting_point(0), starting_point(1), current_position, tangent_u, tangent_v);
    _points_collection.update_site(index, optimal_position);

    double final_error = compute_total_error(target_mass);

    std::cout <<
        "    " << objective_call_count << " calls" <<
        ", final error = " << std::sqrt(final_error) <<
        " (Δ = " << (std::sqrt(final_error) - std::sqrt(initial_error)) << ")" <<
        std::endl;

    return final_error;
}

template<PointGenerator PG, ScalarField DF>
std::vector<Point3> GlobeGenerator<PG, DF>::sample_points(size_t n) {
    std::vector<Point3> points;

    for (size_t i = 0; i < n; i++) {
        points.push_back(_point_generator.generate());
    }

    return points;
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::add_point() {
    Point3 point = _point_generator.generate();
    _points_collection.insert(point);
}

template<PointGenerator PG, ScalarField DF>
std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator> GlobeGenerator<PG, DF>::build_cell_mass_heap() {
    std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator> heap;

    size_t i = 0;
    for (const auto &cell : _points_collection.dual_cells()) {
        double cell_mass = mass(cell);
        heap.push({i, cell_mass});
        i++;
    }

    return heap;
}

template<PointGenerator PG, ScalarField DF>
double GlobeGenerator<PG, DF>::compute_total_error(double target_mass) {
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

GlobeGenerator() -> GlobeGenerator<RandomSpherePointGenerator, NoiseField>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_