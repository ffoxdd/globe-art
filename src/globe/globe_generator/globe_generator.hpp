#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "../types.hpp"
#include "../point_generator/point_generator.hpp"
#include "../point_generator/random_sphere_point_generator.hpp"
#include "../points_collection/voronoi_sphere.hpp"
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
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <dlib/optimization.h>

namespace globe {

const Interval DENSITY_FIELD_INTERVAL = Interval(1, 100);
const int POINT_COUNT = 10;

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
    void initialize();
    void add_points(int count = POINT_COUNT);

    auto dual_arcs();

 private:
    PG _point_generator;
    VoronoiSphere _points_collection;
    DF _density_field;
    MonteCarloIntegrableField<DF&> _integrable_field;

    void normalize_density_field();
    void calculate_target_mass();
    void add_point();
    std::vector<Point3> sample_points(size_t n);
    double mass(const SphericalPolygon &spherical_polygon);
    double total_mass();
    double average_mass();
    Point3 optimize_vertex_position(size_t index, double target_mass);
    void adjust_mass();
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
    calculate_target_mass();
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::normalize_density_field() {
    _density_field.normalize(sample_points(1000), DENSITY_FIELD_INTERVAL);
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::calculate_target_mass() {
    SphericalCircle3 circle(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0));

    SphericalPolygon spherical_polygon = SphericalPolygon(
        std::vector<Arc>{
            Arc(circle, SphericalPoint3(1, 0, 0), SphericalPoint3(0, 1, 0)),
            Arc(circle, SphericalPoint3(0, 1, 0), SphericalPoint3(0, 0, 1)),
            Arc(circle, SphericalPoint3(0, 0, 1), SphericalPoint3(1, 0, 0))
        }
    );


}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::add_points(int count) {
    for (int i = 0; i < count; i++) {
        add_point();
    }
}

struct VoronoiCell {
    size_t index;
    double mass{};
};

struct MinMassComparator {
    bool operator()(const VoronoiCell &a, const VoronoiCell &b) const {
        return a.mass > b.mass;
    }
};

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::adjust_mass() {
    std::cout << "Calculating total mass..." << std::endl;
    double target_mass = average_mass();
    std::cout << "Target mass per cell: " << target_mass << std::endl;

    const size_t max_passes = 2;
    for (size_t pass = 0; pass < max_passes; pass++) {
        std::cout << std::endl;
        std::cout << "=== Optimization pass " << pass + 1 << " / " << max_passes << " ===" << std::endl;

        std::cout << "Calculating cell masses..." << std::endl;
        std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator> heap;
        for (size_t i = 0; i < _points_collection.size(); i++) {
            double cell_mass = mass(SphericalPolygon(_points_collection.dual_cell_arcs(i)));
            std::cout << "  Cell " << i << " mass: " << cell_mass << std::endl;
            heap.push({i, cell_mass});
        }


        std::cout << "Optimizing vertices..." << std::endl;
        size_t vertex_count = 0;
        while (!heap.empty()) {
            size_t i = heap.top().index;
            double current_mass = heap.top().mass;
            heap.pop();

            std::cout << "  Optimizing vertex " << i << " (mass: " << current_mass << ")" << std::endl;
            optimize_vertex_position(i, target_mass);
            std::cout << "    Vertex " << i << " optimized" << std::endl;
            vertex_count++;
        }
        std::cout << "Optimized " << vertex_count << " vertices" << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Final cell masses after optimization:" << std::endl;
    double total_error = 0.0;
    double max_error = 0.0;
    for (size_t i = 0; i < _points_collection.size(); i++) {
        double cell_mass = mass(SphericalPolygon(_points_collection.dual_cell_arcs(i)));
        double error = std::abs(cell_mass - target_mass);
        total_error += error;
        max_error = std::max(max_error, error);
        std::cout << "  Cell " << i << " mass: " << cell_mass << ", error: " << error << std::endl;
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
Point3 GlobeGenerator<PG, DF>::optimize_vertex_position(size_t index, double target_mass) {
    Point3 current_position = _points_collection.site(index);

    Point3 north(current_position.x(), current_position.y(), current_position.z());
    Point3 south(-current_position.x(), -current_position.y(), -current_position.z());

    Point3 tangent_u, tangent_v;
    if (std::abs(north.z()) < 0.9) {
        tangent_u = Point3(north.y(), -north.x(), 0.0);
    } else {
        tangent_u = Point3(0.0, north.z(), -north.y());
    }

    double tu_len = std::sqrt(
        tangent_u.x() * tangent_u.x() +
        tangent_u.y() * tangent_u.y() +
        tangent_u.z() * tangent_u.z()
    );

    tangent_u = Point3(
        tangent_u.x() / tu_len,
        tangent_u.y() / tu_len,
        tangent_u.z() / tu_len
    );

    tangent_v = Point3(
        north.y() * tangent_u.z() - north.z() * tangent_u.y(),
        north.z() * tangent_u.x() - north.x() * tangent_u.z(),
        north.x() * tangent_u.y() - north.y() * tangent_u.x()
    );

    auto plane_to_sphere = [&](double u, double v) -> Point3 {
        double r_squared = u * u + v * v;
        double scale = 4.0 / (4.0 + r_squared);

        Point3 result(
            south.x() + scale * (u * tangent_u.x() + v * tangent_v.x() - south.x() * r_squared / 4.0),
            south.y() + scale * (u * tangent_u.y() + v * tangent_v.y() - south.y() * r_squared / 4.0),
            south.z() + scale * (u * tangent_u.z() + v * tangent_v.z() - south.z() * r_squared / 4.0)
        );

        double len = std::sqrt(
            result.x() * result.x() +
            result.y() * result.y() +
            result.z() * result.z()
        );

        return Point3(result.x() / len, result.y() / len, result.z() / len);
    };

    using column_vector = dlib::matrix<double, 2, 1>;

    int objective_call_count = 0;
    auto objective = [&](const column_vector& params) -> double {
        objective_call_count++;

        Point3 candidate = plane_to_sphere(params(0), params(1));

        _points_collection.update_site(index, candidate);
        double cell_mass = mass(SphericalPolygon(_points_collection.dual_cell_arcs(index)));
        double error = std::abs(cell_mass - target_mass);

        if (objective_call_count % 10 == 0) {
            std::cout << "      Objective call " << objective_call_count
                      << ": mass = " << cell_mass
                      << ", error = " << error << std::endl;
        }

        return error;
    };

    column_vector starting_point;
    starting_point = 0.0, 0.0;

    std::cout << "    Starting optimization (target mass: " << target_mass << ")" << std::endl;
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
    std::cout << "    Optimization completed after " << objective_call_count << " objective calls" << std::endl;

    Point3 optimal_position = plane_to_sphere(starting_point(0), starting_point(1));
    _points_collection.update_site(index, optimal_position);
    return optimal_position;
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
auto GlobeGenerator<PG, DF>::dual_arcs() {
    return _points_collection.dual_arcs();
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::add_point() {
    Point3 point = _point_generator.generate();
    _points_collection.insert(point);
}

GlobeGenerator() -> GlobeGenerator<RandomSpherePointGenerator, NoiseField>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_