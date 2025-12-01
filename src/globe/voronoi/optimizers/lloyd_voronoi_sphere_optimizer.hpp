#ifndef GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_LLOYD_VORONOI_SPHERE_OPTIMIZER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_LLOYD_VORONOI_SPHERE_OPTIMIZER_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../core/voronoi_sphere.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include <memory>
#include <cstddef>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace globe {

class LloydVoronoiSphereOptimizer {
 public:
    static constexpr size_t DEFAULT_PASSES = 4;
    static constexpr double CONVERGENCE_THRESHOLD = 1e-20;
    static constexpr size_t MAX_PASSES_WITHOUT_IMPROVEMENT = 5;

    LloydVoronoiSphereOptimizer(
        std::unique_ptr<VoronoiSphere> voronoi_sphere,
        size_t max_passes = DEFAULT_PASSES
    );

    std::unique_ptr<VoronoiSphere> optimize();

 private:
    std::unique_ptr<VoronoiSphere> _voronoi_sphere;
    size_t _max_passes;

    double run_single_pass();
    double compute_total_deviation() const;
};

inline LloydVoronoiSphereOptimizer::LloydVoronoiSphereOptimizer(
    std::unique_ptr<VoronoiSphere> voronoi_sphere,
    size_t max_passes
) :
    _voronoi_sphere(std::move(voronoi_sphere)),
    _max_passes(max_passes) {
}

inline std::unique_ptr<VoronoiSphere> LloydVoronoiSphereOptimizer::optimize() {
    std::cout << std::endl;
    std::cout << "=== Lloyd Relaxation ===" << std::endl;
    std::cout << "Max passes: " << _max_passes << std::endl;

    double initial_deviation = compute_total_deviation();
    std::cout << std::fixed << std::setprecision(6) <<
        "Initial deviation: " << initial_deviation << std::endl;
    std::cout << std::endl;

    double final_deviation = initial_deviation;
    double best_deviation = initial_deviation;
    size_t passes_completed = 0;
    size_t passes_without_improvement = 0;

    for (size_t pass = 0; pass < _max_passes; pass++) {
        double max_movement = run_single_pass();
        final_deviation = compute_total_deviation();
        passes_completed = pass + 1;

        std::cout << std::fixed << std::setprecision(6) <<
            "  Pass " << std::setw(3) << passes_completed <<
            ": deviation = " << std::setw(10) << final_deviation <<
            ", max_movement = " << std::setw(10) << max_movement <<
            std::defaultfloat << std::endl;

        if (max_movement < CONVERGENCE_THRESHOLD) {
            std::cout << "Converged." << std::endl;
            break;
        }

        if (final_deviation < best_deviation) {
            best_deviation = final_deviation;
            passes_without_improvement = 0;
        } else {
            passes_without_improvement++;
            if (passes_without_improvement >= MAX_PASSES_WITHOUT_IMPROVEMENT) {
                std::cout << "No improvement for " << MAX_PASSES_WITHOUT_IMPROVEMENT <<
                    " passes, stopping." << std::endl;
                break;
            }
        }
    }

    std::cout << std::endl;
    std::cout << std::fixed << std::setprecision(6) <<
        "Lloyd relaxation complete: " << passes_completed << " passes, " <<
        "deviation " << initial_deviation << " -> " << final_deviation <<
        std::defaultfloat << std::endl;

    return std::move(_voronoi_sphere);
}

inline double LloydVoronoiSphereOptimizer::run_single_pass() {
    double max_movement = 0.0;

    size_t index = 0;
    for (const auto &cell : _voronoi_sphere->cells()) {
        VectorS2 site = to_vector_s2(_voronoi_sphere->site(index));
        VectorS2 centroid = cell.centroid();

        double movement = distance(site, centroid);
        max_movement = std::max(max_movement, movement);

        _voronoi_sphere->update_site(index, cgal::to_point(centroid));
        index++;
    }

    return max_movement;
}

inline double LloydVoronoiSphereOptimizer::compute_total_deviation() const {
    double total = 0.0;

    size_t index = 0;
    for (const auto &cell : _voronoi_sphere->cells()) {
        VectorS2 site = to_vector_s2(_voronoi_sphere->site(index));
        VectorS2 centroid = cell.centroid();

        double deviation = distance(site, centroid);
        total += deviation * deviation;
        index++;
    }

    return std::sqrt(total / _voronoi_sphere->size());
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_LLOYD_VORONOI_SPHERE_OPTIMIZER_HPP_
