#ifndef GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_OPTIMIZERS_LLOYD_OPTIMIZER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_OPTIMIZERS_LLOYD_OPTIMIZER_HPP_

#include "../../../types.hpp"
#include "../../../geometry/spherical/helpers.hpp"
#include "../core/sphere.hpp"
#include "../../../geometry/spherical/polygon/polygon.hpp"
#include <memory>
#include <cstddef>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace globe::voronoi::spherical {

using geometry::spherical::distance;

class LloydOptimizer {
 public:
    static constexpr size_t DEFAULT_PASSES = 4;
    static constexpr double CONVERGENCE_THRESHOLD = 1e-20;
    static constexpr size_t MAX_PASSES_WITHOUT_IMPROVEMENT = 5;

    LloydOptimizer(
        std::unique_ptr<Sphere> sphere,
        size_t max_passes = DEFAULT_PASSES
    );

    std::unique_ptr<Sphere> optimize();

 private:
    std::unique_ptr<Sphere> _sphere;
    size_t _max_passes;

    double run_single_pass();
    double compute_total_deviation() const;
};

inline LloydOptimizer::LloydOptimizer(
    std::unique_ptr<Sphere> sphere,
    size_t max_passes
) :
    _sphere(std::move(sphere)),
    _max_passes(max_passes) {
}

inline std::unique_ptr<Sphere> LloydOptimizer::optimize() {
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

    return std::move(_sphere);
}

inline double LloydOptimizer::run_single_pass() {
    double max_movement = 0.0;

    size_t index = 0;
    for (const auto &cell : _sphere->cells()) {
        VectorS2 site = to_vector_s2(_sphere->site(index));
        VectorS2 centroid = cell.centroid();

        double movement = distance(site, centroid);
        max_movement = std::max(max_movement, movement);

        _sphere->update_site(index, cgal::to_point(centroid));
        index++;
    }

    return max_movement;
}

inline double LloydOptimizer::compute_total_deviation() const {
    double total = 0.0;

    size_t index = 0;
    for (const auto &cell : _sphere->cells()) {
        VectorS2 site = to_vector_s2(_sphere->site(index));
        VectorS2 centroid = cell.centroid();

        double deviation = distance(site, centroid);
        total += deviation * deviation;
        index++;
    }

    return std::sqrt(total / _sphere->size());
}

} // namespace globe::voronoi::spherical

#endif //GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_OPTIMIZERS_LLOYD_OPTIMIZER_HPP_
