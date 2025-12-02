#ifndef GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_OPTIMIZERS_LLOYD_OPTIMIZER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_OPTIMIZERS_LLOYD_OPTIMIZER_HPP_

#include "../../../types.hpp"
#include "../../../geometry/spherical/helpers.hpp"
#include "../core/sphere.hpp"
#include "../core/progress_callback.hpp"
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
        size_t max_passes = DEFAULT_PASSES,
        ProgressCallback progress_callback = no_progress_callback()
    );

    std::unique_ptr<Sphere> optimize();
    double final_deviation() const;

 private:
    std::unique_ptr<Sphere> _sphere;
    size_t _max_passes;
    ProgressCallback _progress_callback;
    double _final_deviation = 0.0;

    double run_single_pass();
    double compute_total_deviation() const;
};

inline LloydOptimizer::LloydOptimizer(
    std::unique_ptr<Sphere> sphere,
    size_t max_passes,
    ProgressCallback progress_callback
) :
    _sphere(std::move(sphere)),
    _max_passes(max_passes),
    _progress_callback(std::move(progress_callback)) {
}

inline std::unique_ptr<Sphere> LloydOptimizer::optimize() {
    _final_deviation = compute_total_deviation();
    double best_deviation = _final_deviation;
    size_t passes_without_improvement = 0;

    for (size_t pass = 0; pass < _max_passes; pass++) {
        double max_movement = run_single_pass();
        _final_deviation = compute_total_deviation();

        _progress_callback(*_sphere);

        if (max_movement < CONVERGENCE_THRESHOLD) {
            break;
        }

        if (_final_deviation < best_deviation) {
            best_deviation = _final_deviation;
            passes_without_improvement = 0;
        } else {
            passes_without_improvement++;
            if (passes_without_improvement >= MAX_PASSES_WITHOUT_IMPROVEMENT) {
                break;
            }
        }
    }

    return std::move(_sphere);
}

inline double LloydOptimizer::final_deviation() const {
    return _final_deviation;
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
