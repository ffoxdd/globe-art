#ifndef GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"
#include "../../fields/spherical/spherical_field.hpp"
#include "../../fields/spherical/constant_spherical_field.hpp"
#include "../../generators/sphere_point_generator/sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/random_sphere_point_generator.hpp"
#include "../core/voronoi_sphere.hpp"
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <optional>

namespace globe {

template<
    SphericalField FieldType = ConstantSphericalField,
    SpherePointGenerator GeneratorType = RandomSpherePointGenerator<>
>
class GradientDensityOptimizer {
 public:
    static constexpr size_t DEFAULT_ITERATIONS = 100;
    static constexpr double CONVERGENCE_THRESHOLD = 1e-8;
    static constexpr double INITIAL_STEP_SIZE = 0.1;
    static constexpr double MAX_STEP_SIZE = 1.0;
    static constexpr double MIN_STEP_SIZE = 1e-10;
    static constexpr double STEP_SHRINK_FACTOR = 0.5;
    static constexpr double STEP_GROW_FACTOR = 1.2;

    static constexpr double PERTURBATION_ANGLE = 0.05;
    static constexpr size_t MAX_PERTURBATION_ATTEMPTS = 50;
    static constexpr size_t MAX_PERTURBATIONS_BEFORE_RESTORE = 5;
    static constexpr size_t MAX_RESTORES_PER_CHECKPOINT = 3;

    GradientDensityOptimizer(
        std::unique_ptr<VoronoiSphere> voronoi_sphere,
        FieldType field,
        size_t max_iterations = DEFAULT_ITERATIONS,
        GeneratorType point_generator = GeneratorType()
    );

    std::unique_ptr<VoronoiSphere> optimize();

 private:
    using Checkpoint = std::vector<Point3>;

    std::unique_ptr<VoronoiSphere> _voronoi_sphere;
    FieldType _field;
    size_t _max_iterations;
    double _target_mass;
    double _step_size;
    GeneratorType _point_generator;

    Eigen::VectorXd sites_to_vector() const;
    void vector_to_sites_normalized(const Eigen::VectorXd& x);

    std::vector<double> compute_mass_errors() const;
    std::vector<Eigen::Vector3d> compute_gradients(
        const std::vector<double>& mass_errors
    ) const;
    Eigen::VectorXd compute_projected_gradient() const;
    double compute_total_error(const std::vector<double>& mass_errors) const;
    double compute_error() const;

    double backtracking_line_search(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& grad,
        double current_error
    );

    Checkpoint save_checkpoint() const;
    void restore_checkpoint(const Checkpoint& checkpoint);
    std::optional<size_t> find_most_undersized_index() const;
    bool perturb_site_randomly(size_t index);
};

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
GradientDensityOptimizer<FieldType, GeneratorType>::GradientDensityOptimizer(
    std::unique_ptr<VoronoiSphere> voronoi_sphere,
    FieldType field,
    size_t max_iterations,
    GeneratorType point_generator
) :
    _voronoi_sphere(std::move(voronoi_sphere)),
    _field(std::move(field)),
    _max_iterations(max_iterations),
    _target_mass(0.0),
    _step_size(INITIAL_STEP_SIZE),
    _point_generator(std::move(point_generator)) {
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
std::unique_ptr<VoronoiSphere> GradientDensityOptimizer<FieldType, GeneratorType>::optimize() {
    _target_mass = _field.total_mass() / _voronoi_sphere->size();
    std::cout << "Target mass per cell: " << _target_mass << std::endl;

    double best_error = std::numeric_limits<double>::max();
    Checkpoint best_checkpoint = save_checkpoint();
    size_t perturbation_attempts = 0;
    size_t perturbations_since_best = 0;
    size_t restores_to_current_best = 0;

    double prev_error = std::numeric_limits<double>::max();
    size_t stall_count = 0;
    static constexpr size_t MAX_STALLS = 10;

    for (size_t iteration = 0; iteration < _max_iterations; ++iteration) {
        double error = compute_error();
        double rms_error = std::sqrt(2.0 * error / _voronoi_sphere->size());

        if (iteration % 10 == 0) {
            std::cout << std::fixed << std::setprecision(8)
                << "  Iteration " << std::setw(4) << iteration
                << ": RMS error = " << rms_error
                << ", step = " << _step_size
                << "  [perturb " << perturbation_attempts << "/" << MAX_PERTURBATION_ATTEMPTS
                << ", since_best " << perturbations_since_best << "/" << MAX_PERTURBATIONS_BEFORE_RESTORE
                << ", restores " << restores_to_current_best << "/" << MAX_RESTORES_PER_CHECKPOINT << "]"
                << std::defaultfloat << std::endl;
        }

        if (rms_error < CONVERGENCE_THRESHOLD) {
            std::cout << "Converged at iteration " << iteration << std::endl;
            break;
        }

        if (error < best_error) {
            best_error = error;
            best_checkpoint = save_checkpoint();
            perturbations_since_best = 0;
            restores_to_current_best = 0;
            stall_count = 0;
        }

        double improvement = (prev_error - error) / prev_error;
        if (improvement < 1e-6 && iteration > 0) {
            ++stall_count;
            if (stall_count >= MAX_STALLS) {
                if (perturbations_since_best >= MAX_PERTURBATIONS_BEFORE_RESTORE &&
                    restores_to_current_best < MAX_RESTORES_PER_CHECKPOINT) {
                    restore_checkpoint(best_checkpoint);
                    ++restores_to_current_best;
                    perturbations_since_best = 0;
                    stall_count = 0;
                    _step_size = INITIAL_STEP_SIZE;
                    std::cout << "  [restore #" << restores_to_current_best << "]" << std::endl;
                    continue;
                }

                if (perturbation_attempts >= MAX_PERTURBATION_ATTEMPTS) {
                    std::cout << "Stopped after " << perturbation_attempts
                        << " perturbation attempts" << std::endl;
                    break;
                }

                auto undersized = find_most_undersized_index();
                if (undersized.has_value() && perturb_site_randomly(undersized.value())) {
                    ++perturbation_attempts;
                    ++perturbations_since_best;
                    stall_count = 0;
                    _step_size = INITIAL_STEP_SIZE;
                    std::cout << "  [perturb #" << perturbation_attempts << "]" << std::endl;
                    continue;
                }
            }
        } else {
            stall_count = 0;
        }
        prev_error = error;

        Eigen::VectorXd x = sites_to_vector();
        Eigen::VectorXd grad = compute_projected_gradient();

        double step = backtracking_line_search(x, grad, error);
        if (step < MIN_STEP_SIZE) {
            ++stall_count;
            if (stall_count >= MAX_STALLS) {
                if (perturbations_since_best >= MAX_PERTURBATIONS_BEFORE_RESTORE &&
                    restores_to_current_best < MAX_RESTORES_PER_CHECKPOINT) {
                    restore_checkpoint(best_checkpoint);
                    ++restores_to_current_best;
                    perturbations_since_best = 0;
                    stall_count = 0;
                    _step_size = INITIAL_STEP_SIZE;
                    std::cout << "  [restore #" << restores_to_current_best << "]" << std::endl;
                    continue;
                }

                if (perturbation_attempts >= MAX_PERTURBATION_ATTEMPTS) {
                    std::cout << "Stopped after " << perturbation_attempts
                        << " perturbation attempts" << std::endl;
                    break;
                }

                auto undersized = find_most_undersized_index();
                if (undersized.has_value() && perturb_site_randomly(undersized.value())) {
                    ++perturbation_attempts;
                    ++perturbations_since_best;
                    stall_count = 0;
                    _step_size = INITIAL_STEP_SIZE;
                    std::cout << "  [perturb #" << perturbation_attempts << "]" << std::endl;
                    continue;
                }
            }
            _step_size = std::max(_step_size * STEP_SHRINK_FACTOR, MIN_STEP_SIZE);
            continue;
        }

        Eigen::VectorXd new_x = x - step * grad;
        vector_to_sites_normalized(new_x);

        _step_size = std::min(step * STEP_GROW_FACTOR, MAX_STEP_SIZE);
    }

    restore_checkpoint(best_checkpoint);

    double final_error = compute_error();
    double rms_error = std::sqrt(2.0 * final_error / _voronoi_sphere->size());
    std::cout << "Final RMS error: " << std::fixed << std::setprecision(8)
        << rms_error << std::defaultfloat << std::endl;

    return std::move(_voronoi_sphere);
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
double GradientDensityOptimizer<FieldType, GeneratorType>::backtracking_line_search(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& grad,
    double current_error
) {
    double step = _step_size;

    while (step > MIN_STEP_SIZE) {
        Eigen::VectorXd new_x = x - step * grad;
        vector_to_sites_normalized(new_x);
        double new_error = compute_error();

        if (new_error < current_error) {
            return step;
        }

        step *= STEP_SHRINK_FACTOR;
    }

    vector_to_sites_normalized(x);
    return 0.0;
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
Eigen::VectorXd GradientDensityOptimizer<FieldType, GeneratorType>::sites_to_vector() const {
    size_t n = _voronoi_sphere->size();
    Eigen::VectorXd x(3 * n);

    for (size_t i = 0; i < n; ++i) {
        Point3 site = _voronoi_sphere->site(i);
        x[3 * i + 0] = site.x();
        x[3 * i + 1] = site.y();
        x[3 * i + 2] = site.z();
    }

    return x;
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
void GradientDensityOptimizer<FieldType, GeneratorType>::vector_to_sites_normalized(const Eigen::VectorXd& x) {
    size_t n = _voronoi_sphere->size();

    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3d v(x[3 * i + 0], x[3 * i + 1], x[3 * i + 2]);
        v.normalize();
        _voronoi_sphere->update_site(i, Point3(v.x(), v.y(), v.z()));
    }
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
double GradientDensityOptimizer<FieldType, GeneratorType>::compute_error() const {
    auto mass_errors = compute_mass_errors();
    return compute_total_error(mass_errors);
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
Eigen::VectorXd GradientDensityOptimizer<FieldType, GeneratorType>::compute_projected_gradient() const {
    auto mass_errors = compute_mass_errors();
    auto gradients = compute_gradients(mass_errors);

    size_t n = _voronoi_sphere->size();
    Eigen::VectorXd grad(3 * n);

    for (size_t i = 0; i < n; ++i) {
        Point3 site = _voronoi_sphere->site(i);
        Eigen::Vector3d s(site.x(), site.y(), site.z());
        s.normalize();

        Eigen::Vector3d g = gradients[i];
        g -= g.dot(s) * s;

        grad[3 * i + 0] = g.x();
        grad[3 * i + 1] = g.y();
        grad[3 * i + 2] = g.z();
    }

    return grad;
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
std::vector<double> GradientDensityOptimizer<FieldType, GeneratorType>::compute_mass_errors() const {
    size_t n = _voronoi_sphere->size();
    std::vector<double> errors(n);

    size_t i = 0;
    for (const auto& cell : _voronoi_sphere->dual_cells()) {
        auto moments = compute_polygon_moments(cell);
        double mass = _field.mass(moments);
        errors[i] = mass - _target_mass;
        ++i;
    }

    return errors;
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
std::vector<Eigen::Vector3d> GradientDensityOptimizer<FieldType, GeneratorType>::compute_gradients(
    const std::vector<double>& mass_errors
) const {
    size_t n = _voronoi_sphere->size();
    std::vector<Eigen::Vector3d> gradients(n, Eigen::Vector3d::Zero());

    for (size_t k = 0; k < n; ++k) {
        Point3 site_k = _voronoi_sphere->site(k);
        auto cell_edges = _voronoi_sphere->cell_edges(k);

        for (const auto& edge_info : cell_edges) {
            size_t j = edge_info.neighbor_index;
            Point3 site_j = _voronoi_sphere->site(j);

            Point3 v1 = to_point(edge_info.arc.source());
            Point3 v2 = to_point(edge_info.arc.target());

            auto arc_moments = compute_arc_moments(v1, v2);
            Eigen::Vector3d rho_weighted_moment = _field.edge_gradient_integral(arc_moments);

            Eigen::Vector3d n_vec = to_eigen(site_j) - to_eigen(site_k);
            double n_norm = n_vec.norm();

            if (n_norm < GEOMETRIC_EPSILON) {
                continue;
            }

            Eigen::Vector3d edge_grad = rho_weighted_moment / n_norm;
            gradients[k] += (mass_errors[k] - mass_errors[j]) * edge_grad;
        }
    }

    return gradients;
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
double GradientDensityOptimizer<FieldType, GeneratorType>::compute_total_error(
    const std::vector<double>& mass_errors
) const {
    double sum = 0.0;
    for (double e : mass_errors) {
        sum += e * e;
    }
    return sum / 2.0;
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
typename GradientDensityOptimizer<FieldType, GeneratorType>::Checkpoint
GradientDensityOptimizer<FieldType, GeneratorType>::save_checkpoint() const {
    Checkpoint checkpoint;
    checkpoint.reserve(_voronoi_sphere->size());

    for (size_t i = 0; i < _voronoi_sphere->size(); ++i) {
        checkpoint.push_back(_voronoi_sphere->site(i));
    }

    return checkpoint;
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
void GradientDensityOptimizer<FieldType, GeneratorType>::restore_checkpoint(const Checkpoint& checkpoint) {
    for (size_t i = 0; i < checkpoint.size(); ++i) {
        _voronoi_sphere->update_site(i, checkpoint[i]);
    }
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
std::optional<size_t> GradientDensityOptimizer<FieldType, GeneratorType>::find_most_undersized_index() const {
    auto mass_errors = compute_mass_errors();

    size_t best_index = 0;
    double largest_deficit = 0.0;
    double largest_abs_error = 0.0;
    size_t largest_error_index = 0;

    for (size_t i = 0; i < mass_errors.size(); ++i) {
        double deficit = -mass_errors[i];
        if (deficit > largest_deficit) {
            largest_deficit = deficit;
            best_index = i;
        }
        double abs_error = std::abs(mass_errors[i]);
        if (abs_error > largest_abs_error) {
            largest_abs_error = abs_error;
            largest_error_index = i;
        }
    }

    if (largest_deficit > 0.0) {
        return best_index;
    }

    if (largest_abs_error > 0.0) {
        return largest_error_index;
    }

    return std::nullopt;
}

template<SphericalField FieldType, SpherePointGenerator GeneratorType>
bool GradientDensityOptimizer<FieldType, GeneratorType>::perturb_site_randomly(size_t index) {
    Point3 site = _voronoi_sphere->site(index);
    Eigen::Vector3d s(site.x(), site.y(), site.z());
    s.normalize();

    auto random_points = _point_generator.generate(1);
    if (random_points.empty()) {
        return false;
    }

    Point3 random_point = random_points[0];
    Eigen::Vector3d random_dir(random_point.x(), random_point.y(), random_point.z());

    Eigen::Vector3d tangent = random_dir - random_dir.dot(s) * s;
    double tangent_norm = tangent.norm();

    if (tangent_norm < GEOMETRIC_EPSILON) {
        return false;
    }

    tangent /= tangent_norm;

    Eigen::Vector3d new_pos =
        std::cos(PERTURBATION_ANGLE) * s +
        std::sin(PERTURBATION_ANGLE) * tangent;

    new_pos.normalize();
    _voronoi_sphere->update_site(index, Point3(new_pos.x(), new_pos.y(), new_pos.z()));

    return true;
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_
