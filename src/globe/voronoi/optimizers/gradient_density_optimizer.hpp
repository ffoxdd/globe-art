#ifndef GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"
#include "../../fields/spherical/spherical_field.hpp"
#include "../../fields/spherical/constant_spherical_field.hpp"
#include "../core/voronoi_sphere.hpp"
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace globe {

template<SphericalField FieldType = ConstantSphericalField>
class GradientDensityOptimizer {
 public:
    static constexpr size_t DEFAULT_ITERATIONS = 100;
    static constexpr double CONVERGENCE_THRESHOLD = 1e-8;
    static constexpr double INITIAL_STEP_SIZE = 0.1;
    static constexpr double MAX_STEP_SIZE = 1.0;
    static constexpr double MIN_STEP_SIZE = 1e-10;
    static constexpr double STEP_SHRINK_FACTOR = 0.5;
    static constexpr double STEP_GROW_FACTOR = 1.2;

    GradientDensityOptimizer(
        std::unique_ptr<VoronoiSphere> voronoi_sphere,
        FieldType field,
        size_t max_iterations = DEFAULT_ITERATIONS,
        double step_size = 0.1
    );

    std::unique_ptr<VoronoiSphere> optimize();

 private:
    std::unique_ptr<VoronoiSphere> _voronoi_sphere;
    FieldType _field;
    size_t _max_iterations;
    double _target_mass;
    double _step_size;

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

    static Eigen::Vector3d compute_edge_sensitivity(
        const Point3& site_i,
        const Point3& site_j,
        const Point3& edge_v1,
        const Point3& edge_v2
    );
};

template<SphericalField FieldType>
GradientDensityOptimizer<FieldType>::GradientDensityOptimizer(
    std::unique_ptr<VoronoiSphere> voronoi_sphere,
    FieldType field,
    size_t max_iterations,
    [[maybe_unused]] double step_size
) :
    _voronoi_sphere(std::move(voronoi_sphere)),
    _field(std::move(field)),
    _max_iterations(max_iterations),
    _target_mass(0.0),
    _step_size(INITIAL_STEP_SIZE) {
}

template<SphericalField FieldType>
std::unique_ptr<VoronoiSphere> GradientDensityOptimizer<FieldType>::optimize() {
    _target_mass = _field.total_mass() / _voronoi_sphere->size();
    std::cout << "Target mass per cell: " << _target_mass << std::endl;

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
                << std::defaultfloat << std::endl;
        }

        if (rms_error < CONVERGENCE_THRESHOLD) {
            std::cout << "Converged at iteration " << iteration << std::endl;
            break;
        }

        double improvement = (prev_error - error) / prev_error;
        if (improvement < 1e-6 && iteration > 0) {
            ++stall_count;
            if (stall_count >= MAX_STALLS) {
                std::cout << "Stalled at iteration " << iteration << std::endl;
                break;
            }
        } else {
            stall_count = 0;
        }
        prev_error = error;

        Eigen::VectorXd x = sites_to_vector();
        Eigen::VectorXd grad = compute_projected_gradient();

        double step = backtracking_line_search(x, grad, error);
        if (step < MIN_STEP_SIZE) {
            _step_size = std::max(_step_size * STEP_SHRINK_FACTOR, MIN_STEP_SIZE);
            continue;
        }

        Eigen::VectorXd new_x = x - step * grad;
        vector_to_sites_normalized(new_x);

        _step_size = std::min(step * STEP_GROW_FACTOR, MAX_STEP_SIZE);
    }

    double final_error = compute_error();
    double rms_error = std::sqrt(2.0 * final_error / _voronoi_sphere->size());
    std::cout << "Final RMS error: " << std::fixed << std::setprecision(8)
        << rms_error << std::defaultfloat << std::endl;

    return std::move(_voronoi_sphere);
}

template<SphericalField FieldType>
double GradientDensityOptimizer<FieldType>::backtracking_line_search(
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

template<SphericalField FieldType>
Eigen::VectorXd GradientDensityOptimizer<FieldType>::sites_to_vector() const {
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

template<SphericalField FieldType>
void GradientDensityOptimizer<FieldType>::vector_to_sites_normalized(const Eigen::VectorXd& x) {
    size_t n = _voronoi_sphere->size();

    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3d v(x[3 * i + 0], x[3 * i + 1], x[3 * i + 2]);
        v.normalize();
        _voronoi_sphere->update_site(i, Point3(v.x(), v.y(), v.z()));
    }
}

template<SphericalField FieldType>
double GradientDensityOptimizer<FieldType>::compute_error() const {
    auto mass_errors = compute_mass_errors();
    return compute_total_error(mass_errors);
}

template<SphericalField FieldType>
Eigen::VectorXd GradientDensityOptimizer<FieldType>::compute_projected_gradient() const {
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

template<SphericalField FieldType>
std::vector<double> GradientDensityOptimizer<FieldType>::compute_mass_errors() const {
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

template<SphericalField FieldType>
std::vector<Eigen::Vector3d> GradientDensityOptimizer<FieldType>::compute_gradients(
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
            double rho_integral = _field.edge_integral(arc_moments);
            Eigen::Vector3d sensitivity = compute_edge_sensitivity(
                site_k, site_j, v1, v2
            );

            Eigen::Vector3d edge_grad = rho_integral * sensitivity;
            gradients[k] += (mass_errors[k] - mass_errors[j]) * edge_grad;
        }
    }

    return gradients;
}

template<SphericalField FieldType>
double GradientDensityOptimizer<FieldType>::compute_total_error(
    const std::vector<double>& mass_errors
) const {
    double sum = 0.0;
    for (double e : mass_errors) {
        sum += e * e;
    }
    return sum / 2.0;
}

template<SphericalField FieldType>
Eigen::Vector3d GradientDensityOptimizer<FieldType>::compute_edge_sensitivity(
    const Point3& site_i,
    const Point3& site_j,
    const Point3& edge_v1,
    const Point3& edge_v2
) {
    Eigen::Vector3d si = to_eigen(site_i).normalized();
    Eigen::Vector3d sj = to_eigen(site_j).normalized();

    Eigen::Vector3d toward_j = sj - si;

    Eigen::Vector3d tangent = toward_j - si.dot(toward_j) * si;
    double tangent_norm = tangent.norm();

    if (tangent_norm < GEOMETRIC_EPSILON) {
        return Eigen::Vector3d::Zero();
    }

    tangent /= tangent_norm;

    Eigen::Vector3d v1 = to_eigen(edge_v1).normalized();
    Eigen::Vector3d v2 = to_eigen(edge_v2).normalized();

    double cos_angle = v1.dot(v2);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double edge_length = std::acos(cos_angle);

    return 0.5 * edge_length * tangent;
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_
