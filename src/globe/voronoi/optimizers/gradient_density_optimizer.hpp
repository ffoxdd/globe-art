#ifndef GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"
#include "../../fields/spherical/spherical_field.hpp"
#include "../../fields/spherical/constant_spherical_field.hpp"
#include "../core/voronoi_sphere.hpp"
#include <LBFGS.h>
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
    size_t _iteration_count;
    Eigen::VectorXd _best_x;
    double _best_error;

    class MassBalanceObjective;

    Eigen::VectorXd sites_to_vector() const;
    void vector_to_sites(const Eigen::VectorXd& x);
    void vector_to_sites_normalized(const Eigen::VectorXd& x);
    void project_sites_to_sphere();

    double compute_objective_and_gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad);
    void maybe_save_best(const Eigen::VectorXd& x, double error);
    std::vector<double> compute_mass_errors() const;
    std::vector<Eigen::Vector3d> compute_energy_gradients(
        const std::vector<double>& mass_errors
    ) const;
    double compute_total_error(const std::vector<double>& mass_errors) const;

    static Eigen::Vector3d compute_edge_sensitivity(
        const Point3& site_i,
        const Point3& site_j,
        const Point3& edge_v1,
        const Point3& edge_v2
    );
};

template<SphericalField FieldType>
class GradientDensityOptimizer<FieldType>::MassBalanceObjective {
 public:
    explicit MassBalanceObjective(GradientDensityOptimizer& optimizer) :
        _optimizer(optimizer) {}

    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
        _optimizer.vector_to_sites_normalized(x);
        double error = _optimizer.compute_objective_and_gradient(x, grad);
        _optimizer.maybe_save_best(x, error);
        return error;
    }

 private:
    GradientDensityOptimizer& _optimizer;
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
    _iteration_count(0),
    _best_error(std::numeric_limits<double>::max()) {
}

template<SphericalField FieldType>
std::unique_ptr<VoronoiSphere> GradientDensityOptimizer<FieldType>::optimize() {
    _target_mass = _field.total_mass() / _voronoi_sphere->size();
    std::cout << "Target mass per cell: " << _target_mass << std::endl;

    LBFGSpp::LBFGSParam<double> params;
    params.max_iterations = static_cast<int>(_max_iterations);
    params.epsilon = CONVERGENCE_THRESHOLD;
    params.epsilon_rel = 1e-5;
    params.past = 3;
    params.delta = 1e-10;
    params.max_linesearch = 40;

    LBFGSpp::LBFGSSolver<double> solver(params);
    MassBalanceObjective objective(*this);

    Eigen::VectorXd x = sites_to_vector();
    double final_error = 0.0;

    _iteration_count = 0;
    _best_x = x;
    _best_error = std::numeric_limits<double>::max();

    try {
        int iterations = solver.minimize(objective, x, final_error);
        std::cout << "L-BFGS converged in " << iterations << " iterations" << std::endl;
        _best_x = x;
    } catch (const std::runtime_error& e) {
        std::cout << "L-BFGS stopped: " << e.what() << std::endl;
    }

    vector_to_sites_normalized(_best_x);

    auto mass_errors = compute_mass_errors();
    double actual_error = compute_total_error(mass_errors);
    double rms_error = std::sqrt(2.0 * actual_error / _voronoi_sphere->size());
    std::cout << "Final RMS error: " << std::fixed << std::setprecision(8)
        << rms_error << std::defaultfloat << std::endl;

    return std::move(_voronoi_sphere);
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
void GradientDensityOptimizer<FieldType>::vector_to_sites(const Eigen::VectorXd& x) {
    size_t n = _voronoi_sphere->size();

    for (size_t i = 0; i < n; ++i) {
        Point3 new_site(x[3 * i + 0], x[3 * i + 1], x[3 * i + 2]);
        _voronoi_sphere->update_site(i, new_site);
    }
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
void GradientDensityOptimizer<FieldType>::project_sites_to_sphere() {
    size_t n = _voronoi_sphere->size();

    for (size_t i = 0; i < n; ++i) {
        Point3 site = _voronoi_sphere->site(i);
        Eigen::Vector3d v(site.x(), site.y(), site.z());
        v.normalize();
        _voronoi_sphere->update_site(i, Point3(v.x(), v.y(), v.z()));
    }
}

template<SphericalField FieldType>
void GradientDensityOptimizer<FieldType>::maybe_save_best(
    const Eigen::VectorXd& x,
    double total_error
) {
    auto mass_errors = compute_mass_errors();
    double mass_error = compute_total_error(mass_errors);
    if (mass_error < _best_error) {
        _best_error = mass_error;
        _best_x = x;
    }
}

template<SphericalField FieldType>
double GradientDensityOptimizer<FieldType>::compute_objective_and_gradient(
    const Eigen::VectorXd& x,
    Eigen::VectorXd& grad
) {
    auto mass_errors = compute_mass_errors();
    double error = compute_total_error(mass_errors);

    if (_iteration_count % 10 == 0) {
        double rms_error = std::sqrt(2.0 * error / _voronoi_sphere->size());
        std::cout << std::fixed << std::setprecision(8)
            << "  Iteration " << std::setw(4) << _iteration_count
            << ": RMS error = " << rms_error
            << std::defaultfloat << std::endl;
    }
    ++_iteration_count;

    auto gradients = compute_energy_gradients(mass_errors);

    size_t n = _voronoi_sphere->size();

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

    return error;
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
std::vector<Eigen::Vector3d> GradientDensityOptimizer<FieldType>::compute_energy_gradients(
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
