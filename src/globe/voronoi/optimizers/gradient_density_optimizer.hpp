#ifndef GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"
#include "../../geometry/spherical/edge_sensitivity.hpp"
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
    static constexpr double DEFAULT_STEP_SIZE = 0.1;
    static constexpr double CONVERGENCE_THRESHOLD = 1e-8;

    GradientDensityOptimizer(
        std::unique_ptr<VoronoiSphere> voronoi_sphere,
        FieldType field,
        size_t max_iterations = DEFAULT_ITERATIONS,
        double step_size = DEFAULT_STEP_SIZE
    );

    std::unique_ptr<VoronoiSphere> optimize();

 private:
    std::unique_ptr<VoronoiSphere> _voronoi_sphere;
    FieldType _field;
    size_t _max_iterations;
    double _step_size;

    double compute_target_mass() const;
    std::vector<double> compute_mass_errors(double target_mass) const;
    std::vector<Eigen::Vector3d> compute_energy_gradients(
        const std::vector<double>& mass_errors
    ) const;
    void step(const std::vector<Eigen::Vector3d>& gradients);
    double compute_total_error(const std::vector<double>& mass_errors) const;
    void print_progress(size_t iteration, double error) const;
};

template<SphericalField FieldType>
GradientDensityOptimizer<FieldType>::GradientDensityOptimizer(
    std::unique_ptr<VoronoiSphere> voronoi_sphere,
    FieldType field,
    size_t max_iterations,
    double step_size
) :
    _voronoi_sphere(std::move(voronoi_sphere)),
    _field(std::move(field)),
    _max_iterations(max_iterations),
    _step_size(step_size) {
}

template<SphericalField FieldType>
std::unique_ptr<VoronoiSphere> GradientDensityOptimizer<FieldType>::optimize() {
    double target_mass = compute_target_mass();
    std::cout << "Target mass per cell: " << target_mass << std::endl;

    double prev_error = std::numeric_limits<double>::max();

    for (size_t iter = 0; iter < _max_iterations; ++iter) {
        auto mass_errors = compute_mass_errors(target_mass);
        double error = compute_total_error(mass_errors);

        print_progress(iter, error);

        if (error < CONVERGENCE_THRESHOLD) {
            std::cout << "Converged at iteration " << iter << std::endl;
            break;
        }

        if (std::abs(error - prev_error) < CONVERGENCE_THRESHOLD * prev_error) {
            std::cout << "Progress stalled at iteration " << iter << std::endl;
            break;
        }

        auto gradients = compute_energy_gradients(mass_errors);
        step(gradients);

        prev_error = error;
    }

    return std::move(_voronoi_sphere);
}

template<SphericalField FieldType>
double GradientDensityOptimizer<FieldType>::compute_target_mass() const {
    return _field.total_mass() / _voronoi_sphere->size();
}

template<SphericalField FieldType>
std::vector<double> GradientDensityOptimizer<FieldType>::compute_mass_errors(
    double target_mass
) const {
    size_t n = _voronoi_sphere->size();
    std::vector<double> errors(n);

    size_t i = 0;
    for (const auto& cell : _voronoi_sphere->dual_cells()) {
        auto moments = compute_polygon_moments(cell);
        double mass = _field.mass(moments);
        errors[i] = mass - target_mass;
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
void GradientDensityOptimizer<FieldType>::step(
    const std::vector<Eigen::Vector3d>& gradients
) {
    size_t n = _voronoi_sphere->size();

    for (size_t k = 0; k < n; ++k) {
        Point3 current = _voronoi_sphere->site(k);
        Eigen::Vector3d s(current.x(), current.y(), current.z());
        s.normalize();

        Eigen::Vector3d g = gradients[k];
        g -= g.dot(s) * s;

        Eigen::Vector3d new_pos = s - _step_size * g;
        new_pos.normalize();

        _voronoi_sphere->update_site(k, Point3(new_pos.x(), new_pos.y(), new_pos.z()));
    }
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
void GradientDensityOptimizer<FieldType>::print_progress(
    size_t iteration,
    double error
) const {
    double rms_error = std::sqrt(2.0 * error / _voronoi_sphere->size());
    std::cout << std::fixed << std::setprecision(8)
        << "  Iteration " << std::setw(4) << iteration
        << ": RMS error = " << rms_error
        << std::defaultfloat << std::endl;
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_OPTIMIZERS_GRADIENT_DENSITY_OPTIMIZER_HPP_
