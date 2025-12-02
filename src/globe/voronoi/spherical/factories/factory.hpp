#ifndef GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_FACTORIES_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_FACTORIES_FACTORY_HPP_

#include "../core/sphere.hpp"
#include "../core/random_builder.hpp"
#include "../optimizers/field_density_optimizer.hpp"
#include "../optimizers/gradient_density_optimizer.hpp"
#include "../optimizers/lloyd_optimizer.hpp"
#include "../../../fields/scalar/noise_field.hpp"
#include "../../../fields/spherical/constant_field.hpp"
#include "../../../fields/spherical/linear_field.hpp"
#include "../../../fields/spherical/sampled_field.hpp"
#include <string>
#include <memory>
#include <algorithm>
#include <iostream>

namespace globe::voronoi::spherical {

using fields::scalar::NoiseField;
using fields::spherical::ConstantField;
using fields::spherical::LinearField;
using fields::spherical::SampledField;
using geometry::spherical::UNIT_SPHERE_AREA;

class Factory {
 public:
    Factory(
        int points_count,
        std::string density_function,
        std::string optimization_strategy,
        int optimization_passes,
        int lloyd_passes,
        int max_perturbations = 50
    );

    std::unique_ptr<Sphere> build();

 private:
    static constexpr double NYQUIST_SAFETY_FACTOR = 4.0;
    static constexpr size_t MIN_SAMPLES = 1000;

    int _points_count;
    std::string _density_function;
    std::string _optimization_strategy;
    size_t _optimization_passes;
    size_t _lloyd_passes;
    size_t _max_perturbations;

    std::unique_ptr<Sphere> build_initial();
    std::unique_ptr<Sphere> optimize_density(std::unique_ptr<Sphere> sphere);

    std::unique_ptr<Sphere> optimize_ccvd(std::unique_ptr<Sphere> sphere);
    std::unique_ptr<Sphere> optimize_gradient(std::unique_ptr<Sphere> sphere);

    std::unique_ptr<Sphere> optimize_constant_ccvd(std::unique_ptr<Sphere> sphere);
    std::unique_ptr<Sphere> optimize_linear_ccvd(std::unique_ptr<Sphere> sphere);
    std::unique_ptr<Sphere> optimize_noise_ccvd(std::unique_ptr<Sphere> sphere);
    std::unique_ptr<Sphere> optimize_constant_gradient(std::unique_ptr<Sphere> sphere);
    std::unique_ptr<Sphere> optimize_linear_gradient(std::unique_ptr<Sphere> sphere);

    static double nyquist_density(double max_frequency);
    static size_t sample_count(double max_frequency);
};

inline Factory::Factory(
    int points_count,
    std::string density_function,
    std::string optimization_strategy,
    int optimization_passes,
    int lloyd_passes,
    int max_perturbations
) :
    _points_count(points_count),
    _density_function(std::move(density_function)),
    _optimization_strategy(std::move(optimization_strategy)),
    _optimization_passes(static_cast<size_t>(optimization_passes)),
    _lloyd_passes(static_cast<size_t>(lloyd_passes)),
    _max_perturbations(static_cast<size_t>(max_perturbations)) {
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::build() {
    auto sphere = build_initial();

    sphere = optimize_density(std::move(sphere));

    for (size_t i = 0; i < _lloyd_passes; i++) {
        LloydOptimizer lloyd_optimizer(std::move(sphere), 1);
        sphere = lloyd_optimizer.optimize();

        sphere = optimize_density(std::move(sphere));
    }

    return sphere;
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::optimize_density(
    std::unique_ptr<Sphere> sphere
) {
    if (_optimization_strategy == "gradient") {
        return optimize_gradient(std::move(sphere));
    } else {
        return optimize_ccvd(std::move(sphere));
    }
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::optimize_ccvd(
    std::unique_ptr<Sphere> sphere
) {
    if (_density_function == "constant") {
        return optimize_constant_ccvd(std::move(sphere));
    } else if (_density_function == "linear") {
        return optimize_linear_ccvd(std::move(sphere));
    } else {
        return optimize_noise_ccvd(std::move(sphere));
    }
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::optimize_gradient(
    std::unique_ptr<Sphere> sphere
) {
    if (_density_function == "constant") {
        return optimize_constant_gradient(std::move(sphere));
    } else if (_density_function == "linear") {
        return optimize_linear_gradient(std::move(sphere));
    } else {
        std::cerr << "Gradient optimization not yet implemented for noise field" << std::endl;
        return sphere;
    }
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::build_initial() {
    RandomBuilder<> builder;
    return builder.build(_points_count);
} // namespace globe::voronoi::spherical

inline double Factory::nyquist_density(double max_frequency) {
    return 4.0 * max_frequency * max_frequency;
} // namespace globe::voronoi::spherical

inline size_t Factory::sample_count(double max_frequency) {
    double density = nyquist_density(max_frequency);
    auto count = static_cast<size_t>(UNIT_SPHERE_AREA * density * NYQUIST_SAFETY_FACTOR);
    return std::max(count, MIN_SAMPLES);
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::optimize_constant_ccvd(
    std::unique_ptr<Sphere> sphere
) {
    ConstantField field(1.0);

    FieldDensityOptimizer optimizer(
        std::move(sphere),
        field,
        _optimization_passes
    );

    return optimizer.optimize();
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::optimize_linear_ccvd(
    std::unique_ptr<Sphere> sphere
) {
    LinearField field(2.0, 2.0);

    FieldDensityOptimizer optimizer(
        std::move(sphere),
        field,
        _optimization_passes
    );

    return optimizer.optimize();
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::optimize_noise_ccvd(
    std::unique_ptr<Sphere> sphere
) {
    NoiseField noise_field;
    size_t samples = sample_count(noise_field.max_frequency());
    std::cout << "Sample count: " << samples << std::endl;

    SampledField field(noise_field, generators::spherical::RandomPointGenerator<>(), samples);

    FieldDensityOptimizer optimizer(
        std::move(sphere),
        std::move(field),
        _optimization_passes
    );

    return optimizer.optimize();
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::optimize_constant_gradient(
    std::unique_ptr<Sphere> sphere
) {
    ConstantField field(1.0);

    GradientDensityOptimizer optimizer(
        std::move(sphere),
        field,
        _optimization_passes,
        _max_perturbations
    );

    return optimizer.optimize();
} // namespace globe::voronoi::spherical

inline std::unique_ptr<Sphere> Factory::optimize_linear_gradient(
    std::unique_ptr<Sphere> sphere
) {
    LinearField field(2.0, 2.0);

    GradientDensityOptimizer<LinearField> optimizer(
        std::move(sphere),
        field,
        _optimization_passes,
        _max_perturbations
    );

    return optimizer.optimize();
} // namespace globe::voronoi::spherical

} // namespace globe::voronoi::spherical

#endif //GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_FACTORIES_FACTORY_HPP_

