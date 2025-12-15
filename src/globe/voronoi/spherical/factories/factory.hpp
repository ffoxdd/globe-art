#ifndef GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_FACTORIES_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_FACTORIES_FACTORY_HPP_

#include "../core/sphere.hpp"
#include "../core/random_builder.hpp"
#include "../core/callback.hpp"
#include "../optimizers/field_density_optimizer.hpp"
#include "../optimizers/gradient_density_optimizer.hpp"
#include "../optimizers/lloyd_optimizer.hpp"
#include "../../../fields/scalar/noise_field.hpp"
#include "../../../fields/spherical/constant_field.hpp"
#include "../../../fields/spherical/linear_field.hpp"
#include "../../../fields/spherical/harmonic_field.hpp"
#include "../../../fields/spherical/sampled_field.hpp"
#include "../../../generators/spherical/random_point_generator.hpp"
#include <string>
#include <memory>
#include <variant>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace globe::voronoi::spherical {

using fields::scalar::NoiseField;
using fields::spherical::ConstantField;
using fields::spherical::LinearField;
using fields::spherical::HarmonicField;
using fields::spherical::SampledField;
using geometry::spherical::UNIT_SPHERE_AREA;

using AnalyticField = std::variant<ConstantField, LinearField, HarmonicField>;
using SampledNoiseField = SampledField<NoiseField>;

class Factory {
 public:
    Factory(
        int points_count,
        std::string density_function,
        std::string optimization_strategy,
        int optimization_passes,
        int lloyd_passes,
        int max_perturbations = 50,
        Callback callback = noop_callback()
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
    Callback _callback;

    std::unique_ptr<Sphere> build_initial();
    std::unique_ptr<Sphere> optimize_density(std::unique_ptr<Sphere> sphere);

    AnalyticField create_analytic_field() const;

    template<fields::spherical::Field FieldType>
    std::unique_ptr<Sphere> optimize_ccvd(std::unique_ptr<Sphere> sphere, FieldType& field);

    template<fields::spherical::Field FieldType>
    std::unique_ptr<Sphere> optimize_gradient(std::unique_ptr<Sphere> sphere, FieldType& field);

    std::unique_ptr<Sphere> optimize_noise_ccvd(std::unique_ptr<Sphere> sphere);

    static double nyquist_density(double max_frequency);
    static size_t sample_count(double max_frequency);
};

inline Factory::Factory(
    int points_count,
    std::string density_function,
    std::string optimization_strategy,
    int optimization_passes,
    int lloyd_passes,
    int max_perturbations,
    Callback callback
) :
    _points_count(points_count),
    _density_function(std::move(density_function)),
    _optimization_strategy(std::move(optimization_strategy)),
    _optimization_passes(static_cast<size_t>(optimization_passes)),
    _lloyd_passes(static_cast<size_t>(lloyd_passes)),
    _max_perturbations(static_cast<size_t>(max_perturbations)),
    _callback(std::move(callback)) {
}

inline std::unique_ptr<Sphere> Factory::build() {
    std::cout << "Generating " << _points_count << " random points..." << std::flush;
    auto sphere = build_initial();
    std::cout << " done" << std::endl;

    _callback(*sphere);

    std::cout << "Initial density optimization..." << std::endl;
    sphere = optimize_density(std::move(sphere));

    for (size_t i = 0; i < _lloyd_passes; i++) {
        LloydOptimizer lloyd_optimizer(std::move(sphere), 1, _callback);
        sphere = lloyd_optimizer.optimize();
        double deviation = lloyd_optimizer.final_deviation();

        std::cout << "  " << std::setw(8) << std::left << "Lloyd" << std::right <<
            std::setw(4) << (i + 1) << "/" <<
            std::setw(4) << std::left << _lloyd_passes << std::right <<
            ": dev " << std::fixed << std::setprecision(8) << deviation <<
            std::defaultfloat << std::endl;

        sphere = optimize_density(std::move(sphere));
    }

    return sphere;
}

inline AnalyticField Factory::create_analytic_field() const {
    if (_density_function == "constant") {
        return ConstantField(1.0);
    } else if (_density_function == "linear") {
        return LinearField(2.0, 2.0);
    } else {
        // Equator-dense, pole-sparse: f(p) = 1 - 0.9*z²
        Eigen::Matrix3d quadratic = Eigen::Matrix3d::Zero();
        quadratic(2, 2) = -0.9;
        return HarmonicField(1.0, Eigen::Vector3d::Zero(), quadratic);
    }
}

inline std::unique_ptr<Sphere> Factory::optimize_density(
    std::unique_ptr<Sphere> sphere
) {
    if (_density_function == "noise") {
        return optimize_noise_ccvd(std::move(sphere));
    }

    auto field = create_analytic_field();

    return std::visit([this, &sphere](auto& f) mutable {
        if (_optimization_strategy == "gradient") {
            return optimize_gradient(std::move(sphere), f);
        } else {
            return optimize_ccvd(std::move(sphere), f);
        }
    }, field);
}

inline std::unique_ptr<Sphere> Factory::build_initial() {
    RandomBuilder<> builder;
    return builder.build(_points_count);
}

inline double Factory::nyquist_density(double max_frequency) {
    return 4.0 * max_frequency * max_frequency;
}

inline size_t Factory::sample_count(double max_frequency) {
    double density = nyquist_density(max_frequency);
    auto count = static_cast<size_t>(UNIT_SPHERE_AREA * density * NYQUIST_SAFETY_FACTOR);
    return std::max(count, MIN_SAMPLES);
}

template<fields::spherical::Field FieldType>
std::unique_ptr<Sphere> Factory::optimize_ccvd(
    std::unique_ptr<Sphere> sphere,
    FieldType& field
) {
    FieldDensityOptimizer optimizer(
        std::move(sphere),
        field,
        _optimization_passes,
        generators::spherical::RandomPointGenerator<>(),
        _callback
    );

    return optimizer.optimize();
}

template<fields::spherical::Field FieldType>
std::unique_ptr<Sphere> Factory::optimize_gradient(
    std::unique_ptr<Sphere> sphere,
    FieldType& field
) {
    GradientDensityOptimizer<FieldType> optimizer(
        std::move(sphere),
        field,
        _optimization_passes,
        _max_perturbations,
        generators::spherical::RandomPointGenerator<>(),
        _callback
    );

    return optimizer.optimize();
}

inline std::unique_ptr<Sphere> Factory::optimize_noise_ccvd(
    std::unique_ptr<Sphere> sphere
) {
    NoiseField noise_field;
    size_t samples = sample_count(noise_field.max_frequency());
    std::cout << "Sample count: " << samples << std::endl;

    SampledNoiseField field(noise_field, generators::spherical::RandomPointGenerator<>(), samples);

    FieldDensityOptimizer optimizer(
        std::move(sphere),
        std::move(field),
        _optimization_passes,
        generators::spherical::RandomPointGenerator<>(),
        _callback
    );

    return optimizer.optimize();
}

} // namespace globe::voronoi::spherical

#endif //GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_FACTORIES_FACTORY_HPP_
