#ifndef GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_

#include "../core/voronoi_sphere.hpp"
#include "../core/random_voronoi_sphere_builder.hpp"
#include "../optimizers/density_voronoi_sphere_optimizer.hpp"
#include "../optimizers/gradient_density_optimizer.hpp"
#include "../optimizers/lloyd_voronoi_sphere_optimizer.hpp"
#include "../../fields/scalar/noise_field.hpp"
#include "../../fields/integrable/constant_integrable_field.hpp"
#include "../../fields/integrable/sampled_integrable_field.hpp"
#include "../../fields/spherical/constant_spherical_field.hpp"
#include "../../fields/spherical/linear_spherical_field.hpp"
#include "../../generators/sphere_point_generator/rejection_sampling_sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/poisson_sphere_point_generator/poisson_sphere_point_generator.hpp"
#include <string>
#include <memory>
#include <algorithm>
#include <iostream>

namespace globe {

class VoronoiSphereFactory {
 public:
    VoronoiSphereFactory(
        int points_count,
        std::string density_function,
        std::string optimization_strategy,
        int optimization_passes,
        int lloyd_passes
    );

    std::unique_ptr<VoronoiSphere> build();

 private:
    using NoiseGenerator = PoissonSpherePointGenerator<RejectionSamplingSpherePointGenerator<NoiseField>>;

    static constexpr double NYQUIST_SAFETY_FACTOR = 4.0;
    static constexpr size_t MIN_SAMPLES = 1000;

    int _points_count;
    std::string _density_function;
    std::string _optimization_strategy;
    size_t _optimization_passes;
    size_t _lloyd_passes;

    std::unique_ptr<VoronoiSphere> build_initial();
    std::unique_ptr<VoronoiSphere> optimize_density(std::unique_ptr<VoronoiSphere> voronoi_sphere);

    std::unique_ptr<VoronoiSphere> optimize_ccvd(std::unique_ptr<VoronoiSphere> voronoi_sphere);
    std::unique_ptr<VoronoiSphere> optimize_gradient(std::unique_ptr<VoronoiSphere> voronoi_sphere);

    std::unique_ptr<VoronoiSphere> optimize_constant_ccvd(std::unique_ptr<VoronoiSphere> voronoi_sphere);
    std::unique_ptr<VoronoiSphere> optimize_noise_ccvd(std::unique_ptr<VoronoiSphere> voronoi_sphere);
    std::unique_ptr<VoronoiSphere> optimize_constant_gradient(std::unique_ptr<VoronoiSphere> voronoi_sphere);
    std::unique_ptr<VoronoiSphere> optimize_linear_gradient(std::unique_ptr<VoronoiSphere> voronoi_sphere);

    static double nyquist_density(double max_frequency);
    static size_t sample_count(double max_frequency);
};

inline VoronoiSphereFactory::VoronoiSphereFactory(
    int points_count,
    std::string density_function,
    std::string optimization_strategy,
    int optimization_passes,
    int lloyd_passes
) :
    _points_count(points_count),
    _density_function(std::move(density_function)),
    _optimization_strategy(std::move(optimization_strategy)),
    _optimization_passes(static_cast<size_t>(optimization_passes)),
    _lloyd_passes(static_cast<size_t>(lloyd_passes)) {
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::build() {
    auto voronoi_sphere = build_initial();

    voronoi_sphere = optimize_density(std::move(voronoi_sphere));

    for (size_t i = 0; i < _lloyd_passes; i++) {
        LloydVoronoiSphereOptimizer lloyd_optimizer(std::move(voronoi_sphere), 1);
        voronoi_sphere = lloyd_optimizer.optimize();

        voronoi_sphere = optimize_density(std::move(voronoi_sphere));
    }

    return voronoi_sphere;
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_density(
    std::unique_ptr<VoronoiSphere> voronoi_sphere
) {
    if (_optimization_strategy == "gradient") {
        return optimize_gradient(std::move(voronoi_sphere));
    } else {
        return optimize_ccvd(std::move(voronoi_sphere));
    }
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_ccvd(
    std::unique_ptr<VoronoiSphere> voronoi_sphere
) {
    if (_density_function == "constant") {
        return optimize_constant_ccvd(std::move(voronoi_sphere));
    } else {
        return optimize_noise_ccvd(std::move(voronoi_sphere));
    }
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_gradient(
    std::unique_ptr<VoronoiSphere> voronoi_sphere
) {
    if (_density_function == "constant") {
        return optimize_constant_gradient(std::move(voronoi_sphere));
    } else if (_density_function == "linear") {
        return optimize_linear_gradient(std::move(voronoi_sphere));
    } else {
        std::cerr << "Gradient optimization not yet implemented for noise field" << std::endl;
        return voronoi_sphere;
    }
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::build_initial() {
    RandomVoronoiSphereBuilder<> builder;
    return builder.build(_points_count);
}

inline double VoronoiSphereFactory::nyquist_density(double max_frequency) {
    return 4.0 * max_frequency * max_frequency;
}

inline size_t VoronoiSphereFactory::sample_count(double max_frequency) {
    double density = nyquist_density(max_frequency);
    auto count = static_cast<size_t>(UNIT_SPHERE_AREA * density * NYQUIST_SAFETY_FACTOR);
    return std::max(count, MIN_SAMPLES);
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_constant_ccvd(
    std::unique_ptr<VoronoiSphere> voronoi_sphere
) {
    auto integrable_field = std::make_unique<ConstantIntegrableField>();

    DensityVoronoiSphereOptimizer<ConstantIntegrableField> optimizer(
        std::move(voronoi_sphere),
        std::move(integrable_field),
        _optimization_passes
    );

    return optimizer.optimize();
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_noise_ccvd(
    std::unique_ptr<VoronoiSphere> voronoi_sphere
) {
    NoiseField noise_field;
    size_t samples = sample_count(noise_field.max_frequency());
    std::cout << "Sample count: " << samples << std::endl;

    RejectionSamplingSpherePointGenerator<NoiseField> rejection_generator(noise_field, 1.0);
    NoiseGenerator generator(std::move(rejection_generator));

    auto integrable_field = std::make_unique<SampledIntegrableField<NoiseGenerator>>(
        std::move(generator),
        samples,
        1.0
    );

    DensityVoronoiSphereOptimizer optimizer(
        std::move(voronoi_sphere),
        std::move(integrable_field),
        _optimization_passes
    );

    return optimizer.optimize();
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_constant_gradient(
    std::unique_ptr<VoronoiSphere> voronoi_sphere
) {
    ConstantSphericalField field(1.0);

    GradientDensityOptimizer optimizer(
        std::move(voronoi_sphere),
        field,
        _optimization_passes
    );

    return optimizer.optimize();
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_linear_gradient(
    std::unique_ptr<VoronoiSphere> voronoi_sphere
) {
    LinearSphericalField field(1.0, 2.0);

    GradientDensityOptimizer optimizer(
        std::move(voronoi_sphere),
        field,
        _optimization_passes
    );

    return optimizer.optimize();
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_

