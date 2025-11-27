#ifndef GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_

#include "../core/voronoi_sphere.hpp"
#include "../core/random_voronoi_sphere_builder.hpp"
#include "../optimizers/density_voronoi_sphere_optimizer.hpp"
#include "../../fields/scalar/noise_field.hpp"
#include "../../fields/integrable/constant_integrable_field.hpp"
#include "../../fields/integrable/sampled_integrable_field.hpp"
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
        int optimization_passes
    );

    std::unique_ptr<VoronoiSphere> build();

 private:
    static constexpr double NYQUIST_SAFETY_FACTOR = 4.0;
    static constexpr size_t MIN_SAMPLES = 1000;

    int _points_count;
    std::string _density_function;
    size_t _optimization_passes;

    std::unique_ptr<VoronoiSphere> build_initial();

    using NoiseGenerator = PoissonSpherePointGenerator<RejectionSamplingSpherePointGenerator<NoiseField>>;

    std::unique_ptr<VoronoiSphere> optimize_constant(std::unique_ptr<VoronoiSphere> voronoi_sphere);
    std::unique_ptr<VoronoiSphere> optimize_noise(std::unique_ptr<VoronoiSphere> voronoi_sphere);

    static double nyquist_density(double max_frequency);
    static size_t sample_count(double max_frequency);
};

inline VoronoiSphereFactory::VoronoiSphereFactory(
    int points_count,
    std::string density_function,
    int optimization_passes
) :
    _points_count(points_count),
    _density_function(std::move(density_function)),
    _optimization_passes(static_cast<size_t>(optimization_passes)) {
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::build() {
    auto initial_voronoi_sphere = build_initial();

    if (_density_function == "constant") {
        return optimize_constant(std::move(initial_voronoi_sphere));
    } else {
        return optimize_noise(std::move(initial_voronoi_sphere));
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

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_constant(
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

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize_noise(
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

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_

