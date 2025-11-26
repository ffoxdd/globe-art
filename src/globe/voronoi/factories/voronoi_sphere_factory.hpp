#ifndef GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_

#include "../core/voronoi_sphere.hpp"
#include "../core/random_voronoi_sphere_builder.hpp"
#include "../optimizers/density_voronoi_sphere_optimizer.hpp"
#include "../../fields/scalar/constant_scalar_field.hpp"
#include "../../fields/scalar/noise_field.hpp"
#include "../../fields/integrable/sampled_integrable_field.hpp"
#include "../../generators/sphere_point_generator/random_sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/rejection_sampling_sphere_point_generator.hpp"
#include <string>
#include <memory>
#include <algorithm>

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
    static constexpr size_t MIN_DENSITY_FIELD_SAMPLES = 60'000;
    static constexpr size_t DENSITY_FIELD_SAMPLES_PER_POINT = 3'000;

    int _points_count;
    std::string _density_function;
    size_t _optimization_passes;

    std::unique_ptr<VoronoiSphere> build_initial();

    template<ScalarField ScalarFieldType>
    std::unique_ptr<VoronoiSphere> optimize(std::unique_ptr<VoronoiSphere> voronoi_sphere);

    template<ScalarField ScalarFieldType>
    std::unique_ptr<SampledIntegrableField<RejectionSamplingSpherePointGenerator<ScalarFieldType>>> build_integrable_field();

    size_t density_field_sample_count();
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
        return optimize<ConstantScalarField>(std::move(initial_voronoi_sphere));
    } else {
        return optimize<NoiseField>(std::move(initial_voronoi_sphere));
    }
}

inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::build_initial() {
    RandomVoronoiSphereBuilder<> builder;
    return builder.build(_points_count);
}

template<ScalarField ScalarFieldType>
inline std::unique_ptr<VoronoiSphere> VoronoiSphereFactory::optimize(std::unique_ptr<VoronoiSphere> voronoi_sphere) {
    auto integrable_field = build_integrable_field<ScalarFieldType>();

    DensityVoronoiSphereOptimizer optimizer(
        std::move(voronoi_sphere),
        std::move(integrable_field)
    );

    return optimizer.optimize(_optimization_passes);
}

inline size_t VoronoiSphereFactory::density_field_sample_count() {
    return std::max(
        MIN_DENSITY_FIELD_SAMPLES,
        static_cast<size_t>(_points_count) * DENSITY_FIELD_SAMPLES_PER_POINT
    );
}

template<ScalarField ScalarFieldType>
inline std::unique_ptr<SampledIntegrableField<RejectionSamplingSpherePointGenerator<ScalarFieldType>>> VoronoiSphereFactory::build_integrable_field() {
    ScalarFieldType density_field;
    size_t sample_count = density_field_sample_count();

    RejectionSamplingSpherePointGenerator<ScalarFieldType> generator(density_field, 1.0);

    return std::make_unique<SampledIntegrableField<RejectionSamplingSpherePointGenerator<ScalarFieldType>>>(
        std::move(generator),
        sample_count,
        1.0
    );
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_

