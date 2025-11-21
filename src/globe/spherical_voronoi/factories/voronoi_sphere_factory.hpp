#ifndef GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_

#include "../core/voronoi_sphere.hpp"
#include "../core/random_voronoi_sphere_builder.hpp"
#include "../optimizers/density_voronoi_sphere_optimizer.hpp"
#include "../../fields/scalar/constant_scalar_field.hpp"
#include "../../fields/scalar/noise_field.hpp"
#include "../../fields/integrable/density_sampled_integrable_field.hpp"
#include "../../generators/spherical_random_point_generator.hpp"
#include <string>
#include <memory>
#include <vector>
#include <algorithm>

namespace globe {

class VoronoiSphereFactory {
 public:
    VoronoiSphereFactory(
        int points_count,
        std::string density_function,
        int optimization_passes
    );

    VoronoiSphere build();

 private:
    int _points_count;
    std::string _density_function;
    size_t _optimization_passes;

    VoronoiSphere build_initial();

    template<typename SF>
    VoronoiSphere optimize(VoronoiSphere voronoi_sphere);

    template<typename SF>
    std::unique_ptr<DensitySampledIntegrableField<SF>> build_integrable_field();
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

inline VoronoiSphere VoronoiSphereFactory::build() {
    VoronoiSphere initial_voronoi_sphere = build_initial();

    if (_density_function == "constant") {
        return optimize<ConstantScalarField>(std::move(initial_voronoi_sphere));
    } else {
        return optimize<NoiseField>(std::move(initial_voronoi_sphere));
    }
}

inline VoronoiSphere VoronoiSphereFactory::build_initial() {
    RandomVoronoiSphereBuilder<> builder;
    return builder.build(_points_count);
}

template<typename SF>
inline VoronoiSphere VoronoiSphereFactory::optimize(VoronoiSphere voronoi_sphere) {
    auto integrable_field = build_integrable_field<SF>();

    DensityVoronoiSphereOptimizer optimizer(
        std::move(voronoi_sphere),
        std::move(integrable_field)
    );

    return optimizer.optimize(_optimization_passes);
}

template<typename SF>
inline std::unique_ptr<DensitySampledIntegrableField<SF>> VoronoiSphereFactory::build_integrable_field() {
    SF density_field;

    size_t target_samples = std::max(
        static_cast<size_t>(60'000),
        static_cast<size_t>(_points_count) * 3'000
    );

    return std::make_unique<DensitySampledIntegrableField<SF>>(
        density_field,
        target_samples
    );
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_FACTORIES_VORONOI_SPHERE_FACTORY_HPP_

