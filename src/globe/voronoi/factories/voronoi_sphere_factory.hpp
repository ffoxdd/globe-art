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

    VoronoiSphere create();

 private:
    int _points_count;
    std::string _density_function;
    int _optimization_passes;

    template<typename SF>
    VoronoiSphere create_with_field();

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
    _optimization_passes(optimization_passes) {
}

inline VoronoiSphere VoronoiSphereFactory::create() {
    if (_density_function == "constant") {
        return create_with_field<ConstantScalarField>();
    } else {
        return create_with_field<NoiseField>();
    }
}

template<typename SF>
inline VoronoiSphere VoronoiSphereFactory::create_with_field() {
    RandomVoronoiSphereBuilder<> builder{};
    VoronoiSphere initial_voronoi = builder.build(_points_count);

    auto integrable_field = build_integrable_field<SF>();

    DensityVoronoiSphereOptimizer optimizer(
        std::move(initial_voronoi),
        std::move(integrable_field)
    );

    return optimizer.optimize(static_cast<size_t>(_optimization_passes));
}

template<typename SF>
inline std::unique_ptr<DensitySampledIntegrableField<SF>> VoronoiSphereFactory::build_integrable_field() {
    SF density_field;

    SphericalRandomPointGenerator point_generator;
    std::vector<Point3> sample_points;
    for (int i = 0; i < 1000; i++) {
        sample_points.push_back(point_generator.generate());
    }
    density_field.normalize(sample_points);

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

