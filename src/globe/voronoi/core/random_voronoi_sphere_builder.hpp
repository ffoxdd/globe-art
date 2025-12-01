#ifndef GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_

#include "voronoi_sphere.hpp"
#include "../../generators/spherical/point_generator.hpp"
#include "../../generators/spherical/random_point_generator.hpp"
#include <memory>

namespace globe {

template<generators::spherical::PointGenerator SpherePointGeneratorType = generators::spherical::RandomPointGenerator<>>
class RandomVoronoiSphereBuilder {
 public:
    RandomVoronoiSphereBuilder(SpherePointGeneratorType point_generator = SpherePointGeneratorType());
    [[nodiscard]] std::unique_ptr<VoronoiSphere> build(int point_count);

 private:
    SpherePointGeneratorType _point_generator;
};

template<generators::spherical::PointGenerator SpherePointGeneratorType>
RandomVoronoiSphereBuilder<SpherePointGeneratorType>::RandomVoronoiSphereBuilder(SpherePointGeneratorType point_generator) :
    _point_generator(std::move(point_generator)) {
}

template<generators::spherical::PointGenerator SpherePointGeneratorType>
std::unique_ptr<VoronoiSphere> RandomVoronoiSphereBuilder<SpherePointGeneratorType>::build(int point_count) {
    auto voronoi_sphere = std::make_unique<VoronoiSphere>();

    for (const auto& point : _point_generator.generate(point_count)) {
        voronoi_sphere->insert(cgal::to_point(point));
    }

    return voronoi_sphere;
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_

