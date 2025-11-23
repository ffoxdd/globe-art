#ifndef GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_

#include "voronoi_sphere.hpp"
#include "../../generators/sphere_point_generator.hpp"
#include "../../generators/random_sphere_point_generator.hpp"
#include <memory>

namespace globe {

template<SpherePointGenerator PG = RandomSpherePointGenerator<>>
class RandomVoronoiSphereBuilder {
 public:
    RandomVoronoiSphereBuilder(PG point_generator = PG());
    [[nodiscard]] std::unique_ptr<VoronoiSphere> build(int point_count);

 private:
    PG _point_generator;
};

template<SpherePointGenerator PG>
RandomVoronoiSphereBuilder<PG>::RandomVoronoiSphereBuilder(PG point_generator) :
    _point_generator(std::move(point_generator)) {
}

template<SpherePointGenerator PG>
std::unique_ptr<VoronoiSphere> RandomVoronoiSphereBuilder<PG>::build(int point_count) {
    auto voronoi_sphere = std::make_unique<VoronoiSphere>();

    for (int i = 0; i < point_count; i++) {
        voronoi_sphere->insert(_point_generator.generate());
    }

    return voronoi_sphere;
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_

