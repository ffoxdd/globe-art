#ifndef GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_

#include "voronoi_sphere.hpp"
#include "../../generators/point_generator.hpp"
#include "../../generators/spherical_random_point_generator.hpp"

namespace globe {

template<PointGenerator PG = SphericalRandomPointGenerator>
class RandomVoronoiSphereBuilder {
 public:
    RandomVoronoiSphereBuilder(PG point_generator = PG());
    [[nodiscard]] VoronoiSphere build(int point_count);

 private:
    PG _point_generator;
};

template<PointGenerator PG>
RandomVoronoiSphereBuilder<PG>::RandomVoronoiSphereBuilder(PG point_generator) :
    _point_generator(std::move(point_generator)) {
}

template<PointGenerator PG>
VoronoiSphere RandomVoronoiSphereBuilder<PG>::build(int point_count) {
    VoronoiSphere voronoi_sphere;

    for (int i = 0; i < point_count; i++) {
        voronoi_sphere.insert(_point_generator.generate());
    }

    return voronoi_sphere;
}

}

#endif //GLOBEART_SRC_GLOBE_VORONOI_CORE_RANDOM_VORONOI_SPHERE_BUILDER_HPP_

