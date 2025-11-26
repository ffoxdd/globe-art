#ifndef GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"

namespace globe {

template<typename T>
concept SpherePointGenerator = requires(T t, const SphericalBoundingBox& bounding_box) {
    { t.generate() } -> std::same_as<Point3>;
    { t.generate(bounding_box) } -> std::same_as<Point3>;
};

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_HPP_

