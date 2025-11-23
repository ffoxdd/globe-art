#ifndef GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include <vector>
#include <concepts>

namespace globe {

template<typename T>
concept SpherePointGenerator = requires(T t, const SphericalBoundingBox &bounding_box, size_t count) {
    { t.generate(count) } -> std::convertible_to<std::vector<Point3>>;
    { t.generate(count, bounding_box) } -> std::convertible_to<std::vector<Point3>>;
};

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_HPP_
