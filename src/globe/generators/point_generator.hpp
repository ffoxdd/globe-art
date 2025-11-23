#ifndef GLOBEART_SRC_GLOBE_GENERATORS_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_POINT_GENERATOR_HPP_

#include "../types.hpp"
#include "../geometry/cartesian/bounding_box.hpp"
#include <concepts>

namespace globe {

template<typename T>
concept PointGenerator = requires(T t, const BoundingBox &bbox) {
    { t.generate() } -> std::same_as<Point3>;
    { t.generate(bbox) } -> std::same_as<Point3>;
};

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_POINT_GENERATOR_HPP_
