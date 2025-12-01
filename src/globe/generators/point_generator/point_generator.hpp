#ifndef GLOBEART_SRC_GLOBE_GENERATORS_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_POINT_GENERATOR_HPP_

#include "../../cgal_types.hpp"
#include "../../geometry/cartesian/bounding_box.hpp"
#include <vector>
#include <concepts>

namespace globe {

template<typename T>
concept PointGenerator = requires(T t, const BoundingBox &bounding_box, size_t count) {
    { t.generate(count) } -> std::convertible_to<std::vector<Point3>>;
    { t.generate(count, bounding_box) } -> std::convertible_to<std::vector<Point3>>;
};

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_POINT_GENERATOR_HPP_
