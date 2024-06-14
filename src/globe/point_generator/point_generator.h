#ifndef GLOBEART_SRC_GLOBE_POINT_GENERATOR_POINT_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_POINT_GENERATOR_POINT_GENERATOR_H_

#include "../types.h"

namespace globe {

template<typename T>
concept PointGenerator = requires(T t) {
    { t.generate() } -> std::same_as<Point3>;
};

}

#endif //GLOBEART_SRC_GLOBE_POINT_GENERATOR_POINT_GENERATOR_H_
