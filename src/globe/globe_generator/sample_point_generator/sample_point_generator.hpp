#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_SAMPLE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_SAMPLE_POINT_GENERATOR_HPP_

#include "../../types.hpp"

namespace globe {

template<typename T>
concept SamplePointGenerator = requires(T t) {
    { t.generate() } -> std::same_as<Point3>;
};

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_SAMPLE_POINT_GENERATOR_HPP_
