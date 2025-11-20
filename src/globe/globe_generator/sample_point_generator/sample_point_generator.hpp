#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_SAMPLE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_SAMPLE_POINT_GENERATOR_HPP_

#include <concepts>
#include "../../types.hpp"
#include "../spherical_bounding_box.hpp"

namespace globe {

template<typename T>
concept SamplePointGenerator = requires(T t, const SphericalBoundingBox &bbox) {
    { t.generate(bbox) } -> std::same_as<Point3>;
};

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_SAMPLE_POINT_GENERATOR_HPP_
