#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_CARTESIAN_BOUNDING_BOX_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_CARTESIAN_BOUNDING_BOX_SAMPLER_HPP_

#include "../../types.hpp"
#include "bounding_box.hpp"
#include <concepts>

namespace globe {

template<typename T>
concept CartesianBoundingBoxSampler = requires(T sampler, const BoundingBox& bbox) {
    { sampler.sample(bbox) } -> std::same_as<Point3>;
};

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_CARTESIAN_BOUNDING_BOX_SAMPLER_HPP_
