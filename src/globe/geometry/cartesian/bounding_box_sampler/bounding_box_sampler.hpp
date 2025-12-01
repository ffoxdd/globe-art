#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_BOUNDING_BOX_SAMPLER_BOUNDING_BOX_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_BOUNDING_BOX_SAMPLER_BOUNDING_BOX_SAMPLER_HPP_

#include "../../../cgal_types.hpp"
#include "../bounding_box.hpp"
#include <concepts>

namespace globe {

template<typename T>
concept BoundingBoxSampler = requires(T sampler, const BoundingBox& bounding_box) {
    { sampler.sample(bounding_box) } -> std::same_as<cgal::Point3>;
};

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_BOUNDING_BOX_SAMPLER_BOUNDING_BOX_SAMPLER_HPP_
